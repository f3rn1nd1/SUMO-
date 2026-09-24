# -*- coding: utf-8 -*-
"""
server.py
Coordinador ZMQ JADE -> Python/SUMO.

Responsabilidades:
- Cargar la red SUMO mediante routing_service.py.
- Entregar flota a JADE con GET_FLEET.
- Responder consultas de ruta con ROUTE.
- Recibir cronograma final de JADE y llamar a sumo_writer.py.

Estructura esperada:
- En la raíz: routing_service.py, sumo_writer.py, net.net.xml, objects.xml y trucks.xml.
- En mediano/: objects.xml y trucks.xml del escenario mediano.
- En grande/: objects.xml y trucks.xml del escenario grande.
- En config/<escenario>/: cronogramas, XML generados, runner y reportes de SUMO.
"""

import argparse
import csv
import json
import os
import sys
import traceback
from datetime import datetime
import xml.etree.ElementTree as ET
from collections import defaultdict
from typing import Any, Dict, List

import zmq

from metricas import guardar_metricas_jade, registrar_componente

from routing_service import (
    NetSumo,
    normalizar_id,
    convertir_int,
    tomar,
    archivo_existente,
    parsear_velocidad_ms,
    NET_FILE_CANDIDATOS,
    OBJECTS_FILE_CANDIDATOS,
    TRUCKS_FILE_CANDIDATOS,
    VELOCIDAD_DEF_VACIO_KMH,
    VELOCIDAD_DEF_CARGADO_KMH,
    configurar_escenario as configurar_escenario_routing,
    normalizar_escenario,
)
from sumo_writer import (
    generar_archivos_sumo,
    configurar_salida,
    CRONOGRAMA_CSV,
    CRONOGRAMA_JSON,
)

PORT = 5555
EDGES_BLOQUEADOS_ACTUALES = set()

# ============================================================
# Velocidades nominales controladas por server.py
# ============================================================
# JADE y SUMO reciben estas velocidades nominales. Los valores de trucks.xml
# se conservan como datos de respaldo, pero no gobiernan la planificación
# mientras esta opción permanezca activa.
USAR_VELOCIDAD_CAMIONES_CONTROLADA = True
VELOCIDAD_BASE_VACIO_KMH = 23.4      # 6.50 m/s
VELOCIDAD_BASE_CARGADO_KMH = 18.72   # 5.20 m/s
DIFERENCIAS_VACIO_KMH = [0.0, 0.5, 1.0]
DIFERENCIAS_CARGADO_KMH = [0.0, 0.3, 0.6]


def _indice_camion(camion: str) -> int:
    import re
    coincidencia = re.search(r"(\d+)$", str(camion or ""))
    return int(coincidencia.group(1)) if coincidencia else 0


def velocidades_camion(camion: str) -> tuple[float, float]:
    indice = _indice_camion(camion)
    diferencia_vacio = DIFERENCIAS_VACIO_KMH[indice % len(DIFERENCIAS_VACIO_KMH)]
    diferencia_cargado = DIFERENCIAS_CARGADO_KMH[indice % len(DIFERENCIAS_CARGADO_KMH)]
    velocidad_vacio = max(1.0, VELOCIDAD_BASE_VACIO_KMH - diferencia_vacio)
    velocidad_cargado = max(1.0, VELOCIDAD_BASE_CARGADO_KMH - diferencia_cargado)
    return velocidad_vacio, velocidad_cargado

# ============================================================
# Velocidades de los camiones
# ============================================================
# JADE y SUMO usan emptySpeed y loadedSpeed definidos en trucks.xml.
# Los valores por defecto se aplican únicamente cuando falta un dato.


# ============================================================
# Tiempo de viaje enviado a JADE
# ============================================================
# JADE recibe el tiempo ideal calculado con la distancia COMPLETA de los
# mismos routeEdges que posteriormente ejecutara SUMO:
#     tiempo = distancia_ruta_sumo / velocidad
#
# No se aplican factores fisicos, penalizaciones por zona, tramo
# ni cantidad de edges. El comportamiento real del trafico queda
# a cargo de SUMO y del runner TraCI.


def _texto_hijo(el: ET.Element, tag: str, default: str = "") -> str:
    child = el.find(tag)
    if child is None or child.text is None:
        return default
    return child.text.strip()


def _float_texto(el: ET.Element, tag: str, default: float = 0.0) -> float:
    try:
        return float(_texto_hijo(el, tag, str(default)).replace(",", "."))
    except Exception:
        return default


def _long_texto(el: ET.Element, tag: str, default: int = 0) -> int:
    try:
        return int(float(_texto_hijo(el, tag, str(default)).replace(",", ".")))
    except Exception:
        return default


def construir_flota(net: NetSumo) -> Dict[str, Any]:
    """
    Construye la flota desde objects.xml y trucks.xml para que JADE no lea archivos locales.
    JADE recibirá ubicaciones lógicas: PA01, Truck0, CS03, etc.
    routing_service.py internamente resuelve esas ubicaciones a edge/lane/pos.
    """
    shovels: List[Dict[str, Any]] = []
    trucks: List[Dict[str, Any]] = []
    errores: List[str] = []

    objects_path = net.objects_path or archivo_existente(OBJECTS_FILE_CANDIDATOS, obligatorio=False)
    trucks_path = net.trucks_path or archivo_existente(TRUCKS_FILE_CANDIDATOS, obligatorio=False)

    if not objects_path or not os.path.exists(objects_path):
        errores.append("No se encontro objects.xml para GET_FLEET")
    else:
        root = ET.parse(objects_path).getroot()
        for obj in root.findall("object"):
            obj_type = normalizar_id(obj.get("type", ""))
            obj_id = normalizar_id(obj.get("id", ""))
            if obj_type.lower() != "shovel":
                continue
            if not obj_id:
                errores.append("Shovel sin id en objects.xml")
                continue

            loc = obj.find("location")
            location_edge = normalizar_id(loc.get("edge", "")) if loc is not None else ""
            location_lane = normalizar_id(loc.get("lane", "")) if loc is not None else ""

            shovels.append({
                "id": obj_id,
                "ubicacionInicial": obj_id,
                "capacidad": _float_texto(obj, "capacity", 0.0),
                "duracionPalada": _long_texto(obj, "loadTime", 0),
                "destinoMateriales": normalizar_id(_texto_hijo(obj, "destination", "")),
                "edge": location_edge,
                "lane": location_lane,
            })

    if not trucks_path or not os.path.exists(trucks_path):
        errores.append("No se encontro trucks.xml para GET_FLEET")
    else:
        root = ET.parse(trucks_path).getroot()
        for tr in root.findall("truck"):
            jade_id = normalizar_id(tr.get("jadeId", ""))
            sumo_id = normalizar_id(tr.get("sumoId", ""))
            if not jade_id:
                errores.append("Truck sin jadeId en trucks.xml")
                continue

            loc = tr.find("location")
            location_edge = normalizar_id(loc.get("edge", "")) if loc is not None else ""
            location_lane = normalizar_id(loc.get("lane", "")) if loc is not None else ""

            # Para JADE, ubicacionInicial debe ser el id logico del truck.
            # Python/routing_service lo resolvera a edge/lane desde trucks.xml.
            if USAR_VELOCIDAD_CAMIONES_CONTROLADA:
                vel_vacio, vel_cargado = velocidades_camion(jade_id)
            else:
                vel_vacio = _float_texto(tr, "emptySpeed", VELOCIDAD_DEF_VACIO_KMH)
                vel_cargado = _float_texto(tr, "loadedSpeed", VELOCIDAD_DEF_CARGADO_KMH)

            trucks.append({
                "id": jade_id,
                "sumoId": sumo_id,
                "ubicacionInicial": jade_id,
                "velocidadVacio": vel_vacio,
                "velocidadCargado": vel_cargado,
                "capacidad": _float_texto(tr, "capacity", 0.0),
                "duracionDescarga": _long_texto(tr, "dischargeTime", 0),
                "spottingTime": _long_texto(tr, "spottingTime", 0),
                "operationTime": _long_texto(tr, "operationTime", 0),
                "edge": location_edge,
                "lane": location_lane,
                "ubicacionValida": bool(location_edge and location_lane),
            })

            if not location_edge or not location_lane:
                errores.append(
                    f"{jade_id} no tiene edge/lane en trucks.xml. "
                    "Debe usarse trucks.xml enriquecido; el junction no se usa para rutas."
                )

    ok = bool(shovels and trucks and not errores)
    return {
        "ok": ok,
        "error": "" if ok else "ERROR_RECEPCION_FLOTA",
        "errores": errores,
        "shovels": shovels,
        "trucks": trucks,
        "numShovels": len(shovels),
        "numTrucks": len(trucks),
        "diagnosticoRouting": net.diagnostico() if hasattr(net, "diagnostico") else {},
    }


def velocidad_operacion_ms(net: NetSumo, camion: str, operacion: str, velocidad_msg: Any = "") -> float:
    op = normalizar_id(operacion).upper()

    # Mantiene coherencia entre GET_FLEET y ROUTE: JADE planifica siempre con
    # las mismas velocidades nominales entregadas por server.py.
    if USAR_VELOCIDAD_CAMIONES_CONTROLADA:
        vel_vacio, vel_cargado = velocidades_camion(camion)
        if op == "VIAJE_CARGADO":
            return parsear_velocidad_ms(vel_cargado, VELOCIDAD_DEF_CARGADO_KMH)
        return parsear_velocidad_ms(vel_vacio, VELOCIDAD_DEF_VACIO_KMH)

    if velocidad_msg not in (None, ""):
        default = VELOCIDAD_DEF_CARGADO_KMH if op == "VIAJE_CARGADO" else VELOCIDAD_DEF_VACIO_KMH
        return parsear_velocidad_ms(velocidad_msg, default)

    loc = net.resolver_ubicacion(camion, como_destino=False)
    if op == "VIAJE_CARGADO":
        return parsear_velocidad_ms(loc.get("loadedSpeed", ""), VELOCIDAD_DEF_CARGADO_KMH)
    return parsear_velocidad_ms(loc.get("emptySpeed", ""), VELOCIDAD_DEF_VACIO_KMH)


def calcular_ruta_response(net: NetSumo, msg: Dict[str, Any]) -> Dict[str, Any]:
    camion = normalizar_id(tomar(msg, ["camion", "truck", "vehiculo", "vehicle"], ""))
    origen = normalizar_id(tomar(msg, ["origen", "from", "inicio"], ""))
    destino = normalizar_id(tomar(msg, ["destino", "to", "fin"], ""))
    operacion = normalizar_id(tomar(msg, ["operacion", "operation", "estado"], "VIAJE_VACIO")).upper()

    if not origen:
        # Para el primer viaje vacío, si no viene origen, se usa la ubicación inicial del camión.
        origen = camion

    if not camion:
        return {"ok": False, "error": "ROUTE_SIN_CAMION"}
    if not origen or not destino:
        return {"ok": False, "error": "ROUTE_SIN_ORIGEN_DESTINO", "origen": origen, "destino": destino}

    edges, estado = net.shortest_edges(origen, destino, excluded_edges=EDGES_BLOQUEADOS_ACTUALES)
    if not edges:
        return {
            "ok": False,
            "error": estado,
            "camion": camion,
            "origen": origen,
            "destino": destino,
            "operacion": operacion,
            "edges": [],
            "cantidadEdges": 0,
            "distanciaM": 0.0,
            "tiempoMs": -1,
            "origenInfo": net.diagnostico_ubicacion(origen) if hasattr(net, "diagnostico_ubicacion") else {},
            "destinoInfo": net.diagnostico_ubicacion(destino) if hasattr(net, "diagnostico_ubicacion") else {},
        }

    # IMPORTANTE: JADE debe planificar con la distancia de los mismos routeEdges
    # que posteriormente ejecutara SUMO. No se usa distancia_parcial()
    # para el tiempo de planificacion, porque reducir el primer/ultimo edge puede
    # subestimar fuertemente la duracion respecto de la ruta fisica aplicada.
    distancia_m = net.distancia_edges(edges)
    motivo_distancia = "DISTANCIA_RUTA_SUMO_COMPLETA"

    if distancia_m <= 0:
        return {
            "ok": False,
            "error": "RUTA_SIN_DISTANCIA_VALIDA",
            "estadoRuta": estado,
            "motivoDistancia": motivo_distancia,
            "camion": camion,
            "origen": origen,
            "destino": destino,
            "operacion": operacion,
            "edges": edges,
            "routeEdges": " ".join(edges),
            "cantidadEdges": len(edges),
            "distanciaM": distancia_m,
            "tiempoMs": -1,
        }

    velocidad_ms = velocidad_operacion_ms(net, camion, operacion, tomar(msg, ["velocidad", "speed"], ""))

    # Tiempo ideal enviado a JADE: distancia completa de la ruta SUMO / velocidad nominal.
    tiempo_ideal_ms = int(round((distancia_m / max(0.1, velocidad_ms)) * 1000.0))
    tiempo_ms = tiempo_ideal_ms

    print(
        f"[ROUTE_TIME] {camion} {operacion} {origen}->{destino} "
        f"edges={len(edges)} dist={distancia_m:.2f}m "
        f"vel={velocidad_ms:.3f}m/s tiempo={tiempo_ms / 1000.0:.3f}s "
        f"motivo={motivo_distancia}"
    )

    return {
        "ok": True,
        "error": "",
        "estadoRuta": estado,
        "motivoDistancia": motivo_distancia,
        "camion": camion,
        "origen": origen,
        "destino": destino,
        "operacion": operacion,
        "edges": edges,
        "routeEdges": " ".join(edges),
        "cantidadEdges": len(edges),
        "distanciaM": distancia_m,
        "tiempoMs": tiempo_ms,
        "tiempoIdealMs": tiempo_ideal_ms,
        "factorFisicoTiempo": 1.0,
        "penalizacionEdgesMs": 0,
        "velocidadMs": velocidad_ms,
        "origenInfo": net.diagnostico_ubicacion(origen) if hasattr(net, "diagnostico_ubicacion") else {},
        "destinoInfo": net.diagnostico_ubicacion(destino) if hasattr(net, "diagnostico_ubicacion") else {},
    }


# ============================================================
# Memoria del cronograma
# ============================================================

class MemoriaCronograma:
    def __init__(self, net: NetSumo):
        self.net = net
        self.eventos: List[Dict[str, Any]] = []
        self.por_camion: Dict[str, List[Dict[str, Any]]] = defaultdict(list)

    def reset(self) -> None:
        self.eventos.clear()
        self.por_camion.clear()

    def agregar_evento(self, evento_jade: Dict[str, Any], fila: int = 0) -> Dict[str, Any]:
        maquina = normalizar_id(tomar(evento_jade, ["maquina", "machine", "camion", "truck", "vehiculo", "vehicle"]))
        propietaria = normalizar_id(tomar(evento_jade, ["maquinaPropietariaSchedule", "maquina_propietaria", "propietaria", "owner", "truck"]))
        if not maquina and propietaria:
            maquina = propietaria
        if not propietaria:
            propietaria = maquina

        operacion = normalizar_id(tomar(evento_jade, ["operacion", "operation", "actividad", "tipoOperacion"])).upper()
        ubicacion = normalizar_id(tomar(evento_jade, ["ubicacion", "location", "destino", "ubicacionLogica"], ""))
        hora_inicio = convertir_int(tomar(evento_jade, ["horaInicio", "inicio", "start", "startTime", "hora_inicio"], 0))
        hora_fin = convertir_int(tomar(evento_jade, ["horaFin", "fin", "end", "endTime", "hora_fin"], 0))
        ubicacion_inicial = normalizar_id(tomar(evento_jade, ["ubicacionInicialCamion", "ubicacionInicial", "positionedAt", "origenInicial", "initialLocation"], ""))

        if not maquina:
            raise ValueError(f"Evento sin maquina/camion: {evento_jade}")
        if not operacion:
            raise ValueError(f"Evento sin operacion: {evento_jade}")
        if not ubicacion:
            raise ValueError(f"Evento sin ubicacion: {evento_jade}")

        evento = {
            "fila_original": fila,
            "maquina": maquina,
            "maquinaPropietariaSchedule": propietaria,
            "operacion": operacion,
            "horaInicio": hora_inicio,
            "horaFin": hora_fin,
            "duracion": max(0, hora_fin - hora_inicio),
            "ubicacion": ubicacion,
            "ubicacionInicialCamion": ubicacion_inicial,
            "cantidadMaterial": tomar(evento_jade, ["cantidadMaterial", "cantidad", "material"], ""),
            "velocidadVacio": tomar(evento_jade, ["velocidadVacio", "maximumPossibleVelocity", "velocidad_vacio", "velocidadMaxima", "vacio"], ""),
            "velocidadCargado": tomar(evento_jade, ["velocidadCargado", "loadedSpeed", "velocidad_cargado", "cargado"], ""),
            "routeEdges": tomar(evento_jade, ["routeEdges", "route_edges", "route_edges_str"], ""),
            "origenRuta": normalizar_id(tomar(evento_jade, ["origenRuta", "origenRutaJade", "origen"], "")),
            "destinoRuta": normalizar_id(tomar(evento_jade, ["destinoRuta", "destinoRutaJade", "destino"], "")),
        }

        self.eventos.append(evento)
        self.ordenar()
        return evento

    def agregar_eventos(self, eventos: List[Dict[str, Any]], limpiar: bool = False) -> Dict[str, Any]:
        if limpiar:
            self.reset()
        agregados = 0
        errores = []
        for i, ev in enumerate(eventos, start=1):
            try:
                self.agregar_evento(ev, fila=i)
                agregados += 1
            except Exception as e:
                errores.append({"indice": i, "error": str(e), "evento": ev})
        self.ordenar()
        return {"agregados": agregados, "errores": errores, "eventos_en_memoria": len(self.eventos)}

    def ordenar(self) -> None:
        self.eventos.sort(key=lambda e: (
            e.get("maquinaPropietariaSchedule", e.get("maquina", "")),
            convertir_int(e.get("horaInicio", 0)),
            convertir_int(e.get("horaFin", 0)),
            e.get("operacion", "")
        ))
        self.por_camion.clear()
        for ev in self.eventos:
            camion = ev.get("maquina")
            self.por_camion[camion].append(ev)
        for camion in list(self.por_camion.keys()):
            self.por_camion[camion].sort(key=lambda e: (
                convertir_int(e.get("horaInicio", 0)),
                convertir_int(e.get("horaFin", 0)),
                e.get("operacion", "")
            ))

    def validar(self) -> Dict[str, Any]:
        solapados = []
        tiempos_invalidos = []
        for camion, lista in self.por_camion.items():
            anterior_fin = None
            for ev in lista:
                inicio = convertir_int(ev.get("horaInicio", 0))
                fin = convertir_int(ev.get("horaFin", 0))
                if fin < inicio:
                    tiempos_invalidos.append({
                        "maquina": camion,
                        "operacion": ev.get("operacion", ""),
                        "horaInicio": inicio,
                        "horaFin": fin,
                    })
                if anterior_fin is not None and inicio < anterior_fin:
                    solapados.append({
                        "maquina": camion,
                        "operacion": ev.get("operacion", ""),
                        "horaInicio": inicio,
                        "horaFinAnterior": anterior_fin,
                    })
                anterior_fin = max(anterior_fin or 0, fin)
        return {
            "ok": not tiempos_invalidos,
            "eventos": len(self.eventos),
            "camiones_o_propietarios": len(self.por_camion),
            "tiempos_invalidos": tiempos_invalidos,
            "advertencias_solapados": solapados,
        }

    def exportar_csv(self, path: str) -> None:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        columnas = [
            "fila_original", "maquina", "maquinaPropietariaSchedule", "operacion",
            "horaInicio", "horaFin", "duracion", "ubicacion", "ubicacionInicialCamion",
            "cantidadMaterial", "velocidadVacio", "velocidadCargado",
            "origenRuta", "destinoRuta", "routeEdges",
        ]
        with open(path, "w", newline="", encoding="utf-8-sig") as f:
            writer = csv.DictWriter(f, fieldnames=columnas)
            writer.writeheader()
            for ev in self.eventos:
                writer.writerow({c: ev.get(c, "") for c in columnas})

    def exportar_json(self, path: str) -> None:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"eventos": self.eventos, "validacion": self.validar()}, f, ensure_ascii=False, indent=2)



# ============================================================
# Memoria de solicitudes de rescheduling dinámico
# ============================================================

ESCENARIO_ACTUAL = "small"
ESCENARIO_OUTPUT_DIR = os.path.dirname(CRONOGRAMA_CSV)
RESCHEDULING_CSV = os.path.join(ESCENARIO_OUTPUT_DIR, "rescheduling.csv")
SNAPSHOT_FLOTA_JSON = os.path.join(ESCENARIO_OUTPUT_DIR, "snapshot_flota_rescheduling.json")
SNAPSHOT_FLOTA_CSV = os.path.join(ESCENARIO_OUTPUT_DIR, "snapshot_flota_rescheduling.csv")
RESCHEDULE_TRACE_DEFAULT = os.path.join(ESCENARIO_OUTPUT_DIR, "sumoFleetSinRLreschedules.csv")
TRACE_TRUCK_FREE_CSV = os.path.join(ESCENARIO_OUTPUT_DIR, "trace_truck_free.csv")
HORIZONTE_TURNO_MS = 6 * 60 * 60 * 1000


def configurar_rutas(config_salida: Dict[str, str]) -> None:
    """Sincroniza cronogramas y reportes internos con el escenario activo."""
    global ESCENARIO_ACTUAL, ESCENARIO_OUTPUT_DIR
    global CRONOGRAMA_CSV, CRONOGRAMA_JSON
    global RESCHEDULING_CSV, SNAPSHOT_FLOTA_JSON, SNAPSHOT_FLOTA_CSV
    global RESCHEDULE_TRACE_DEFAULT, TRACE_TRUCK_FREE_CSV

    ESCENARIO_ACTUAL = config_salida["escenario"]
    ESCENARIO_OUTPUT_DIR = config_salida["output_dir"]
    CRONOGRAMA_CSV = config_salida["cronograma_csv"]
    CRONOGRAMA_JSON = config_salida["cronograma_json"]
    RESCHEDULING_CSV = os.path.join(ESCENARIO_OUTPUT_DIR, "rescheduling.csv")
    SNAPSHOT_FLOTA_JSON = os.path.join(ESCENARIO_OUTPUT_DIR, "snapshot_flota_rescheduling.json")
    SNAPSHOT_FLOTA_CSV = os.path.join(ESCENARIO_OUTPUT_DIR, "snapshot_flota_rescheduling.csv")
    RESCHEDULE_TRACE_DEFAULT = os.path.join(ESCENARIO_OUTPUT_DIR, "sumoFleetSinRLreschedules.csv")
    TRACE_TRUCK_FREE_CSV = os.path.join(ESCENARIO_OUTPUT_DIR, "trace_truck_free.csv")
    os.makedirs(ESCENARIO_OUTPUT_DIR, exist_ok=True)


TRACE_TRUCK_FREE_COLUMNS = [
    "freeRequestId", "truck", "stage", "sim_time_ms", "wall_time",
    "server_wall_time", "jade_wall_time", "cycle_id", "secuencia", "detalle",
]

def _wall_now() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")

def _trace_truck_free(row: Dict[str, Any]) -> None:
    os.makedirs(os.path.dirname(TRACE_TRUCK_FREE_CSV) or ".", exist_ok=True)
    existe = os.path.exists(TRACE_TRUCK_FREE_CSV) and os.path.getsize(TRACE_TRUCK_FREE_CSV) > 0
    with open(TRACE_TRUCK_FREE_CSV, "a", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=TRACE_TRUCK_FREE_COLUMNS)
        if not existe:
            w.writeheader()
        w.writerow({c: row.get(c, "") for c in TRACE_TRUCK_FREE_COLUMNS})

RESCHEDULE_TRACE_COLUMNS = [
    "requestId", "camion", "secuenciaCiclo", "cycleId", "operacion",
    "horaInicio", "horaFin", "origenRuta", "destinoRuta", "routeEdges",
    "cantidadEdges", "distanciaM", "estado", "tiempoSumo", "detalle",
]


def _to_float(valor: Any, default: float = 0.0) -> float:
    try:
        return float(str(valor).replace(",", "."))
    except Exception:
        return default


class ReschedulingManager:
    """
    Guarda UNA solicitud de rescheduling global generada durante la simulacion.

    Regla principal:
    - No se compara un camion individual contra el 10%.
    - El runner SUMO suma el retraso actual de todos los camiones.
    - Si el retraso total de la flota supera el 10% del cronograma JADE,
      se crea una solicitud GLOBAL para replanificar todos los camiones.
    """

    columnas = [
        "request_id",
        "estado",
        "accion",
        "alcance",
        "motivo",
        "camion",
        "camion_disparador",
        "tiempo_primera_deteccion_s",
        "tiempo_ultima_deteccion_s",
        "primer_segmento",
        "ultimo_segmento",
        "operacion_primera",
        "operacion_ultima",
        "ruta_primera",
        "ruta_ultima",
        "retraso_inicial_s",
        "retraso_max_s",
        "retraso_total_flota_s",
        "desvio_max",
        "total_jade_s",
        "umbral_s",
        "cantidad_eventos",
        "camiones_con_retraso",
        "edge_actual",
        "lane_actual",
        "pos_actual",
        "velocidad_actual_s",
        "aplicar_desde",
        "ack_tiempo_s",
        "ack_origen",
        "observacion",
    ]

    def __init__(self, net: NetSumo, output_csv: str = ""):
        self.net = net
        self.output_csv = output_csv or RESCHEDULING_CSV
        self.snapshot_json = SNAPSHOT_FLOTA_JSON
        self.snapshot_csv = SNAPSHOT_FLOTA_CSV
        self.requests: Dict[str, Dict[str, Any]] = {}
        self.snapshot_flota: List[Dict[str, Any]] = []
        self.snapshot_tiempo_s: float = 0.0
        # Estado disponible calculado por SUMO para el rescheduling.
        # clave: TruckX -> fila snapshot con ubicacion_disponible, edge_disponible, tiempo_disponible_estimado_s.
        self.disponibilidad_por_camion: Dict[str, Dict[str, Any]] = {}
        # clave: PAXX -> estado de disponibilidad calculado por SUMO.
        # Esto evita que todas las palas reinicien desde la misma base global.
        self.disponibilidad_por_pala: Dict[str, Dict[str, Any]] = {}

        # RESCHEDULING incremental. No interviene en el scheduling inicial.
        self.free: Dict[str, Dict[str, Any]] = {}
        self.cycles: List[Dict[str, Any]] = []
        self.last_id: int = 0
        self.trucks_inicializados: set[str] = set()
        self.trucks_plan_completo: set[str] = set()
        # True cuando JADE envía FINALIZE_RESCHEDULE. Desde ese instante
        # no se aceptan nuevos ciclos y SUMO solo debe vaciar lo ya recibido.
        self.rescheduling_finalizado: bool = False

        # CSV de trazabilidad del rescheduling recibido y ejecutado por SUMO.
        # Usa el mismo nombre enviado por JADE y se guarda en el escenario activo.
        self.trace_csv_default = RESCHEDULE_TRACE_DEFAULT
        self.trace_csv_inicializados: set[str] = set()

    def reset(self) -> None:
        self.requests.clear()
        self.snapshot_flota.clear()
        self.snapshot_tiempo_s = 0.0
        self.disponibilidad_por_camion.clear()
        self.disponibilidad_por_pala.clear()
        self.free.clear()
        self.cycles.clear()
        self.last_id = 0
        self.trucks_inicializados.clear()
        self.trucks_plan_completo.clear()
        self.rescheduling_finalizado = False
        for trace_path in list(self.trace_csv_inicializados):
            try:
                if os.path.exists(trace_path):
                    os.remove(trace_path)
            except Exception as e:
                print(f"[rescheduling] No se pudo limpiar CSV de trazabilidad {trace_path}: {e}")
        self.trace_csv_inicializados.clear()
        try:
            if os.path.exists(TRACE_TRUCK_FREE_CSV):
                os.remove(TRACE_TRUCK_FREE_CSV)
        except Exception:
            pass
        self.exportar_csv()
        self.exportar_snapshot()

    def registrar(self, msg: Dict[str, Any]) -> Dict[str, Any]:
        # En esta version la solicitud es global. El camion solo queda como referencia
        # del evento que gatillo la evaluacion, no como alcance del rescheduling.
        camion_disparador = normalizar_id(tomar(
            msg,
            ["camion_disparador", "camion", "truck", "vehiculo", "vehicle"],
            ""
        ))

        tiempo_sumo_s = _to_float(tomar(msg, ["tiempo_sumo_s", "time", "time_s", "simTime"], 0.0), 0.0)
        retraso_total_flota_s = _to_float(tomar(
            msg,
            ["retraso_total_flota_s", "retraso_flota_s", "total_delay_s", "fleet_delay_s", "retraso_s"],
            0.0
        ), 0.0)
        desvio = _to_float(tomar(msg, ["desvio", "delay_percent", "porcentaje"], 0.0), 0.0)
        total_jade_s = _to_float(tomar(msg, ["total_jade_s", "total_jade"], 0.0), 0.0)
        umbral_s = _to_float(tomar(msg, ["umbral_s", "threshold_s"], 0.0), 0.0)
        segmento = convertir_int(tomar(msg, ["segmento", "seg_num", "segment", "segmento_actual"], 0), 0)
        operacion = normalizar_id(tomar(msg, ["operacion_actual", "operacion", "operation"], ""))
        origen = normalizar_id(tomar(msg, ["origen", "origenRuta", "from"], ""))
        destino = normalizar_id(tomar(msg, ["destino", "destinoRuta", "to"], ""))
        ruta = normalizar_id(tomar(msg, ["ruta", "route"], ""))
        if not ruta and (origen or destino):
            ruta = f"{origen}->{destino}"

        edge_actual = normalizar_id(tomar(msg, ["edge_actual", "edge", "current_edge"], ""))
        lane_actual = normalizar_id(tomar(msg, ["lane_actual", "lane", "current_lane"], ""))
        pos_actual = _to_float(tomar(msg, ["pos_actual", "pos", "position"], 0.0), 0.0)
        velocidad_actual_s = _to_float(tomar(msg, ["velocidad_actual_s", "speed", "speed_s"], 0.0), 0.0)
        camiones_con_retraso = normalizar_id(tomar(msg, ["camiones_con_retraso", "trucks_delay", "camiones_afectados"], ""))
        motivo = normalizar_id(tomar(msg, ["motivo", "evento", "reason"], "RETRASO_TOTAL_FLOTA_SUPERA_10")) or "RETRASO_TOTAL_FLOTA_SUPERA_10"

        snapshot = tomar(msg, ["snapshot_flota", "snapshot", "fotografia_flota"], [])
        disponibilidad_palas = tomar(msg, ["disponibilidad_por_pala", "disponibilidad_palas", "shovel_availability"], {})

        key = "GLOBAL"
        existente = self.requests.get(key)

        # El snapshot oficial del rescheduling se congela con la primera solicitud.
        # Si llega otra actualización posterior, se actualiza solo el resumen de la solicitud,
        # pero NO se reemplaza la fotografía usada por JADE.
        if not existente:
            if isinstance(snapshot, list):
                self.snapshot_flota = snapshot
                self.snapshot_tiempo_s = _to_float(tomar(msg, ["snapshot_tiempo_s", "tiempo_sumo_s", "time"], tiempo_sumo_s), tiempo_sumo_s)
                self.actualizar_camiones()

            self.actualizar_palas(disponibilidad_palas, tiempo_sumo_s)
            self.exportar_snapshot()
        if existente:
            existente["tiempo_ultima_deteccion_s"] = f"{tiempo_sumo_s:.2f}"
            existente["ultimo_segmento"] = segmento
            existente["operacion_ultima"] = operacion
            existente["ruta_ultima"] = ruta
            existente["cantidad_eventos"] = convertir_int(existente.get("cantidad_eventos", 0), 0) + 1
            existente["camion_disparador"] = camion_disparador
            existente["camiones_con_retraso"] = camiones_con_retraso
            existente["edge_actual"] = edge_actual
            existente["lane_actual"] = lane_actual
            existente["pos_actual"] = f"{pos_actual:.2f}"
            existente["velocidad_actual_s"] = f"{velocidad_actual_s:.2f}"
            existente["total_jade_s"] = f"{total_jade_s:.2f}"
            existente["umbral_s"] = f"{umbral_s:.2f}"
            existente["retraso_total_flota_s"] = f"{retraso_total_flota_s:.2f}"
            if retraso_total_flota_s > _to_float(existente.get("retraso_max_s", 0.0), 0.0):
                existente["retraso_max_s"] = f"{retraso_total_flota_s:.2f}"
                existente["desvio_max"] = f"{desvio:.2f}"
                existente["observacion"] = "Solicitud global actualizada con nuevo retraso total maximo de flota"
            else:
                existente["observacion"] = "Solicitud global ya existente; se actualiza conteo/posicion"
            self.exportar_csv()
            return {
                "ok": True,
                "accion": "UPDATED_GLOBAL",
                "msg": "Solicitud global de rescheduling actualizada; no se duplico",
                "request": existente,
            }

        request_id = f"RS_GLOBAL_{int(tiempo_sumo_s * 10):010d}"
        solicitud = {
            "request_id": request_id,
            "estado": "PENDIENTE",
            "accion": "RESCHEDULE_GLOBAL",
            "alcance": "GLOBAL_FLOTA",
            "motivo": motivo,
            "camion": key,
            "camion_disparador": camion_disparador,
            "tiempo_primera_deteccion_s": f"{tiempo_sumo_s:.2f}",
            "tiempo_ultima_deteccion_s": f"{tiempo_sumo_s:.2f}",
            "primer_segmento": segmento,
            "ultimo_segmento": segmento,
            "operacion_primera": operacion,
            "operacion_ultima": operacion,
            "ruta_primera": ruta,
            "ruta_ultima": ruta,
            "retraso_inicial_s": f"{retraso_total_flota_s:.2f}",
            "retraso_max_s": f"{retraso_total_flota_s:.2f}",
            "retraso_total_flota_s": f"{retraso_total_flota_s:.2f}",
            "desvio_max": f"{desvio:.2f}",
            "total_jade_s": f"{total_jade_s:.2f}",
            "umbral_s": f"{umbral_s:.2f}",
            "cantidad_eventos": 1,
            "camiones_con_retraso": camiones_con_retraso,
            "edge_actual": edge_actual,
            "lane_actual": lane_actual,
            "pos_actual": f"{pos_actual:.2f}",
            "velocidad_actual_s": f"{velocidad_actual_s:.2f}",
            "aplicar_desde": "REPLANIFICACION_GLOBAL_DESDE_TIEMPO_ACTUAL",
            "ack_tiempo_s": "",
            "ack_origen": "",
            "observacion": "Solicitud global creada porque el retraso total de la flota supero el 10%",
        }
        self.requests[key] = solicitud
        self.exportar_csv()
        print(
            f"[rescheduling] REQUEST GLOBAL: retraso_total_flota={retraso_total_flota_s:.2f}s "
            f"umbral={umbral_s:.2f}s disparador={camion_disparador} estado=PENDIENTE"
        )
        return {"ok": True, "accion": "CREATED_GLOBAL", "msg": "Solicitud global de rescheduling creada", "request": solicitud}

    def get_requests(self, estado: str = "") -> List[Dict[str, Any]]:
        estado = normalizar_id(estado).upper()
        datos = list(self.requests.values())
        if estado:
            datos = [r for r in datos if normalizar_id(r.get("estado", "")).upper() == estado]
        return sorted(datos, key=lambda r: (r.get("estado", ""), r.get("camion", "")))

    def ack(self, msg: Dict[str, Any]) -> Dict[str, Any]:
        camion = normalizar_id(tomar(msg, ["camion", "truck", "vehiculo", "vehicle"], "")) or "GLOBAL"
        if camion != "GLOBAL" and camion not in self.requests:
            camion = "GLOBAL"
        solicitud = self.requests.get(camion)
        if not solicitud:
            return {"ok": False, "error": f"NO_EXISTE_SOLICITUD:{camion}"}

        estado = normalizar_id(tomar(msg, ["estado", "status"], "ACEPTADA")).upper() or "ACEPTADA"
        solicitud["estado"] = estado
        solicitud["ack_tiempo_s"] = str(tomar(msg, ["tiempo_sumo_s", "ack_tiempo_s", "time"], ""))
        solicitud["ack_origen"] = normalizar_id(tomar(msg, ["origen", "source", "agente"], "JADE")) or "JADE"
        solicitud["observacion"] = normalizar_id(tomar(msg, ["observacion", "msg"], f"Solicitud global {estado}"))
        self.exportar_csv()
        print(f"[rescheduling] ACK GLOBAL: estado={estado}")
        return {"ok": True, "msg": "ACK global aplicado", "request": solicitud}

    def completar(self, msg: Dict[str, Any]) -> Dict[str, Any]:
        camion = normalizar_id(tomar(msg, ["camion", "truck", "vehiculo", "vehicle"], "")) or "GLOBAL"
        if camion != "GLOBAL" and camion not in self.requests:
            camion = "GLOBAL"
        solicitud = self.requests.get(camion)
        if not solicitud:
            return {"ok": False, "error": f"NO_EXISTE_SOLICITUD:{camion}"}
        solicitud["estado"] = "COMPLETADA"
        solicitud["observacion"] = normalizar_id(tomar(msg, ["observacion", "msg"], "Rescheduling global completado"))
        self.exportar_csv()
        return {"ok": True, "msg": "Solicitud global completada", "request": solicitud}


    def add_free(self, msg: Dict[str, Any]) -> Dict[str, Any]:
        """Registra la posición real desde la que un camión puede renegociar."""
        truck = normalizar_id(tomar(msg, ["truck", "camion"], ""))
        if not truck:
            return {"ok": False, "error": "SIN_TRUCK"}

        time_ms = convertir_int(tomar(msg, ["time_ms", "tiempo_disponible_ms"], 0), 0)
        if time_ms <= 0:
            time_s = _to_float(tomar(msg, ["time", "tiempo_sumo_s"], 0.0), 0.0)
            time_ms = int(round(time_s * 1000.0))

        place = normalizar_id(tomar(msg, ["place", "ubicacion", "ubicacion_disponible"], ""))
        edge = normalizar_id(tomar(msg, ["edge", "edge_actual", "edge_disponible"], ""))
        lane = normalizar_id(tomar(msg, ["lane", "lane_actual"], ""))
        pos = _to_float(tomar(msg, ["pos", "posicion", "pos_actual"], 0.0), 0.0)
        estado_disponibilidad = normalizar_id(tomar(msg, ["estado_disponibilidad", "estadoDisponible", "estado"], "DISPONIBLE")) or "DISPONIBLE"
        parking_area = normalizar_id(tomar(msg, ["parkingArea", "parking_area"], ""))

        # Respaldo: el snapshot oficial ya contiene el edge final de descarga.
        estado_snapshot = self.disponibilidad_por_camion.get(truck, {})
        if not place:
            place = normalizar_id(tomar(
                estado_snapshot,
                ["ubicacion_disponible", "ubicacion_referencia"],
                "",
            ))
        if not edge:
            edge = normalizar_id(tomar(
                estado_snapshot,
                ["edge_disponible", "edge_fin_segmento", "edge_actual"],
                "",
            ))

        # Último respaldo: resolver la ubicación lógica mediante routing_service.
        if not edge and place:
            loc = self.net.resolver_ubicacion(place, como_destino=False)
            if loc.get("valid"):
                edge = normalizar_id(loc.get("edge", ""))
                if not lane:
                    lane = normalizar_id(loc.get("lane", ""))

        if not edge:
            return {
                "ok": False,
                "error": "TRUCK_FREE_SIN_EDGE_REAL",
                "truck": truck,
                "msg": msg,
                "snapshot": estado_snapshot,
            }

        if not lane:
            lane = normalizar_id(self.net.edge_to_lane.get(edge, ""))

        free_request_id = normalizar_id(tomar(msg, ["freeRequestId", "free_request_id"], ""))
        if not free_request_id:
            free_request_id = f"FREE_{truck}_{time_ms}"
        server_wall = _wall_now()

        row = {
            "freeRequestId": free_request_id,
            "truck": truck,

            # Contrato TRUCK_FREE real: estos son los datos que manda SUMO
            # cuando el camión terminó de verdad su compromiso actual.
            "time_ms": time_ms,
            "place": place,
            "edge": edge,
            "lane": lane,
            "pos": pos,

            # Alias normalizados para JADE. Mantener ambos formatos evita que
            # TruckAgent vuelva al snapshot estimado cuando ya existe un estado real.
            "tiempo_disponible_ms": time_ms,
            "tiempo_disponible_s": time_ms / 1000.0,
            "ubicacion_disponible": place,
            "edge_disponible": edge,
            "aplicar_desde": "TRUCK_FREE_REAL",
            "estado_disponibilidad": estado_disponibilidad,
            "parkingArea": parking_area,

            "sent": False,
            "serverReceivedWallTime": server_wall,
            "sumoSentWallTime": tomar(msg, ["sumoSentWallTime", "sumo_sent_wall_time"], ""),
        }
        self.free[truck] = row
        _trace_truck_free({
            "freeRequestId": free_request_id, "truck": truck, "stage": "SERVER_RECEIVE",
            "sim_time_ms": time_ms, "wall_time": server_wall,
            "server_wall_time": server_wall, "detalle": f"edge={edge};place={place}",
        })
        print(
            f"[rescheduling] TRUCK_FREE registrado: {truck} "
            f"estado={estado_disponibilidad} edge={edge} place={place}"
        )
        return {"ok": True, "truck": row}

    def get_free(self) -> Dict[str, Any]:
        rows = []
        for truck in sorted(self.free):
            row = self.free[truck]
            if row.get("sent"):
                continue
            rows.append(dict(row))
            row["sent"] = True
            _trace_truck_free({
                "freeRequestId": row.get("freeRequestId", ""), "truck": truck,
                "stage": "SERVER_DELIVER_GET_FREE", "sim_time_ms": row.get("time_ms", ""),
                "wall_time": _wall_now(), "server_wall_time": row.get("serverReceivedWallTime", ""),
            })
        return {"ok": True, "trucks": rows}

    def _resolver_trace_csv(self, nombre_archivo: Any = "") -> str:
        nombre = os.path.basename(normalizar_id(nombre_archivo))
        if not nombre.lower().endswith(".csv"):
            nombre = os.path.basename(self.trace_csv_default)
        return os.path.join(ESCENARIO_OUTPUT_DIR, nombre)

    def _asegurar_trace_csv(self, path: str) -> None:
        if path in self.trace_csv_inicializados:
            return
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        # La primera vez en esta ejecución se crea desde cero para no mezclar pruebas.
        with open(path, "w", newline="", encoding="utf-8-sig") as f:
            writer = csv.DictWriter(f, fieldnames=RESCHEDULE_TRACE_COLUMNS)
            writer.writeheader()
        self.trace_csv_inicializados.add(path)
        print(f"[rescheduling] CSV de trazabilidad activo: {path}")

    def registrar_ciclo(
        self,
        cycle: Dict[str, Any],
        estado: str,
        tiempo_sumo: Any = "",
        detalle: Any = "",
    ) -> None:
        path = normalizar_id(cycle.get("trace_csv", "")) or self.trace_csv_default
        self._asegurar_trace_csv(path)
        events = cycle.get("events", [])
        with open(path, "a", newline="", encoding="utf-8-sig") as f:
            writer = csv.DictWriter(f, fieldnames=RESCHEDULE_TRACE_COLUMNS)
            for ev in events:
                writer.writerow({
                    "requestId": cycle.get("requestId", ""),
                    "camion": cycle.get("truck", ""),
                    "secuenciaCiclo": cycle.get("secuencia", 0),
                    "cycleId": cycle.get("id", 0),
                    "operacion": ev.get("operacion", ""),
                    "horaInicio": ev.get("horaInicio", ""),
                    "horaFin": ev.get("horaFin", ""),
                    "origenRuta": ev.get("origenRuta", ""),
                    "destinoRuta": ev.get("destinoRuta", ""),
                    "routeEdges": ev.get("routeEdges", ""),
                    "cantidadEdges": ev.get("cantidadEdges", ""),
                    "distanciaM": ev.get("distanciaM", ""),
                    "estado": estado,
                    "tiempoSumo": tiempo_sumo,
                    "detalle": detalle,
                })

    def add_cycle(self, msg: Dict[str, Any]) -> Dict[str, Any]:
        """
        Recibe cada ciclo apenas JADE lo confirma. El primer ciclo usa el edge
        real de TRUCK_FREE; los siguientes usan el ultimo edge fisico de la ruta
        del ciclo anterior. Se permiten varios ciclos futuros del mismo camion
        siempre que sus intervalos no se solapen.
        """
        truck = normalizar_id(tomar(msg, ["truck", "camion"], ""))
        events = msg.get("events", [])
        secuencia = convertir_int(tomar(msg, ["secuencia", "sequence"], 0), 0)
        request_id = normalizar_id(tomar(msg, ["requestId", "request_id"], ""))
        free_request_id = normalizar_id(tomar(msg, ["freeRequestId", "free_request_id"], ""))
        archivo_rescheduling = tomar(
            msg,
            ["archivoRescheduling", "archivo_rescheduling"],
            os.path.basename(self.trace_csv_default),
        )
        trace_csv = self._resolver_trace_csv(archivo_rescheduling)
        if not truck or not isinstance(events, list) or not events:
            return {"ok": False, "error": "CICLO_INVALIDO"}
        if self.rescheduling_finalizado:
            return {
                "ok": False,
                "error": "RESCHEDULING_FINALIZADO_NO_ACEPTA_CICLOS",
                "truck": truck,
                "secuencia": secuencia,
            }
        if truck in self.trucks_plan_completo:
            return {
                "ok": False,
                "error": "PLAN_CAMION_FINALIZADO_NO_ACEPTA_CICLOS",
                "truck": truck,
                "secuencia": secuencia,
            }

        inicios = [convertir_int(tomar(ev, ["horaInicio", "inicio"], 0), 0) for ev in events if isinstance(ev, dict)]
        fines = [convertir_int(tomar(ev, ["horaFin", "fin"], 0), 0) for ev in events if isinstance(ev, dict)]
        inicio_nuevo = min(inicios) if inicios else 0
        fin_nuevo = max(fines) if fines else 0
        if fin_nuevo <= inicio_nuevo:
            return {"ok": False, "error": "CICLO_INTERVALO_INVALIDO", "inicio": inicio_nuevo, "fin": fin_nuevo}
        if fin_nuevo > HORIZONTE_TURNO_MS:
            return {
                "ok": False,
                "error": "CICLO_FUERA_HORIZONTE_6_HORAS",
                "truck": truck,
                "inicio": inicio_nuevo,
                "fin": fin_nuevo,
                "limite": HORIZONTE_TURNO_MS,
            }

        ciclos_truck = [c for c in self.cycles if c.get("truck") == truck and c.get("state") != "FAILED"]
        for c in ciclos_truck:
            ci = convertir_int(c.get("inicio_ms", 0), 0)
            cf = convertir_int(c.get("fin_ms", 0), 0)
            if inicio_nuevo < cf and fin_nuevo > ci:
                return {"ok": False, "error": "CICLO_SOLAPADO_CAMION", "cycle": c, "inicio": inicio_nuevo, "fin": fin_nuevo}

        # La secuencia debe representar el orden cronológico real del camión.
        # No se usa el orden de llegada del mensaje.
        if secuencia <= 0:
            return {
                "ok": False,
                "error": "SECUENCIA_CICLO_INVALIDA",
                "truck": truck,
                "secuencia": secuencia,
            }

        duplicado = next(
            (
                c for c in ciclos_truck
                if convertir_int(c.get("secuencia", 0), 0) == secuencia
            ),
            None,
        )
        if duplicado is not None:
            return {
                "ok": False,
                "error": "SECUENCIA_CICLO_DUPLICADA",
                "truck": truck,
                "secuencia": secuencia,
                "cycle": duplicado,
            }

        if secuencia == 1:
            # El primer ciclo siempre parte desde el edge físico comunicado
            # mediante TRUCK_FREE.
            libre = self.free.get(truck, {})
            origen_inicial = normalizar_id(tomar(libre, ["edge"], ""))
            if not origen_inicial:
                return {
                    "ok": False,
                    "error": "TRUCK_FREE_SIN_EDGE_REAL",
                    "truck": truck,
                    "free": libre,
                }
        else:
            # Para secuencia N debe existir exactamente N-1. No se toma el ciclo
            # con mayor fin ni el último recibido.
            anterior = next(
                (
                    c for c in ciclos_truck
                    if convertir_int(c.get("secuencia", 0), 0) == secuencia - 1
                ),
                None,
            )
            if anterior is None:
                return {
                    "ok": False,
                    "error": "CICLO_ANTERIOR_NO_RECIBIDO",
                    "truck": truck,
                    "secuencia": secuencia,
                    "secuenciaEsperada": secuencia - 1,
                }

            fin_anterior = convertir_int(anterior.get("fin_ms", 0), 0)
            if inicio_nuevo < fin_anterior:
                return {
                    "ok": False,
                    "error": "CICLOS_TEMPORALMENTE_DESORDENADOS",
                    "truck": truck,
                    "secuencia": secuencia,
                    "inicioNuevo": inicio_nuevo,
                    "finAnterior": fin_anterior,
                    "cycleAnterior": anterior,
                }

            origen_inicial = normalizar_id(anterior.get("edge_final", ""))
            if not origen_inicial:
                return {
                    "ok": False,
                    "error": "CICLO_ANTERIOR_SIN_EDGE_FINAL",
                    "truck": truck,
                    "secuencia": secuencia,
                    "cycle": anterior,
                }

        eventos_ruteados: List[Dict[str, Any]] = []
        # Estado fisico del camion dentro del rescheduling.
        # Desde este punto, el origen de cada viaje se mantiene exclusivamente
        # como edge SUMO. Las ubicaciones logicas solo identifican objetos destino.
        edge_actual = origen_inicial
        ubicacion_logica_actual = ""

        for indice, original in enumerate(events, start=1):
            ev = dict(original)
            op = normalizar_id(tomar(ev, ["operacion", "operation"], "")).upper()
            ev["operacion"] = op
            if not op:
                return {"ok": False, "error": "EVENTO_SIN_OPERACION", "indice": indice, "truck": truck}

            if op in ("VIAJE_VACIO", "VIAJE_CARGADO"):
                # El camion siempre parte desde el edge fisico donde termino el
                # segmento anterior. No se reutiliza origenRuta logico enviado por JADE.
                origen_edge = normalizar_id(edge_actual)
                destino_logico = normalizar_id(tomar(ev, ["destinoRuta", "destino", "ubicacion"], ""))
                if not origen_edge:
                    return {"ok": False, "error": "VIAJE_SIN_ORIGEN_EDGE", "indice": indice, "truck": truck}
                if not destino_logico:
                    return {"ok": False, "error": "VIAJE_SIN_DESTINO_LOGICO", "indice": indice, "truck": truck}

                ruta = calcular_ruta_response(self.net, {
                    "camion": truck,
                    "origen": origen_edge,
                    "destino": destino_logico,
                    "operacion": op,
                    "velocidad": tomar(ev, ["velocidadCargado" if op == "VIAJE_CARGADO" else "velocidadVacio"], ""),
                })
                if not ruta.get("ok"):
                    return {"ok": False, "error": "RUTA_CICLO", "indice": indice, "truck": truck, "detalle": ruta}

                route_edges = [
                    normalizar_id(edge)
                    for edge in str(ruta.get("routeEdges", "") or "").replace(",", " ").split()
                    if normalizar_id(edge)
                ]
                if not route_edges:
                    return {"ok": False, "error": "VIAJE_SIN_ROUTE_EDGES", "indice": indice, "truck": truck}
                if route_edges[0] != origen_edge:
                    return {
                        "ok": False,
                        "error": "RUTA_NO_COMIENZA_EN_ORIGEN_EDGE",
                        "indice": indice,
                        "truck": truck,
                        "origenEdge": origen_edge,
                        "primerEdgeRuta": route_edges[0],
                    }

                ev["origenRuta"] = origen_edge
                ev["origenEdge"] = origen_edge
                ev["destinoRuta"] = destino_logico
                ev["routeEdges"] = " ".join(route_edges)
                ev["cantidadEdges"] = len(route_edges)
                ev["distanciaM"] = ruta.get("distanciaM", 0.0)
                ev["edgeFinal"] = route_edges[-1]

                # Continuidad fisica exacta para el proximo segmento/ciclo.
                edge_actual = route_edges[-1]
                ubicacion_logica_actual = destino_logico

            elif op in ("CARGA", "DESCARGA"):
                ubicacion_evento = normalizar_id(tomar(ev, ["ubicacion", "location"], ""))
                loc = self.net.resolver_ubicacion(ubicacion_evento, como_destino=True)
                if not ubicacion_evento or not loc.get("valid"):
                    return {"ok": False, "error": f"{op}_UBICACION_NO_RESUELTA", "truck": truck, "ubicacion": ubicacion_evento}

                destino_edge = normalizar_id(loc.get("edge", ""))
                if not destino_edge:
                    return {"ok": False, "error": f"{op}_SIN_DESTINO_EDGE", "truck": truck, "ubicacion": ubicacion_evento}

                # La operacion debe ejecutarse exactamente sobre el edge donde
                # termino el viaje inmediatamente anterior.
                if edge_actual and destino_edge != edge_actual:
                    return {
                        "ok": False,
                        "error": f"{op}_EDGE_NO_COINCIDE_CON_VIAJE",
                        "truck": truck,
                        "ubicacion": ubicacion_evento,
                        "edgeViaje": edge_actual,
                        "destinoEdge": destino_edge,
                    }

                ev["destinoEdge"] = destino_edge
                ev["destinoLane"] = normalizar_id(loc.get("lane", ""))
                ev["endPos"] = _to_float(loc.get("endPos", loc.get("departPos", -1.0)), -1.0)
                ubicacion_logica_actual = ubicacion_evento

            eventos_ruteados.append(ev)

        viajes = [ev for ev in eventos_ruteados if str(ev.get("operacion", "")).startswith("VIAJE")]
        if not viajes or any(not normalizar_id(ev.get("routeEdges", "")) for ev in viajes):
            return {"ok": False, "error": "CICLO_SIN_EDGES", "truck": truck}

        # El edge final fisico se obtiene del ultimo routeEdges del ciclo. Este
        # valor sera el origen exacto del siguiente ciclo del mismo camion.
        route_edges_finales = [
            edge
            for edge in str(viajes[-1].get("routeEdges", "") or "").replace(",", " ").split()
            if edge
        ]
        edge_final = normalizar_id(route_edges_finales[-1]) if route_edges_finales else ""
        if not edge_final:
            return {"ok": False, "error": "CICLO_SIN_EDGE_FINAL", "truck": truck}

        self.last_id += 1
        cycle = {
            "id": self.last_id,
            "requestId": request_id,
            "freeRequestId": free_request_id,
            "truck": truck,
            "secuencia": secuencia,
            "archivoRescheduling": os.path.basename(trace_csv),
            "trace_csv": trace_csv,
            "inicio_ms": inicio_nuevo,
            "fin_ms": fin_nuevo,
            "ubicacion_final": ubicacion_logica_actual,
            "edge_final": edge_final,
            "events": eventos_ruteados,
            "state": "RUTEADO",
            "tiempoSumo": "",
        }
        self.cycles.append(cycle)
        self.cycles.sort(key=lambda c: (
            convertir_int(c.get("inicio_ms", 0), 0),
            convertir_int(c.get("secuencia", 0), 0),
            convertir_int(c.get("id", 0), 0),
        ))
        self.trucks_inicializados.add(truck)
        self.registrar_ciclo(cycle, "RUTEADO")
        _trace_truck_free({
            "freeRequestId": free_request_id, "truck": truck, "stage": "SERVER_ADD_CYCLE_ACCEPTED",
            "sim_time_ms": inicio_nuevo, "wall_time": _wall_now(), "cycle_id": self.last_id,
            "secuencia": secuencia,
            "detalle": (
                f"intervalo={inicio_nuevo}-{fin_nuevo};"
                f"origen_edge={origen_inicial};edge_final={edge_final}"
            ),
        })
        print(
            f"[rescheduling] Ciclo {self.last_id} aceptado para {truck} "
            f"secuencia={secuencia} intervalo={inicio_nuevo}-{fin_nuevo} "
            f"origen_edge={origen_inicial} edge_final={edge_final}"
        )
        return {
            "ok": True,
            "id": self.last_id,
            "secuencia": secuencia,
            "requestId": request_id,
            "archivoRescheduling": os.path.basename(trace_csv),
        }

    def completar_plan_camion(self, msg: Dict[str, Any]) -> Dict[str, Any]:
        truck = normalizar_id(tomar(msg, ["truck", "camion"], ""))
        if not truck:
            return {"ok": False, "error": "SIN_TRUCK"}

        self.trucks_plan_completo.add(truck)
        ciclos = convertir_int(tomar(msg, ["cycles", "ciclos"], 0), 0)
        free_request_id = normalizar_id(tomar(msg, ["freeRequestId", "free_request_id"], ""))
        _trace_truck_free({
            "freeRequestId": free_request_id,
            "truck": truck,
            "stage": "SERVER_TRUCK_PLAN_COMPLETE",
            "wall_time": _wall_now(),
            "detalle": f"cycles={ciclos};no_more_cycles_truck=true",
        })
        return {
            "ok": True,
            "truck": truck,
            "cycles": ciclos,
            "no_more_cycles_truck": True,
            "completed_trucks": len(self.trucks_plan_completo),
        }

    def get_cycles(self, after: int = 0) -> Dict[str, Any]:
        rows = []
        for c in self.cycles:
            if int(c.get("id", 0)) <= int(after):
                continue
            if normalizar_id(c.get("state", "")).upper() in ("EJECUTADO", "ERROR"):
                continue
            rows.append(dict(c))
            c["state"] = "SENT"
        return {
            "ok": True,
            "cycles": rows,
            "last": self.last_id,
            "rescheduling_finalizado": self.rescheduling_finalizado,
            "no_more_cycles": self.rescheduling_finalizado,
            "trucks_plan_completo": sorted(self.trucks_plan_completo),
        }

    def ack_cycle(self, msg: Dict[str, Any]) -> Dict[str, Any]:
        cid = convertir_int(tomar(msg, ["id", "cycle_id"], 0), 0)
        estado = normalizar_id(tomar(msg, ["estado", "state"], "ACK")).upper() or "ACK"
        tiempo_sumo = tomar(msg, ["time", "tiempo_sumo_s"], "")
        detalle = tomar(msg, ["detalle", "detail", "error"], "")
        for c in self.cycles:
            if int(c.get("id", 0)) == cid:
                c["state"] = estado
                c["tiempoSumo"] = tiempo_sumo
                c["detalleEstado"] = detalle
                self.registrar_ciclo(c, estado, tiempo_sumo, detalle)
                return {"ok": True, "id": cid, "estado": estado}
        return {"ok": False, "error": "ID_NO_EXISTE"}


    def exportar_csv(self) -> None:
        os.makedirs(os.path.dirname(self.output_csv) or ".", exist_ok=True)
        with open(self.output_csv, "w", newline="", encoding="utf-8-sig") as f:
            writer = csv.DictWriter(f, fieldnames=self.columnas)
            writer.writeheader()
            for r in self.get_requests():
                writer.writerow({c: r.get(c, "") for c in self.columnas})

    def actualizar_camiones(self) -> None:
        """Indexa la disponibilidad final de cada camion calculada por SUMO."""
        self.disponibilidad_por_camion.clear()
        for row in self.snapshot_flota:
            if not isinstance(row, dict):
                continue
            camion = normalizar_id(tomar(row, ["camion", "truck", "vehiculo", "vehicle"], ""))
            if not camion:
                continue
            self.disponibilidad_por_camion[camion] = row

    def actualizar_palas(self, disponibilidad_palas: Any, tiempo_sumo_s: float = 0.0) -> None:
        """
        Indexa la disponibilidad de palas calculada por SUMO/writer.
        Acepta dos formatos:
        - {"PA01": {"tiempo_disponible_ms": ...}}
        - [{"pala": "PA01", "tiempo_disponible_ms": ...}, ...]
        Si no llega informacion, deja el diccionario vacio y JADE usara base global.
        """
        self.disponibilidad_por_pala.clear()

        if isinstance(disponibilidad_palas, dict):
            iterable = disponibilidad_palas.items()
            for pala_key, estado in iterable:
                pala = normalizar_id(pala_key)
                if not isinstance(estado, dict):
                    estado = {"tiempo_disponible_ms": estado}
                if not pala:
                    pala = normalizar_id(tomar(estado, ["pala", "shovel", "id"], ""))
                if not pala:
                    continue
                row = dict(estado)
                row["pala"] = pala
                if "tiempo_disponible_ms" not in row:
                    tiempo_s = _to_float(tomar(row, ["tiempo_disponible_s", "tiempo_disponible_estimado_s"], tiempo_sumo_s), tiempo_sumo_s)
                    row["tiempo_disponible_ms"] = int(round(tiempo_s * 1000.0))
                self.disponibilidad_por_pala[pala] = row

        elif isinstance(disponibilidad_palas, list):
            for estado in disponibilidad_palas:
                if not isinstance(estado, dict):
                    continue
                pala = normalizar_id(tomar(estado, ["pala", "shovel", "id"], ""))
                if not pala:
                    continue
                row = dict(estado)
                row["pala"] = pala
                if "tiempo_disponible_ms" not in row:
                    tiempo_s = _to_float(tomar(row, ["tiempo_disponible_s", "tiempo_disponible_estimado_s"], tiempo_sumo_s), tiempo_sumo_s)
                    row["tiempo_disponible_ms"] = int(round(tiempo_s * 1000.0))
                self.disponibilidad_por_pala[pala] = row

    def estado_pala(self, pala: str) -> Dict[str, Any]:
        pala = normalizar_id(pala)
        row = self.disponibilidad_por_pala.get(pala, {})
        if not row:
            return {}
        tiempo_ms = convertir_int(tomar(row, ["tiempo_disponible_ms", "tiempoDisponibleMs"], 0), 0)
        if tiempo_ms <= 0:
            tiempo_s = _to_float(tomar(row, ["tiempo_disponible_s", "tiempo_disponible_estimado_s"], 0.0), 0.0)
            tiempo_ms = int(round(tiempo_s * 1000.0)) if tiempo_s > 0 else 0
        return {
            "pala": pala,
            "tiempo_disponible_ms": tiempo_ms,
            "tiempo_disponible_s": tiempo_ms / 1000.0 if tiempo_ms else "",
            "motivo": tomar(row, ["motivo", "reason"], ""),
            "camion_comprometido": tomar(row, ["camion_comprometido", "camion", "truck"], ""),
            "snapshot": row,
        }

    def estado_camion(self, camion: str) -> Dict[str, Any]:
        camion = normalizar_id(camion)
        row = self.disponibilidad_por_camion.get(camion, {})
        if not row:
            return {}
        ubicacion_disponible = normalizar_id(tomar(row, ["ubicacion_disponible", "ubicacion_referencia"], ""))
        edge_disponible = normalizar_id(tomar(row, ["edge_disponible", "edge_fin_segmento", "edge_actual"], ""))
        origen = ubicacion_disponible or edge_disponible
        return {
            "camion": camion,
            "origen_rescheduling": origen,
            "ubicacion_disponible": ubicacion_disponible,
            "edge_disponible": edge_disponible,
            "tiempo_disponible_estimado_s": tomar(row, ["tiempo_disponible_estimado_s"], ""),
            "aplicar_desde": tomar(row, ["aplicar_desde"], ""),
            "snapshot": row,
        }

    def origen_camion(self, camion: str) -> str:
        estado = self.estado_camion(camion)
        return normalizar_id(estado.get("origen_rescheduling", ""))

    def exportar_snapshot(self) -> None:
        """Guarda la ultima fotografia de flota recibida desde SUMO."""
        os.makedirs(os.path.dirname(self.snapshot_json) or ".", exist_ok=True)
        data = {
            "ok": True,
            "tiempo_sumo_s": self.snapshot_tiempo_s,
            "cantidad_camiones": len(self.snapshot_flota),
            "snapshot_flota": self.snapshot_flota,
            "cantidad_palas": len(self.disponibilidad_por_pala),
            "disponibilidad_por_pala": self.disponibilidad_por_pala,
        }
        with open(self.snapshot_json, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

        columnas = [
            "tiempo_sumo_s", "camion", "operacion_planificada", "operacion_estimada",
            "seg_num", "ruta", "edge_actual", "lane_actual", "pos_actual",
            "velocidad_actual_s", "route_index", "distancia_restante_segmento_m",
            "edge_fin_segmento", "jade_inicio_s", "jade_fin_s", "accion_rescheduling",
            "disponibilidad", "ubicacion_referencia",
            "ubicacion_disponible", "edge_disponible", "tiempo_disponible_estimado_s",
            "aplicar_desde", "commitment_restante_s", "observacion"
        ]
        with open(self.snapshot_csv, "w", newline="", encoding="utf-8-sig") as f:
            writer = csv.DictWriter(f, fieldnames=columnas)
            writer.writeheader()
            for row in self.snapshot_flota:
                if isinstance(row, dict):
                    writer.writerow({c: row.get(c, "") for c in columnas})

    def get_snapshot(self) -> Dict[str, Any]:
        return {
            "ok": True,
            "tiempo_sumo_s": self.snapshot_tiempo_s,
            "cantidad_camiones": len(self.snapshot_flota),
            "snapshot_flota": self.snapshot_flota,
            "disponibilidad_por_camion": self.disponibilidad_por_camion,
            "disponibilidad_por_pala": self.disponibilidad_por_pala,
            "snapshot_json": self.snapshot_json,
            "snapshot_csv": self.snapshot_csv,
        }

    def resumen(self) -> Dict[str, Any]:
        conteo = defaultdict(int)
        for r in self.requests.values():
            conteo[normalizar_id(r.get("estado", "SIN_ESTADO"))] += 1
        return {
            "total": len(self.requests),
            "por_estado": dict(conteo),
            "csv": self.output_csv,
            "snapshot_json": self.snapshot_json,
            "snapshot_csv": self.snapshot_csv,
            "snapshot_camiones": len(self.snapshot_flota),
            "disponibilidad_camiones": len(self.disponibilidad_por_camion),
            "disponibilidad_palas": len(self.disponibilidad_por_pala),
        }

# ============================================================
# Comandos ZMQ
# ============================================================

def procesar_json(msg: Dict[str, Any], memoria: MemoriaCronograma, rescheduler: ReschedulingManager) -> Dict[str, Any]:
    tipo = str(msg.get("type", "")).upper().strip()

    if tipo == "JADE_METRICS":
        return guardar_metricas_jade(
            payload=msg,
            escenario=ESCENARIO_ACTUAL,
            output_dir=ESCENARIO_OUTPUT_DIR,
        )

    if tipo == "PING":
        return {"ok": True, "msg": "PONG", "eventos_en_memoria": len(memoria.eventos)}

    if tipo == "GET_FLEET":
        return construir_flota(memoria.net)

    if tipo == "ROUTE":
        msg_route = dict(msg)
        camion = normalizar_id(tomar(msg_route, ["camion", "truck", "vehiculo", "vehicle"], ""))
        origen = normalizar_id(tomar(msg_route, ["origen", "from", "inicio"], ""))
        # Despues de un rescheduling global, si JADE vuelve a consultar rutas desde TruckX,
        # el origen operativo debe ser la ubicacion donde SUMO calculo que el camion queda disponible
        # al terminar su compromiso actual, no la ubicacion inicial antigua del trucks.xml.
        if camion and (not origen or origen == camion):
            origen_rs = rescheduler.origen_camion(camion)
            if origen_rs:
                msg_route["origen"] = origen_rs
                msg_route["origen_rescheduling_usado"] = True
        resp = calcular_ruta_response(memoria.net, msg_route)
        if msg_route.get("origen_rescheduling_usado"):
            resp["origenReschedulingUsado"] = True
            resp["origenRescheduling"] = msg_route.get("origen", "")
            resp["estadoReschedulingCamion"] = rescheduler.estado_camion(camion)
        return resp

    if tipo == "REGISTER_BLOCKED_EDGE":
        edge = normalizar_id(tomar(msg, ["edge", "edge_id"], ""))
        if not edge:
            return {"ok": False, "error": "EDGE_BLOQUEADO_VACIO"}
        EDGES_BLOQUEADOS_ACTUALES.add(edge)
        return {"ok": True, "edge": edge, "blockedEdges": sorted(EDGES_BLOQUEADOS_ACTUALES)}

    if tipo == "UNREGISTER_BLOCKED_EDGE":
        edge = normalizar_id(tomar(msg, ["edge", "edge_id"], ""))
        EDGES_BLOQUEADOS_ACTUALES.discard(edge)
        return {"ok": True, "edge": edge, "blockedEdges": sorted(EDGES_BLOQUEADOS_ACTUALES)}

    if tipo == "GET_BLOCKED_EDGES":
        return {"ok": True, "blockedEdges": sorted(EDGES_BLOQUEADOS_ACTUALES)}

    if tipo == "DIAGNOSTIC":
        return {"ok": True, "diagnostico": memoria.net.diagnostico() if hasattr(memoria.net, "diagnostico") else {}}

    if tipo in ("RESET", "RESET_INITIAL"):
        memoria.reset()
        rescheduler.reset()
        return {"ok": True, "msg": "Memoria y solicitudes de rescheduling limpiadas"}

    if tipo == "RESET_EVENTS_ONLY":
        # Limpia únicamente los eventos que JADE volverá a generar.
        # Conserva la solicitud y el snapshot del rescheduling.
        memoria.reset()
        rescheduler.rescheduling_finalizado = False
        return {"ok": True, "msg": "Eventos del cronograma limpiados; rescheduling abierto"}

    if tipo == "ADD_EVENT":
        evento = memoria.agregar_evento(msg.get("event", {}))
        return {"ok": True, "msg": "Evento agregado", "event": evento, "eventos_en_memoria": len(memoria.eventos)}

    if tipo == "ADD_EVENTS":
        eventos = msg.get("events", [])
        if not isinstance(eventos, list):
            return {"ok": False, "error": "El campo events debe ser una lista"}
        resultado = memoria.agregar_eventos(eventos, limpiar=False)
        return {"ok": len(resultado["errores"]) == 0, "msg": "Eventos procesados", **resultado}

    if tipo == "SET_CRONOGRAMA":
        eventos = msg.get("events", [])
        if not isinstance(eventos, list):
            return {"ok": False, "error": "El campo events debe ser una lista"}
        resultado = memoria.agregar_eventos(eventos, limpiar=True)
        return {"ok": len(resultado["errores"]) == 0, "msg": "Cronograma reemplazado en memoria", **resultado}

    if tipo == "VALIDATE":
        return {"ok": True, "validacion": memoria.validar()}

    if tipo == "GET_CRONOGRAMA":
        return {"ok": True, "eventos": memoria.eventos, "validacion": memoria.validar()}

    if tipo in ("FINALIZE", "FINALIZE_INITIAL"):
        memoria.ordenar()
        memoria.exportar_csv(CRONOGRAMA_CSV)
        memoria.exportar_json(CRONOGRAMA_JSON)
        resultado = generar_archivos_sumo(memoria)
        return {
            "ok": bool(resultado.get("ok")),
            "msg": "Scheduling inicial generado",
            **resultado,
        }

    if tipo == "MARK_RESCHEDULE_GENERATING":
        # Se abre una nueva etapa incremental. FINALIZE_RESCHEDULE la cerrará.
        rescheduler.rescheduling_finalizado = False
        rescheduler.trucks_plan_completo.clear()
        return {
            "ok": True,
            "msg": "Rescheduling en generación",
            "rescheduling_finalizado": False,
            "no_more_cycles": False,
        }

    if tipo == "TRUCK_PLAN_COMPLETE":
        return rescheduler.completar_plan_camion(msg)

    if tipo == "FINALIZE_RESCHEDULE":
        # Señal definitiva: JADE ya envió todos los ciclos confirmados.
        # SUMO debe ejecutar solamente los ciclos activos/encolados y cerrar.
        rescheduler.rescheduling_finalizado = True
        _trace_truck_free({
            "freeRequestId": "", "truck": "GLOBAL", "stage": "SERVER_FINALIZE_RESCHEDULE",
            "sim_time_ms": convertir_int(tomar(msg, ["base_time_ms"], 0), 0),
            "wall_time": _wall_now(),
            "detalle": (
                f"free_registrados={len(rescheduler.free)};"
                f"trucks_con_ciclo={len(rescheduler.trucks_inicializados)};"
                "no_more_cycles=true"
            ),
        })
        return {
            "ok": True,
            "msg": "Rescheduling finalizado; no se recibirán más ciclos",
            "rescheduling_finalizado": True,
            "no_more_cycles": True,
            "ciclos_registrados": len(rescheduler.cycles),
        }


    if tipo in ("REGISTER_RESCHEDULE_REQUEST", "RESCHEDULE_REQUEST"):
        return rescheduler.registrar(msg)

    if tipo == "GET_RESCHEDULE_REQUESTS":
        estado = normalizar_id(tomar(msg, ["estado", "status"], ""))
        return {
            "ok": True,
            "requests": rescheduler.get_requests(estado),
            "resumen": rescheduler.resumen(),
        }

    if tipo in ("GET_RESCHEDULE_SNAPSHOT", "GET_RESCHEDULING_SNAPSHOT", "GET_FLEET_SNAPSHOT"):
        return rescheduler.get_snapshot()

    if tipo in ("GET_TRUCK_RESCHEDULE_STATE", "GET_TRUCK_RESCHEDULING_STATE"):
        camion = normalizar_id(tomar(msg, ["camion", "truck", "vehiculo", "vehicle"], ""))
        estado = rescheduler.estado_camion(camion)
        return {"ok": bool(estado), "camion": camion, "estado": estado, "error": "" if estado else "SIN_SNAPSHOT_CAMION"}

    if tipo in ("GET_SHOVEL_RESCHEDULE_STATE", "GET_SHOVEL_RESCHEDULING_STATE"):
        pala = normalizar_id(tomar(msg, ["pala", "shovel", "id"], ""))
        estado = rescheduler.estado_pala(pala)
        return {"ok": bool(estado), "pala": pala, "estado": estado, "error": "" if estado else "SIN_SNAPSHOT_PALA"}

    if tipo == "ACK_RESCHEDULE_REQUEST":
        return rescheduler.ack(msg)

    if tipo == "COMPLETE_RESCHEDULE_REQUEST":
        return rescheduler.completar(msg)

    if tipo == "RESET_RESCHEDULING":
        rescheduler.reset()
        return {"ok": True, "msg": "Solicitudes de rescheduling limpiadas", "resumen": rescheduler.resumen()}


    if tipo == "TRUCK_FREE":
        return rescheduler.add_free(msg)

    if tipo == "GET_FREE":
        return rescheduler.get_free()

    if tipo == "ADD_CYCLE":
        return rescheduler.add_cycle(msg)

    if tipo == "GET_CYCLES":
        return rescheduler.get_cycles(
            convertir_int(tomar(msg, ["after"], 0), 0)
        )

    if tipo == "ACK_CYCLE":
        return rescheduler.ack_cycle(msg)

    if tipo == "CLOSE":
        return {"ok": True, "closing": True, "msg": "Cerrando server.py"}

    return {"ok": False, "error": f"Tipo de mensaje desconocido: {tipo}"}


def procesar_texto(req: str, memoria: MemoriaCronograma, rescheduler: ReschedulingManager) -> str:
    req = req.strip()
    if req == "hola":
        return "ok"
    if req == "sim.close":
        return json.dumps({"ok": True, "closing": True}, ensure_ascii=False)
    if req == "fleet.get":
        return json.dumps(construir_flota(memoria.net), ensure_ascii=False)
    if req == "routing.diagnostic":
        return json.dumps({"ok": True, "diagnostico": memoria.net.diagnostico() if hasattr(memoria.net, "diagnostico") else {}}, ensure_ascii=False)
    if req == "cron.validate":
        return json.dumps({"ok": True, "validacion": memoria.validar()}, ensure_ascii=False)
    if req == "cron.get":
        return json.dumps({"ok": True, "eventos": memoria.eventos, "validacion": memoria.validar()}, ensure_ascii=False)
    if req == "cron.finalize":
        memoria.ordenar()
        memoria.exportar_csv(CRONOGRAMA_CSV)
        memoria.exportar_json(CRONOGRAMA_JSON)
        resultado = generar_archivos_sumo(memoria)
        return json.dumps({"ok": bool(resultado.get("ok")), "msg": "cron.finalize procesado", **resultado}, ensure_ascii=False)
    if req == "cron.regenerar":
        if not os.path.exists(CRONOGRAMA_JSON):
            return json.dumps({"ok": False, "error": f"No existe {CRONOGRAMA_JSON}."}, ensure_ascii=False)
        memoria.reset()
        with open(CRONOGRAMA_JSON, "r", encoding="utf-8") as f:
            data = json.load(f)
        memoria.agregar_eventos(data.get("eventos", []), limpiar=True)
        memoria.ordenar()
        memoria.exportar_csv(CRONOGRAMA_CSV)
        memoria.exportar_json(CRONOGRAMA_JSON)
        resultado = generar_archivos_sumo(memoria)
        return json.dumps({"ok": bool(resultado.get("ok")), "msg": "Regenerado desde cronograma_sumo_memoria.json", **resultado}, ensure_ascii=False)

    if req == "rescheduling.get":
        return json.dumps({"ok": True, "requests": rescheduler.get_requests(), "resumen": rescheduler.resumen()}, ensure_ascii=False)
    if req == "rescheduling.pending":
        return json.dumps({"ok": True, "requests": rescheduler.get_requests("PENDIENTE"), "resumen": rescheduler.resumen()}, ensure_ascii=False)
    if req == "rescheduling.clear":
        rescheduler.reset()
        return json.dumps({"ok": True, "msg": "Solicitudes de rescheduling limpiadas", "resumen": rescheduler.resumen()}, ensure_ascii=False)
    return json.dumps({"ok": False, "error": f"Comando desconocido: {req}"}, ensure_ascii=False)


def procesar_comando(req: str, memoria: MemoriaCronograma, rescheduler: ReschedulingManager) -> str:
    req = req.strip()
    if not req:
        return json.dumps({"ok": False, "error": "Mensaje vacio"}, ensure_ascii=False)
    if req.startswith("{"):
        try:
            msg = json.loads(req)
            return json.dumps(procesar_json(msg, memoria, rescheduler), ensure_ascii=False)
        except Exception as e:
            return json.dumps({"ok": False, "error": str(e), "raw": req}, ensure_ascii=False)
    return procesar_texto(req, memoria, rescheduler)


def main() -> None:
    parser = argparse.ArgumentParser(description="Servidor JADE -> SUMO por escenario")
    parser.add_argument(
        "--scenario", "--escenario",
        default=os.environ.get("SUMO_SCENARIO", "small"),
        help="Escenario: small/pequeno, mediano/medium o grande/large",
    )
    args = parser.parse_args()

    print("============================================================")
    print("SERVER JADE -> SUMO XML / ROUTING / FLEET")
    print("============================================================")
    print(f"[server] Puerto fijo ZMQ: {PORT}")
    print("------------------------------------------------------------")

    try:
        escenario = normalizar_escenario(args.scenario)
        config_entrada = configurar_escenario_routing(escenario)
        config_salida = configurar_salida(escenario)
        configurar_rutas(config_salida)

        # La red siempre permanece en el directorio de server.py.
        # Solo objects.xml y trucks.xml cambian según el escenario.
        for etiqueta in ("net", "objects", "trucks"):
            path = config_entrada[etiqueta]
            if not os.path.exists(path):
                raise FileNotFoundError(
                    f"No existe {etiqueta} para el escenario {escenario}: {path}"
                )

        net = NetSumo(
            config_entrada["net"],
            objects_path=config_entrada["objects"],
            trucks_path=config_entrada["trucks"],
        )
    except Exception as e:
        print("[server] ERROR inicializando escenario/net/routing_service:")
        print(e)
        sys.exit(1)

    print(f"[server] Escenario activo: {ESCENARIO_ACTUAL}")
    print(f"[server] Red común: {net.path}")
    print(f"[server] Objects de entrada: {net.objects_path}")
    print(f"[server] Trucks de entrada: {net.trucks_path}")
    print(f"[server] Carpeta de resultados: {ESCENARIO_OUTPUT_DIR}")
    print(f"[server] Cronograma CSV: {CRONOGRAMA_CSV}")

    # Registra el proceso del servidor para que metricas.py pueda incluirlo
    # en las muestras de CPU y memoria del escenario activo.
    try:
        registro_pid = registrar_componente(
            escenario=ESCENARIO_ACTUAL,
            componente="server",
            pid=os.getpid(),
            output_dir=ESCENARIO_OUTPUT_DIR,
        )
        print(f"[metricas] PID server registrado: {registro_pid.get('pid')}")
    except Exception as exc:
        print(f"[metricas] advertencia registrando PID server: {exc}")
    print(f"[server] Cronograma JSON: {CRONOGRAMA_JSON}")
    print("------------------------------------------------------------")

    memoria = MemoriaCronograma(net)
    rescheduler = ReschedulingManager(net)

    ctx = zmq.Context()
    sock = ctx.socket(zmq.REP)
    sock.bind(f"tcp://*:{PORT}")

    print(f"[server] Esperando mensajes desde JADE en tcp://localhost:{PORT}")
    print("[server] JSON: PING | GET_FLEET | ROUTE | ADD_EVENT(S) | FINALIZE_INITIAL | ADD_CYCLE | TRUCK_PLAN_COMPLETE | JADE_METRICS | CLOSE")
    print("[server] JSON rescheduling: REGISTER_RESCHEDULE_REQUEST | GET_RESCHEDULE_REQUESTS | GET_RESCHEDULE_SNAPSHOT | GET_TRUCK_RESCHEDULE_STATE | ACK_RESCHEDULE_REQUEST")
    print("[server] Texto: hola | fleet.get | routing.diagnostic | cron.validate | cron.finalize | sim.close")
    print("[server] Texto rescheduling: rescheduling.get | rescheduling.pending | rescheduling.clear")
    print("------------------------------------------------------------")

    try:
        while True:
            req = sock.recv_string()
            resp = procesar_comando(req, memoria, rescheduler)
            sock.send_string(resp)
            try:
                parsed = json.loads(resp)
                if parsed.get("closing"):
                    break
            except Exception:
                pass
    except KeyboardInterrupt:
        print("\n[server] Interrumpido por usuario.")
    except Exception as e:
        print("[server] EXCEPCION NO CONTROLADA:", e)
        traceback.print_exc()
    finally:
        sock.close(0)
        ctx.term()
        print("[server] Conexion cerrada.")


if __name__ == "__main__":
    main()
