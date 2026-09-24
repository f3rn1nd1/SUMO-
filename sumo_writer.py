# -*- coding: utf-8 -*-
"""
sumo_writer.py
Genera archivos SUMO desde la memoria del cronograma:
- mntrucks_generado.rou.xml
- mntrucks_generado.trips.xml
- mnobject_generado.add.xml
- <escenario>_generado.sumocfg
- run_sumo_velocidad_jade.py
"""

from __future__ import annotations

import csv
import os
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List, Tuple

from routing_service import (
    normalizar_id, convertir_int, ms_a_seg, id_xml_seguro, parsear_velocidad_ms,
    es_operacion_viaje, es_carga_descarga, es_camion, indent_xml,
    VELOCIDAD_DEF_VACIO_KMH, VELOCIDAD_DEF_CARGADO_KMH, VELOCIDAD_MAXIMA_SEGURA_MS,
    ESPERA_INACTIVA_MIN_MS, normalizar_escenario, directorio_salida,
)

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
BASE_CONFIG = os.path.join(PROJECT_ROOT, "config")
ESCENARIO = "small"
ESCENARIO_DIR = directorio_salida(ESCENARIO)
# Alias conservado para compatibilidad con código anterior.
SMALL_DIR = ESCENARIO_DIR
CRONOGRAMA_CSV = ""
CRONOGRAMA_JSON = ""
TRIPS_XML = ""
ROU_XML = ""
OBJECTS_GENERADO_XML = ""
SUMOCFG_GENERADO = ""
TRACI_RUNNER_GENERADO = ""


def configurar_salida(escenario: Any = "small") -> Dict[str, str]:
    """Define todos los archivos generados dentro de config/<escenario>."""
    global ESCENARIO, ESCENARIO_DIR, SMALL_DIR
    global CRONOGRAMA_CSV, CRONOGRAMA_JSON, TRIPS_XML, ROU_XML
    global OBJECTS_GENERADO_XML, SUMOCFG_GENERADO, TRACI_RUNNER_GENERADO

    ESCENARIO = normalizar_escenario(escenario)
    ESCENARIO_DIR = directorio_salida(ESCENARIO)
    SMALL_DIR = ESCENARIO_DIR

    CRONOGRAMA_CSV = os.path.join(ESCENARIO_DIR, "cronograma_sumo.csv")
    CRONOGRAMA_JSON = os.path.join(ESCENARIO_DIR, "cronograma_sumo_memoria.json")
    TRIPS_XML = os.path.join(ESCENARIO_DIR, "mntrucks_generado.trips.xml")
    ROU_XML = os.path.join(ESCENARIO_DIR, "mntrucks_generado.rou.xml")
    OBJECTS_GENERADO_XML = os.path.join(ESCENARIO_DIR, "mnobject_generado.add.xml")
    SUMOCFG_GENERADO = os.path.join(ESCENARIO_DIR, f"{ESCENARIO}_generado.sumocfg")
    TRACI_RUNNER_GENERADO = os.path.join(ESCENARIO_DIR, "run_sumo_velocidad_jade.py")

    os.makedirs(ESCENARIO_DIR, exist_ok=True)
    return {
        "escenario": ESCENARIO,
        "output_dir": ESCENARIO_DIR,
        "cronograma_csv": CRONOGRAMA_CSV,
        "cronograma_json": CRONOGRAMA_JSON,
        "trips_xml": TRIPS_XML,
        "rou_xml": ROU_XML,
        "objects_add": OBJECTS_GENERADO_XML,
        "sumocfg": SUMOCFG_GENERADO,
        "runner": TRACI_RUNNER_GENERADO,
    }


configurar_salida(os.environ.get("SUMO_SCENARIO", "small"))

# ============================================================
# Generador SUMO
# ============================================================
# ============================================================
# PRUEBA: velocidades estáticas por estado operacional
# ============================================================
# Deben coincidir con las constantes usadas en server.py.
# Aquí se expresan en m/s porque SUMO trabaja con m/s.
USAR_VELOCIDADES_ESTATICAS = False
VELOCIDAD_ESTATICA_VACIO_MS = 6.5
VELOCIDAD_ESTATICA_CARGADO_MS = 5.2


def velocidad_estatica_ms(operacion: str) -> float:
    op = str(operacion or "").upper()
    if op == "VIAJE_CARGADO":
        return min(VELOCIDAD_MAXIMA_SEGURA_MS, VELOCIDAD_ESTATICA_CARGADO_MS)
    return min(VELOCIDAD_MAXIMA_SEGURA_MS, VELOCIDAD_ESTATICA_VACIO_MS)



def velocidad_camion(ev: Dict[str, Any], operacion: str) -> float:
    op = str(operacion or "").upper()

    # En esta prueba todos los camiones usan la misma velocidad base
    # según el estado operacional. No se aplican aumentos ni compensaciones.
    if USAR_VELOCIDADES_ESTATICAS:
        return velocidad_estatica_ms(op)

    if op == "VIAJE_CARGADO":
        return min(VELOCIDAD_MAXIMA_SEGURA_MS, parsear_velocidad_ms(ev.get("velocidadCargado", ""), VELOCIDAD_DEF_CARGADO_KMH))
    return min(VELOCIDAD_MAXIMA_SEGURA_MS, parsear_velocidad_ms(ev.get("velocidadVacio", ""), VELOCIDAD_DEF_VACIO_KMH))


def parse_route_edges(valor: Any) -> List[str]:
    txt = str(valor or "").replace(",", " ").strip()
    if not txt:
        return []
    return [normalizar_id(x) for x in txt.split() if normalizar_id(x)]


def limpiar_edges(route_edges: List[str]) -> List[str]:
    """
    SUMO no acepta una ruta con el mismo edge pegado dos veces, por ejemplo:
    A B B C.

    Esto puede pasar al unir segmentos JADE porque un viaje termina en un edge
    y el siguiente viaje parte desde ese mismo edge. No es un cambio de ruta;
    solo evita duplicar el edge de empalme.
    """
    salida: List[str] = []
    for edge in route_edges:
        edge = normalizar_id(edge)
        if not edge:
            continue
        if salida and salida[-1] == edge:
            continue
        salida.append(edge)
    return salida


def unir_segmento(route_total: List[str], route_edges: List[str]) -> List[str]:
    """
    Devuelve los edges que deben agregarse al route_total del vehículo.
    Si el primer edge del nuevo segmento es igual al último ya agregado, se omite
    para no generar una transición edge->mismo edge, que SUMO rechaza.
    """
    limpios = limpiar_edges(route_edges)
    if route_total and limpios and route_total[-1] == limpios[0]:
        limpios = limpios[1:]
    return limpios


def distancia_edges(net: NetSumo, route_edges: List[str]) -> float:
    total = 0.0
    for edge_id in route_edges:
        total += float(net.edge_length.get(edge_id, 0.0))
    return total


# Parámetros físicos usados para transformar el tiempo objetivo de JADE
# en una velocidad de ejecución de SUMO. No modifican el cronograma.
ACELERACION_CAMION_MS2 = 2.0
DESACELERACION_CAMION_MS2 = 4.5


def estimar_tiempo(
    distancia_m: float,
    velocidad_max_ms: float,
    aceleracion_ms2: float = ACELERACION_CAMION_MS2,
    desaceleracion_ms2: float = DESACELERACION_CAMION_MS2,
) -> float:
    """
    Estima el tiempo de un viaje que parte detenido, acelera, circula y frena
    hasta detenerse en el stop siguiente.

    Si el segmento es demasiado corto para alcanzar velocidad_max_ms, usa un
    perfil triangular (solo aceleración y frenado).
    """
    distancia_m = max(0.0, float(distancia_m))
    velocidad_max_ms = max(0.1, float(velocidad_max_ms))
    aceleracion_ms2 = max(0.1, float(aceleracion_ms2))
    desaceleracion_ms2 = max(0.1, float(desaceleracion_ms2))

    if distancia_m <= 0:
        return 0.0

    dist_acel = (velocidad_max_ms ** 2) / (2.0 * aceleracion_ms2)
    dist_freno = (velocidad_max_ms ** 2) / (2.0 * desaceleracion_ms2)

    if dist_acel + dist_freno >= distancia_m:
        velocidad_pico = (
            (2.0 * distancia_m)
            / ((1.0 / aceleracion_ms2) + (1.0 / desaceleracion_ms2))
        ) ** 0.5
        return velocidad_pico / aceleracion_ms2 + velocidad_pico / desaceleracion_ms2

    tiempo_acel = velocidad_max_ms / aceleracion_ms2
    tiempo_freno = velocidad_max_ms / desaceleracion_ms2
    distancia_constante = distancia_m - dist_acel - dist_freno
    tiempo_constante = distancia_constante / velocidad_max_ms
    return tiempo_acel + tiempo_constante + tiempo_freno


def buscar_stop_posterior(
    eventos_camion: List[Dict[str, Any]],
    indice_viaje: int,
    operacion_viaje: str,
    todos_eventos: List[Dict[str, Any]] = None,
    camion: str = ""
) -> Dict[str, Any]:
    op_esperada = "CARGA" if operacion_viaje == "VIAJE_VACIO" else "DESCARGA" if operacion_viaje == "VIAJE_CARGADO" else ""
    if not op_esperada:
        return {}

    ev_viaje = eventos_camion[indice_viaje]
    camion = normalizar_id(camion or ev_viaje.get("maquina", ""))
    ubicacion_destino = normalizar_id(ev_viaje.get("ubicacion", ""))
    fin_viaje = convertir_int(ev_viaje.get("horaFin", 0))

    def construir_stop(ev: Dict[str, Any], fuente: str) -> Dict[str, Any]:
        duracion_s = ms_a_seg(ev.get("duracion", 0))
        if duracion_s <= 0:
            return {}

        return {
            "operacion_stop": normalizar_id(ev.get("operacion", "")).upper(),
            "duration": duracion_s,
            "duration_ms": convertir_int(ev.get("duracion", 0)),
            "ubicacion": normalizar_id(ev.get("ubicacion", "")),
            "horaInicio": convertir_int(ev.get("horaInicio", 0)),
            "horaFin": convertir_int(ev.get("horaFin", 0)),
            "fuente_asociacion": fuente,
            "fuente_stop": normalizar_id(ev.get("maquina", "")),
        }

    # 1) Caso normal: la CARGA/DESCARGA viene inmediatamente después en el mismo camión.
    if indice_viaje + 1 < len(eventos_camion):
        siguiente = eventos_camion[indice_viaje + 1]
        op_sig = normalizar_id(siguiente.get("operacion", "")).upper()

        if op_sig == op_esperada:
            stop = construir_stop(siguiente, "SECUENCIA_CAMION")
            if stop:
                return stop

    # 2) Respaldo: la CARGA puede estar guardada como evento de la pala.
    if todos_eventos:
        candidatos = []

        for ev in todos_eventos:
            op = normalizar_id(ev.get("operacion", "")).upper()
            if op != op_esperada:
                continue

            maquina = normalizar_id(ev.get("maquina", ""))
            propietaria = normalizar_id(ev.get("maquinaPropietariaSchedule", ""))
            ubicacion = normalizar_id(ev.get("ubicacion", ""))

            if camion not in (maquina, propietaria):
                continue

            if ubicacion != ubicacion_destino:
                continue

            inicio = convertir_int(ev.get("horaInicio", 0))

            # Debe empezar justo cuando termina el viaje o muy cerca.
            diferencia = abs(inicio - fin_viaje)
            if diferencia <= 1000:
                candidatos.append((diferencia, inicio, ev))

        if candidatos:
            candidatos.sort(key=lambda x: (x[0], x[1]))
            stop = construir_stop(candidatos[0][2], "BUSQUEDA_GLOBAL_PALA")
            if stop:
                return stop

    return {}

def buscar_espera(
    eventos_camion: List[Dict[str, Any]],
    indice_viaje: int,
    stop_descarga: Dict[str, Any],
    camion: str = ""
) -> Dict[str, Any]:
    if normalizar_id(stop_descarga.get("operacion_stop", "")).upper() != "DESCARGA":
        return {}

    idx_descarga = None
    hora_inicio_stop = convertir_int(stop_descarga.get("horaInicio", 0))
    hora_fin_stop = convertir_int(stop_descarga.get("horaFin", 0))

    for j in range(indice_viaje + 1, len(eventos_camion)):
        ev = eventos_camion[j]
        op = normalizar_id(ev.get("operacion", "")).upper()
        if op != "DESCARGA":
            continue

        inicio = convertir_int(ev.get("horaInicio", 0))
        fin = convertir_int(ev.get("horaFin", 0))

        if abs(inicio - hora_inicio_stop) <= 1000 or abs(fin - hora_fin_stop) <= 1000:
            idx_descarga = j
            break

        if inicio > hora_inicio_stop + 1000:
            break

    if idx_descarga is None:
        return {}

    if idx_descarga + 1 >= len(eventos_camion):
        return {}

    siguiente = eventos_camion[idx_descarga + 1]
    op_sig = normalizar_id(siguiente.get("operacion", "")).upper()
    if op_sig != "VIAJE_VACIO":
        return {}

    fin_descarga = convertir_int(eventos_camion[idx_descarga].get("horaFin", hora_fin_stop))
    inicio_siguiente = convertir_int(siguiente.get("horaInicio", 0))
    espera_ms = inicio_siguiente - fin_descarga

    if espera_ms < ESPERA_INACTIVA_MIN_MS:
        return {}

    return {
        "operacion_stop": "ESPERA_INACTIVA",
        "duration": espera_ms / 1000.0,
        "duration_ms": espera_ms,
        "ubicacion": normalizar_id(stop_descarga.get("ubicacion", "")),
        "horaInicio": fin_descarga,
        "horaFin": inicio_siguiente,
        "fuente_asociacion": "GAP_DESCARGA_A_VIAJE_VACIO",
        "fuente_stop": normalizar_id(camion),
        "after_operacion": "DESCARGA",
    }


ESPERA_DISPONIBLE_S = 1000000000.0


def cargar_camiones(memoria) -> List[Dict[str, Any]]:
    """Lee la flota completa de trucks.xml, incluidos camiones sin scheduling."""
    path = str(getattr(memoria.net, "trucks_path", "") or "")
    if not path or not os.path.exists(path):
        return []
    salida = []
    root = ET.parse(path).getroot()
    for tr in root.findall("truck"):
        camion = normalizar_id(tr.get("jadeId", ""))
        if not camion:
            continue
        loc = tr.find("location")
        edge = normalizar_id(loc.get("edge", "")) if loc is not None else ""
        lane = normalizar_id(loc.get("lane", "")) if loc is not None else ""
        try:
            pos = float((loc.get("departPos", loc.get("startPos", "0")) if loc is not None else "0") or 0.0)
        except Exception:
            pos = 0.0
        vacio_txt = (tr.findtext("emptySpeed") or str(VELOCIDAD_DEF_VACIO_KMH)).strip()
        cargado_txt = (tr.findtext("loadedSpeed") or str(VELOCIDAD_DEF_CARGADO_KMH)).strip()
        salida.append({
            "camion": camion, "edge": edge, "lane": lane, "pos": pos,
            "vacio": vacio_txt, "cargado": cargado_txt,
        })
    return salida


def parking_operacional_id(ubicacion: Any, lane: Any) -> str:
    """ID estable del parking usado exclusivamente para CARGA/DESCARGA."""
    return f"PA_AUTO_{id_xml_seguro(normalizar_id(ubicacion))}_{id_xml_seguro(normalizar_id(lane))}"


def parking_espera_id(ubicacion: Any, lane: Any) -> str:
    """ID estable del parking fuera de flujo usado para esperas entre ciclos/finales."""
    return f"PA_WAIT_{id_xml_seguro(normalizar_id(ubicacion))}_{id_xml_seguro(normalizar_id(lane))}"


# Zonas seguras predefinidas para detener un camión averiado sin obligarlo a
# completar primero la CARGA o la DESCARGA. Se crean sobre corredores que forman
# parte de las rutas principales y en ambos sentidos de circulación.
PA_SAFE_LANES: List[Tuple[str, str]] = [
    ("OESTE_IDA", "camino_NR330_NR202_0"),
    ("OESTE_VUELTA", "camino_NR202_NR330_0"),
    ("CENTRAL_IDA", "camino_NU42_NU43_0"),
    ("CENTRAL_VUELTA", "camino_NU43_NU42_0"),
    ("MEDIO_IDA", "camino_NU60_NU49_0"),
    ("MEDIO_VUELTA", "camino_NU49_NU60_0"),
    ("ESTE_IDA", "camino_NU64_NU73_0"),
    ("ESTE_VUELTA", "camino_NU73_NU64_0"),
    ("UB1_IDA", "camino_NU76_NU81_0"),
    ("UB1_VUELTA", "camino_NU81_NU76_0"),
]


def parking_safe_id(nombre: Any) -> str:
    """ID estable de una zona segura utilizada exclusivamente por averías."""
    return f"PA_SAFE_{id_xml_seguro(normalizar_id(nombre))}"


def construir_trips(memoria: MemoriaCronograma) -> Tuple[List[Dict[str, Any]], List[str], Dict[str, Dict[str, Any]]]:
    
    vehicles = []
    advertencias = []
    vtypes: Dict[str, Dict[str, Any]] = {}
    camiones_con_eventos = {normalizar_id(c) for c in memoria.por_camion.keys() if es_camion(c)}

    for camion, eventos in sorted(memoria.por_camion.items()):
        if not es_camion(camion):
            continue

        ubicacion_actual = ""
        ubicacion_inicial = ""
        route_total: List[str] = []
        stops: List[Dict[str, Any]] = []
        segmentos: List[Dict[str, Any]] = []
        first_depart_ms = None
        max_speed_truck = 0.1
        empty_speed_ref = ""
        loaded_speed_ref = ""

        for idx, ev in enumerate(eventos):
            operacion = normalizar_id(ev.get("operacion", "")).upper()
            ubicacion_evento = normalizar_id(ev.get("ubicacion", ""))

            if not ubicacion_actual:
                ubicacion_actual = normalizar_id(ev.get("ubicacionInicialCamion", ""))
                ubicacion_inicial = ubicacion_actual

            if es_operacion_viaje(operacion):
                origen = normalizar_id(ev.get("origenRuta", "")) or ubicacion_actual
                destino = normalizar_id(ev.get("destinoRuta", "")) or ubicacion_evento

                # Prioridad: si JADE mando routeEdges exacto, se usa eso.
                # Solo si viene vacio, el server calcula ruta como respaldo.
                route_edges = parse_route_edges(ev.get("routeEdges", ""))
                if not route_edges:
                    route_edges, estado = memoria.net.shortest_edges(origen, destino)
                else:
                    estado = "OK_ROUTE_EDGES_JADE"

                if not route_edges:
                    advertencias.append(
                        f"{camion}: {operacion} sin ruta SUMO {origen}->{destino}. Estado={estado}. Segmento omitido."
                    )
                    ubicacion_actual = destino or ubicacion_evento or ubicacion_actual
                    continue

                if first_depart_ms is None:
                    first_depart_ms = convertir_int(ev.get("horaInicio", 0))

                # Validar la continuidad fisica antes de concatenar segmentos.
                # Si el primer edge del nuevo segmento no conecta directamente con
                # el ultimo edge ya recorrido, se recalcula el segmento desde el
                # ultimo edge fisico hasta el mismo destino operacional.
                if route_total and route_edges:
                    edge_anterior = normalizar_id(route_total[-1])
                    primer_edge_nuevo = normalizar_id(route_edges[0])

                    if edge_anterior != primer_edge_nuevo:
                        puente, estado_puente = memoria.net.shortest_edges(
                            edge_anterior,
                            primer_edge_nuevo,
                        )
                        puente = limpiar_edges(puente or [])

                        conexion_directa = (
                            len(puente) == 2
                            and puente[0] == edge_anterior
                            and puente[-1] == primer_edge_nuevo
                        )

                        if not conexion_directa:
                            ruta_recalculada, estado_recalculo = memoria.net.shortest_edges(
                                edge_anterior,
                                destino,
                            )
                            ruta_recalculada = limpiar_edges(
                                ruta_recalculada or []
                            )

                            if not ruta_recalculada:
                                raise ValueError(
                                    f"{camion}: RUTA_DESCONECTADA_SIN_REPARACION "
                                    f"{edge_anterior}->{primer_edge_nuevo}; "
                                    f"destino={destino}; "
                                    f"estadoPuente={estado_puente}; "
                                    f"estadoRecalculo={estado_recalculo}"
                                )

                            advertencias.append(
                                f"{camion}: ruta de {operacion} reparada por continuidad fisica: "
                                f"{edge_anterior}->{primer_edge_nuevo} no era conexion directa. "
                                f"Se recalculo desde {edge_anterior} hasta {destino}."
                            )
                            origen = edge_anterior
                            route_edges = ruta_recalculada
                            estado = f"RECALCULADA_CONTINUIDAD:{estado_recalculo}"

                route_edges_sumo = unir_segmento(route_total, route_edges)

                if not route_edges_sumo and route_total:
                    advertencias.append(
                        f"{camion}: {operacion} {origen}->{destino} quedo sin edges nuevos "
                        "al unir segmentos; se omite para evitar edge duplicado consecutivo."
                    )
                    ubicacion_actual = destino
                    continue

                velocidad_ref = velocidad_camion(ev, operacion)
                distancia_segmento_m = distancia_edges(memoria.net, route_edges_sumo)

                # Se usa exactamente la velocidad nominal recibida desde server.py.
                # No se compensa, no se multiplica y no se aplica ningún margen extra.
                max_speed = min(
                    VELOCIDAD_MAXIMA_SEGURA_MS,
                    max(0.1, velocidad_ref),
                )

                duracion_objetivo_s = convertir_int(ev.get("duracion", 0)) / 1000.0
                tiempo_estimado_s = estimar_tiempo(
                    distancia_segmento_m,
                    max_speed,
                )
                objetivo_alcanzable = (
                    duracion_objetivo_s > 0
                    and tiempo_estimado_s <= duracion_objetivo_s + 0.05
                )

                # El vType usa la misma velocidad nominal, sin margen adicional.
                max_speed_truck = max(max_speed_truck, max_speed)
                if not empty_speed_ref and ev.get("velocidadVacio", ""):
                    empty_speed_ref = str(ev.get("velocidadVacio", ""))
                if not loaded_speed_ref and ev.get("velocidadCargado", ""):
                    loaded_speed_ref = str(ev.get("velocidadCargado", ""))

                # Concatenamos todos los edges en una unica ruta del camion.
                # Se usa route_edges_sumo para evitar duplicar el edge de empalme entre segmentos.
                route_total.extend(route_edges_sumo)

                stop = buscar_stop_posterior(
                    eventos,
                    idx,
                    operacion,
                    todos_eventos=memoria.eventos,
                    camion=camion
                )
                if stop:
                    last_edge = route_edges[-1]
                    last_lane = memoria.net.edge_to_lane.get(last_edge, f"{last_edge}_0")
                    stop["lane"] = last_lane
                    stop["edge"] = last_edge
                    # CARGA/DESCARGA usan un parking operacional.
                    # La espera entre ciclos se separa físicamente en PA_WAIT.
                    stop["parkingArea"] = parking_operacional_id(
                        stop.get("ubicacion", ""),
                        last_lane,
                    )
                    stop["after_operacion"] = operacion
                    stop["segmento_destino"] = destino
                    stops.append(stop)

                    espera = buscar_espera(eventos, idx, stop, camion=camion)
                    if espera:
                        espera["lane"] = last_lane
                        espera["edge"] = last_edge
                        espera["parkingArea"] = parking_espera_id(
                            espera.get("ubicacion", stop.get("ubicacion", "")),
                            last_lane,
                        )
                        espera["segmento_destino"] = destino
                        stops.append(espera)
                        advertencias.append(
                            f"{camion}: ESPERA_INACTIVA despues de DESCARGA en {espera.get('ubicacion','')} "
                            f"duracion={espera.get('duration',0):.3f}s"
                        )

                segmentos.append({
                    "operacion": operacion,
                    "origenRuta": origen,
                    "destinoRuta": destino,
                    "depart_ms": convertir_int(ev.get("horaInicio", 0)),
                    "arrival_plan_ms": convertir_int(ev.get("horaFin", 0)),
                    "ubicacion": ubicacion_evento,
                    "route_edges_count": len(route_edges_sumo),
                    "estado_ruta": estado,
                    "speed_ms": max_speed,
                    "speed_ref_ms": velocidad_ref,
                    "distancia_m": distancia_segmento_m,
                    "duracion_plan_ms": convertir_int(ev.get("duracion", 0)),
                    "objetivo_alcanzable": objetivo_alcanzable,
                })

                ubicacion_actual = destino

            elif es_carga_descarga(operacion):
                ubicacion_actual = ubicacion_evento or ubicacion_actual
            else:
                if ubicacion_evento:
                    ubicacion_actual = ubicacion_evento

        if not route_total:
            advertencias.append(f"{camion}: no se genero ruta total. Vehiculo omitido.")
            continue

        # Después de la última DESCARGA el camión NO permanece en el parking
        # operacional. Se mueve al PA_WAIT de la misma zona para conservarlo vivo
        # sin ocupar el punto de descarga ni quedar detenido sobre la vía.
        parking_espera = ""
        if stops and normalizar_id(stops[-1].get("operacion_stop", "")).upper() == "DESCARGA":
            hold = dict(stops[-1])
            parking_espera = parking_espera_id(
                hold.get("ubicacion", ""),
                hold.get("lane", ""),
            )
            hold["parkingArea"] = parking_espera
            hold["operacion_stop"] = "ESPERA_DISPONIBLE_FIN_SCHEDULE"
            hold["duration"] = ESPERA_DISPONIBLE_S
            hold["duration_ms"] = int(ESPERA_DISPONIBLE_S * 1000.0)
            hold["after_operacion"] = "DESCARGA"
            stops.append(hold)

        type_id = f"haul_truck_{id_xml_seguro(camion)}"
        vtypes[type_id] = {
            "id": type_id,
            "maxSpeed": max_speed_truck,
            "operacion": "MIXTA_VACIO_CARGADO",
            "camion": camion,
        }

        vehicles.append({
            "id": camion,
            "camion": camion,
            "depart": ms_a_seg(first_depart_ms or 0),
            "depart_ms": first_depart_ms or 0,
            "type": type_id,
            "maxSpeed": max_speed_truck,
            "route_edges": route_total,
            "stops": stops,
            "segmentos": segmentos,
            "velocidadVacioJADE": empty_speed_ref,
            "velocidadCargadoJADE": loaded_speed_ref,
            "estado_inicial": "PROGRAMADO",
            "parking_espera": parking_espera,
            "ubicacion_inicial": ubicacion_inicial or camion,
            "edge_inicial": route_total[0] if route_total else "",
            "lane_inicial": memoria.net.edge_to_lane.get(route_total[0], "") if route_total else "",
            "pos_inicial": 0.0,
        })

    # Camiones de trucks.xml que no obtuvieron ninguna asignación inicial.
    # Se crean igualmente en SUMO y se estacionan desde t=0.
    for info in cargar_camiones(memoria):
        camion = normalizar_id(info.get("camion", ""))
        if not camion or camion in camiones_con_eventos:
            continue
        edge = normalizar_id(info.get("edge", ""))
        lane = normalizar_id(info.get("lane", ""))
        if not edge or not lane:
            advertencias.append(f"{camion}: sin scheduling y sin edge/lane inicial; vehiculo omitido.")
            continue
        parking_id = f"PA_IDLE_INIT_{id_xml_seguro(lane)}"
        vacio_ms = min(VELOCIDAD_MAXIMA_SEGURA_MS, parsear_velocidad_ms(info.get("vacio", ""), VELOCIDAD_DEF_VACIO_KMH))
        cargado_ms = min(VELOCIDAD_MAXIMA_SEGURA_MS, parsear_velocidad_ms(info.get("cargado", ""), VELOCIDAD_DEF_CARGADO_KMH))
        max_speed = max(0.1, vacio_ms, cargado_ms)
        type_id = f"haul_truck_{id_xml_seguro(camion)}"
        vtypes[type_id] = {
            "id": type_id, "maxSpeed": max_speed,
            "operacion": "DISPONIBLE_SIN_SCHEDULE", "camion": camion,
        }
        vehicles.append({
            "id": camion, "camion": camion, "depart": 0.0, "depart_ms": 0,
            "type": type_id, "maxSpeed": max_speed, "route_edges": [edge],
            "stops": [{
                "operacion_stop": "ESPERA_DISPONIBLE_SIN_SCHEDULE",
                "duration": ESPERA_DISPONIBLE_S,
                "duration_ms": int(ESPERA_DISPONIBLE_S * 1000.0),
                "ubicacion": camion, "lane": lane, "edge": edge,
                "parkingArea": parking_id, "parking_anchor": float(info.get("pos", 0.0) or 0.0),
            }],
            "segmentos": [],
            "velocidadVacioJADE": str(info.get("vacio", "")),
            "velocidadCargadoJADE": str(info.get("cargado", "")),
            "estado_inicial": "DISPONIBLE_SIN_SCHEDULE",
            "parking_espera": parking_id, "ubicacion_inicial": camion,
            "edge_inicial": edge, "lane_inicial": lane, "pos_inicial": float(info.get("pos", 0.0) or 0.0),
        })
        advertencias.append(f"{camion}: sin scheduling inicial; se mantiene disponible en {parking_id}.")

    vehicles.sort(key=lambda t: (t["depart"], t["id"]))
    return vehicles, advertencias, vtypes


def generar_rou_xml_directo(vehicles: List[Dict[str, Any]], output_rou: str, vtypes: Dict[str, Dict[str, Any]]) -> None:
    root = ET.Element("routes")
    root.set("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance")
    root.set("xsi:noNamespaceSchemaLocation", "http://sumo.dlr.de/xsd/routes_file.xsd")

    for type_id, info in sorted(vtypes.items()):
        ET.SubElement(root, "vType", {
            "id": type_id,
            "length": "12.00",
            "maxSpeed": f"{float(info.get('maxSpeed', 10.0)):.2f}",
            "vClass": "truck",
            "width": "3.00",
            "accel": "2.00",
            "decel": "4.50",
            "tau": "1.20",
            "minGap": "4.00",
            "sigma": "0",
            "speedDev": "0",
            "color": "orange",

            # Caso SIN ADELANTAMIENTO:
            # Sin sublane/lateral-resolution y sin uso del carril contrario.
            # Estos parametros son candados para evitar cambios por ganancia de velocidad.
            "laneChangeModel": "LC2013",
            "lcSpeedGain": "0.00",
            "lcStrategic": "0.00",
            "lcCooperative": "0.00",
            "lcKeepRight": "0.00",
            "lcOpposite": "0.00",
        })

    for veh_data in vehicles:
        veh_attrs = {
            "id": veh_data["id"],
            "type": veh_data["type"],
            "depart": f"{float(veh_data['depart']):.2f}",
            "departSpeed": "0",
        }
        if str(veh_data.get("estado_inicial", "")).upper() == "DISPONIBLE_SIN_SCHEDULE":
            lane_ini = str(veh_data.get("lane_inicial", "") or "")
            lane_idx = lane_ini.rsplit("_", 1)[1] if "_" in lane_ini else "0"
            veh_attrs["departLane"] = lane_idx if lane_idx.isdigit() else "0"
            veh_attrs["departPos"] = f"{max(0.0, float(veh_data.get('pos_inicial', 0.0) or 0.0)):.2f}"
        veh = ET.SubElement(root, "vehicle", veh_attrs)

        ET.SubElement(veh, "route", {"edges": " ".join(veh_data["route_edges"])})

        for stop in veh_data.get("stops", []):
            parking = normalizar_id(stop.get("parkingArea", ""))
            lane = normalizar_id(stop.get("lane", ""))
            dur_ms = convertir_int(stop.get("duration_ms", 0))
            dur = (dur_ms / 1000.0) if dur_ms > 0 else float(stop.get("duration", 0))
            if dur > 0:
                attrs = {"duration": f"{dur:.3f}"}
                if parking:
                    attrs["parkingArea"] = parking
                elif lane:
                    attrs["lane"] = lane
                ET.SubElement(veh, "stop", attrs)

        params = {
            "camion": veh_data.get("camion", ""),
            "modo_generacion": "UN_VEHICULO_POR_CAMION",
            "depart_ms": str(veh_data.get("depart_ms", "")),
            "maxSpeed_ms": f"{float(veh_data.get('maxSpeed', 0)):.2f}",
            "velocidadVacioJADE": str(veh_data.get("velocidadVacioJADE", "")),
            "velocidadCargadoJADE": str(veh_data.get("velocidadCargadoJADE", "")),
            "route_edges_count": str(len(veh_data.get("route_edges", []))),
            "segmentos_count": str(len(veh_data.get("segmentos", []))),
            "stops_count": str(len(veh_data.get("stops", []))),
            "estado_inicial": str(veh_data.get("estado_inicial", "PROGRAMADO")),
            "parking_espera": str(veh_data.get("parking_espera", "")),
            "ubicacion_inicial": str(veh_data.get("ubicacion_inicial", veh_data.get("camion", ""))),
            "edge_inicial": str(veh_data.get("edge_inicial", "")),
            "lane_inicial": str(veh_data.get("lane_inicial", "")),
            "pos_inicial": str(veh_data.get("pos_inicial", 0.0)),
        }
        for k, v in params.items():
            ET.SubElement(veh, "param", {"key": k, "value": v})

        for i, seg in enumerate(veh_data.get("segmentos", []), start=1):
            ET.SubElement(veh, "param", {
                "key": f"seg_{i:03d}",
                "value": (
                    f"{seg.get('operacion','')}"
                    f"|{seg.get('origenRuta','')}->{seg.get('destinoRuta','')}"
                    f"|depart_ms={seg.get('depart_ms','')}"
                    f"|arrival_ms={seg.get('arrival_plan_ms','')}"
                    f"|edges={seg.get('route_edges_count','')}"
                    f"|speed_ms={float(seg.get('speed_ms', 0)):.6f}"
                    f"|speed_ref_ms={float(seg.get('speed_ref_ms', 0)):.6f}"
                    f"|dist_m={float(seg.get('distancia_m', 0)):.3f}"
                    f"|dur_ms={seg.get('duracion_plan_ms','')}"
                    f"|reachable={1 if seg.get('objetivo_alcanzable', False) else 0}"
                )
            })

    indent_xml(root)
    os.makedirs(os.path.dirname(output_rou) or ".", exist_ok=True)
    ET.ElementTree(root).write(output_rou, encoding="utf-8", xml_declaration=True)


def generar_trips(trips: List[Dict[str, Any]], output_trips: str, vtypes: Dict[str, Dict[str, Any]]) -> None:
    # Se mantiene este archivo porque lo usabas en tu flujo.
    # Es una copia equivalente al rou con rutas explicitas.
    generar_rou_xml_directo(trips, output_trips, vtypes)


def generar_object_generado(trips: List[Dict[str, Any]], output_add: str, lane_lengths: Dict[str, float]) -> Dict[str, Any]:
    """
    Genera parkingArea separando:
      - PA_AUTO_* : operación (CARGA/DESCARGA).
      - PA_WAIT_* : espera entre ciclos / espera final.
      - PA_SAFE_* : detención segura temporal por avería de camión.

    Los parking operacionales dejan de ubicarse a 1 m de la junction.
    Las palas (CARGA) tienen capacidad 1. Los PA_WAIT mantienen capacidad
    amplia porque los vehículos estacionados quedan fuera del flujo vial.
    """
    parking_por_id = {}

    def registrar_parking(parking_id: str, lane: str, ubicacion: str, operacion: str, anchor: float = -1.0):
        parking_id = normalizar_id(parking_id)
        lane = normalizar_id(lane)
        if not parking_id or not lane:
            return
        parking_por_id[parking_id] = {
            "id": parking_id,
            "lane": lane,
            "ubicacion": normalizar_id(ubicacion),
            "operacion": normalizar_id(operacion).upper(),
            "anchor": float(anchor or -1.0),
        }

    # Precrear TODAS las palas del escenario, incluso si no aparecieron en el
    # scheduling inicial. El rescheduling puede asignar después una pala nueva
    # (por ejemplo PA10) y SUMO exige que su parkingArea ya exista desde el arranque.
    try:
        input_dir_obj = PROJECT_ROOT if ESCENARIO == "small" else os.path.join(PROJECT_ROOT, ESCENARIO)
        objects_path_obj = os.path.join(input_dir_obj, "objects.xml")
        if os.path.exists(objects_path_obj):
            root_obj = ET.parse(objects_path_obj).getroot()
            for obj in root_obj.findall(".//object"):
                tipo_obj = str(obj.get("type", "") or "").strip().lower()
                obj_id = normalizar_id(obj.get("id", ""))
                if tipo_obj != "shovel" or not obj_id:
                    continue
                loc = obj.find("location")
                if loc is None:
                    continue
                lane_obj = normalizar_id(loc.get("lane", ""))
                if not lane_obj:
                    continue
                registrar_parking(
                    parking_operacional_id(obj_id, lane_obj),
                    lane_obj,
                    obj_id,
                    "CARGA",
                )
    except Exception as e:
        print(f"[server] ADVERTENCIA: no se pudieron precrear todas las palas: {e}")

    # Precrear las zonas seguras de avería. No se agregan como stops normales
    # de los camiones; únicamente existen en el additional y dynamic_events.py
    # inserta el stop temporal cuando E3 realmente se activa.
    for safe_nombre, safe_lane in PA_SAFE_LANES:
        if safe_lane not in lane_lengths:
            print(
                f"[server] ADVERTENCIA: PA_SAFE omitido porque la lane no existe "
                f"en la red: {safe_nombre} -> {safe_lane}"
            )
            continue
        registrar_parking(
            parking_safe_id(safe_nombre),
            safe_lane,
            safe_nombre,
            "AVERIA_SEGURA",
        )

    for trip in trips:
        for stop in trip.get("stops", []):
            parking_id = normalizar_id(stop.get("parkingArea", ""))
            lane = normalizar_id(stop.get("lane", ""))
            ubicacion = normalizar_id(stop.get("ubicacion", ""))
            operacion = normalizar_id(stop.get("operacion_stop", "")).upper()
            anchor = float(stop.get("parking_anchor", -1.0) or -1.0)

            if parking_id and lane:
                registrar_parking(parking_id, lane, ubicacion, operacion, anchor)

            # Toda zona de DESCARGA recibe también un PA_WAIT, aunque el cronograma
            # inicial no tenga un gap allí. Así el runner puede conservar camiones
            # después de ciclos de rescheduling sin usar un stop sobre la calzada.
            if operacion == "DESCARGA" and lane:
                registrar_parking(
                    parking_espera_id(ubicacion, lane),
                    lane,
                    ubicacion,
                    "ESPERA_DISPONIBLE",
                )

    def geometria_lane(largo: float):
        """Devuelve (op_start, op_end, wait_start, wait_end)."""
        largo = max(5.0, float(largo or 0.0))

        # Reserva espacio antes de la junction. En lanes cortas se usa un margen
        # menor para no invalidar la geometría; en lanes largas se dejan 30 m.
        if largo >= 100.0:
            margen_fin = 30.0
        elif largo >= 60.0:
            margen_fin = 15.0
        else:
            margen_fin = min(5.0, max(1.0, largo * 0.12))

        inicio_util = 1.0
        fin_util = max(inicio_util + 2.0, largo - margen_fin)
        espacio = max(2.0, fin_util - inicio_util)

        # La operación usa 15 m cuando existe espacio suficiente. Un camión mide
        # 12 m, por lo que se intenta mantener al menos 12.5 m.
        if espacio >= 30.0:
            largo_op = 15.0
        elif espacio >= 15.0:
            largo_op = 12.5
        else:
            largo_op = max(4.0, espacio * 0.45)

        op_start = inicio_util
        op_end = min(fin_util, op_start + largo_op)

        # El patio de espera comienza después del punto operacional.
        gap = min(10.0, max(2.0, espacio * 0.05))
        wait_start = min(fin_util, op_end + gap)
        wait_end = fin_util

        # Si la lane es muy corta, reduce el gap antes de superponer áreas.
        if wait_end - wait_start < 4.0:
            gap = 1.0
            wait_start = min(fin_util, op_end + gap)

        return op_start, op_end, wait_start, wait_end

    def geometria_safe(largo: float):
        """Ubica un PA_SAFE en la zona media de la lane, lejos de las junctions."""
        largo = max(5.0, float(largo or 0.0))
        margen = min(30.0, max(5.0, largo * 0.10))
        inicio_util = margen
        fin_util = max(inicio_util + 1.0, largo - margen)
        centro = (inicio_util + fin_util) / 2.0

        # 30 m permiten alojar holgadamente un haul truck de 12 m.
        largo_safe = min(30.0, max(12.5, fin_util - inicio_util))
        start = max(inicio_util, centro - largo_safe / 2.0)
        end = min(fin_util, start + largo_safe)

        if end - start < 12.5 and fin_util - inicio_util >= 12.5:
            start = max(inicio_util, fin_util - 12.5)
            end = fin_util
        return start, end

    root = ET.Element("additional")
    root.set("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance")
    root.set("xsi:noNamespaceSchemaLocation", "http://sumo.dlr.de/xsd/additional_file.xsd")

    for parking_id, info in sorted(parking_por_id.items()):
        lane = info["lane"]
        largo = float(lane_lengths.get(lane, 100.0) or 100.0)
        operacion = str(info.get("operacion", "")).upper()

        if operacion == "ESPERA_DISPONIBLE_SIN_SCHEDULE":
            # Camiones que nacen sin scheduling: conserva su posición inicial.
            anchor = max(0.0, float(info.get("anchor", 0.0) or 0.0))
            start = min(max(0.0, anchor), max(0.0, largo - 14.0))
            end = min(max(start + 12.5, start + 1.0), max(start + 1.0, largo - 0.5))
            capacidad = 50
        else:
            op_start, op_end, wait_start, wait_end = geometria_lane(largo)
            es_safe = parking_id.startswith("PA_SAFE_") or operacion == "AVERIA_SEGURA"
            es_wait = parking_id.startswith("PA_WAIT_") or operacion.startswith("ESPERA_")

            if es_safe:
                start, end = geometria_safe(largo)
                capacidad = 1
            elif es_wait:
                start, end = wait_start, wait_end
                capacidad = 50
            else:
                start, end = op_start, op_end
                # Una pala solo puede atender un camión a la vez.
                # Para DESCARGA se conserva capacidad amplia para no imponer
                # una restricción que JADE no está modelando actualmente.
                capacidad = 1 if operacion == "CARGA" else 50

        # Protección final ante lanes extremadamente cortas.
        if end <= start:
            end = min(largo - 0.1, start + 1.0)
        if end <= start:
            start = max(0.0, largo - 1.1)
            end = max(start + 0.1, largo - 0.1)

        parking_el = ET.SubElement(root, "parkingArea", {
            "id": parking_id,
            "lane": lane,
            "startPos": f"{start:.2f}",
            "endPos": f"{end:.2f}",
            "roadsideCapacity": str(capacidad),
            "onRoad": "false",
            "friendlyPos": "true",
        })
        ET.SubElement(parking_el, "param", {"key": "ubicacion", "value": info.get("ubicacion", "")})
        ET.SubElement(parking_el, "param", {"key": "operacionReferencia", "value": operacion})

    indent_xml(root)
    os.makedirs(os.path.dirname(output_add) or ".", exist_ok=True)
    ET.ElementTree(root).write(output_add, encoding="utf-8", xml_declaration=True)
    print(f"[server] ParkingAreas generados: {len(parking_por_id)} -> {output_add}")
    return {"parking_areas": len(parking_por_id), "objects_add": output_add}


def generar_sumocfg(path: str, net_path: str) -> None:
    root = ET.Element("configuration")

    net_rel = os.path.relpath(net_path, os.path.dirname(path) or ".").replace("\\", "/")

    input_el = ET.SubElement(root, "input")
    ET.SubElement(input_el, "net-file", {"value": net_rel})
    ET.SubElement(input_el, "route-files", {"value": os.path.basename(ROU_XML)})
    ET.SubElement(input_el, "additional-files", {"value": os.path.basename(OBJECTS_GENERADO_XML)})

    output_el = ET.SubElement(root, "output")
    ET.SubElement(output_el, "tripinfo-output", {"value": "tripinfo.xml"})
    ET.SubElement(output_el, "tripinfo-output.write-unfinished", {"value": "true"})
    ET.SubElement(output_el, "summary-output", {"value": "summary.xml"})
    ET.SubElement(output_el, "vehroute-output", {"value": "vehroute.xml"})
    ET.SubElement(output_el, "emission-output", {"value": "emissions.xml"})

    time_el = ET.SubElement(root, "time")
    ET.SubElement(time_el, "begin", {"value": "0"})
    ET.SubElement(time_el, "step-length", {"value": "1.0"})

    indent_xml(root)
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    print(f"[server] SUMOCFG generado: {path}")



def generar_runner(path: str) -> None:
    contenido = r'''# -*- coding: utf-8 -*-
# -*- coding: utf-8 -*-
import argparse
import csv
import json
import os
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = BASE_DIR.parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from dynamic_events import DynamicEventManager, EVENTS_CSV_DEFAULT
from routing_service import NetSumo, id_xml_seguro
from metricas import GestorMetricas, calcular_error_relativo

ESCENARIO = "small"
SUMOCFG_DEFAULT = BASE_DIR / "small_generado.sumocfg"
ROU_DEFAULT = BASE_DIR / "mntrucks_generado.rou.xml"

# XML originales utilizados por JADE y por el servicio de rutas.
# La red es común; objects/trucks dependen del escenario.
INPUT_DIR = PROJECT_ROOT if ESCENARIO == "small" else PROJECT_ROOT / ESCENARIO
OBJECTS_INPUT = INPUT_DIR / "objects.xml"
TRUCKS_INPUT = INPUT_DIR / "trucks.xml"
if not TRUCKS_INPUT.exists():
    TRUCKS_INPUT = INPUT_DIR / "trucks_enriquecido.xml"
REPORTE_SEGMENTOS_DEFAULT = BASE_DIR / "reporte_segmentos_sumo.csv"
EVENTOS_DINAMICOS_DEFAULT = BASE_DIR / "eventos_dinamicos_sumo.csv"
REPORTE_DERRUMBE_DEFAULT = BASE_DIR / "reporte_derrumbe_sumo.csv"
EVENTOS_CONFIG_DEFAULT = EVENTS_CSV_DEFAULT
RESCHEDULING_DEFAULT = BASE_DIR / "rescheduling.csv"
SNAPSHOT_FLOTA_DEFAULT = BASE_DIR / "snapshot_flota_rescheduling.csv"
SNAPSHOT_FLOTA_JSON_DEFAULT = BASE_DIR / "snapshot_flota_rescheduling.json"
SNAPSHOT_MONITOREO_DEFAULT = BASE_DIR / "snapshot_flota_monitoreo.csv"
SNAPSHOT_MONITOREO_JSON_DEFAULT = BASE_DIR / "snapshot_flota_monitoreo_ultimo.json"
SNAPSHOT_MONITOREO_INTERVAL_S = 10.0
TOL_JADE = 0.10  # 10% del cronograma global JADE aplicado al retraso total de la flota
SERVER_ZMQ_ENDPOINT_DEFAULT = "tcp://localhost:5555"
SERVER_ZMQ_TIMEOUT_MS = 500

# Horizonte operacional del turno. Los camiones disponibles permanecen en SUMO
# hasta este instante si nunca se activa el rescheduling.
HORIZONTE_TURNO_S = 6.0 * 60.0 * 60.0
ESTADO_SIN_SCHEDULE = "DISPONIBLE_SIN_SCHEDULE"
ESTADO_FIN_SCHEDULE = "DISPONIBLE_FIN_SCHEDULE"
ESTADOS_DISPONIBLES = {ESTADO_SIN_SCHEDULE, ESTADO_FIN_SCHEDULE}

# Aproximacion operacional al destino del segmento.
# Se aplica solo cuando el camion se acerca al final de SU segmento actual
# (es decir, a su proximo stop/pala/botadero), no por pasar cerca de cualquier parkingArea.
APROXIMACION_ACTIVA_DEFAULT = True
APROX_DIST_1_M = 120.0
APROX_DIST_2_M = 70.0
APROX_DIST_3_M = 35.0
APROX_SPEED_1_MS = 4.0
APROX_SPEED_2_MS = 3.0
APROX_SPEED_3_MS = 2.0


def parse_velocidad_ms(valor, default_kmh=10.0):
    txt = str(valor or "").strip().lower().replace(",", ".")
    if not txt:
        return max(0.1, default_kmh / 3.6)
    try:
        num = float(re.sub(r"[^0-9.\-]", "", txt))
        if num <= 0:
            return max(0.1, default_kmh / 3.6)
        if "m/s" in txt or "ms" in txt:
            return max(0.1, num)
        return max(0.1, num / 3.6)
    except Exception:
        return max(0.1, default_kmh / 3.6)


def parse_num(valor, default=0.0):
    try:
        return float(str(valor).replace(",", "."))
    except Exception:
        return default


def cargar_plan_segmentos(rou_path: Path):
    root = ET.parse(rou_path).getroot()
    planes = {}

    for veh in root.findall("vehicle"):
        veh_id = (veh.get("id") or "").strip()
        if not veh_id:
            continue

        route = veh.find("route")
        if route is None:
            continue

        edges = [x.strip() for x in (route.get("edges") or "").split() if x.strip()]
        if not edges:
            continue

        params = {p.get("key", ""): p.get("value", "") for p in veh.findall("param")}
        stops_xml = veh.findall("stop")
        stop_durations_s = []
        stops_plan = []
        for st in stops_xml:
            dur = parse_num(st.get("duration", "0"), 0.0)
            if dur > 0:
                stop_durations_s.append(float(dur))
            stops_plan.append({
                "duration_s": float(max(0.0, dur)),
                "parkingArea": (st.get("parkingArea") or "").strip(),
                "lane": (st.get("lane") or "").strip(),
                "edge": (st.get("edge") or "").strip(),
                "endPos": parse_num(st.get("endPos", "-1"), -1.0),
            })
        vacio_ms = parse_velocidad_ms(params.get("velocidadVacioJADE", ""), 24.0)
        cargado_ms = parse_velocidad_ms(params.get("velocidadCargadoJADE", ""), 18.0)

        index_to_speed = {}
        index_to_operacion = {}
        index_to_segment = {}
        segmentos = []
        pos = 0

        for nseg, sk in enumerate(sorted(k for k in params if k.startswith("seg_")), start=1):
            partes = params.get(sk, "").split("|")
            if not partes:
                continue

            operacion = partes[0].strip().upper()
            ruta_txt = partes[1].strip() if len(partes) > 1 else ""
            edges_count = 0
            speed_segmento = None
            speed_ref_segmento = None
            reachable = True
            depart_ms = 0
            arrival_ms = 0
            dist_m = 0.0
            dur_ms = 0

            for parte in partes:
                parte = parte.strip()
                if parte.startswith("edges="):
                    try:
                        edges_count = int(float(parte.split("=", 1)[1]))
                    except Exception:
                        edges_count = 0
                elif parte.startswith("speed_ms="):
                    speed_segmento = parse_num(parte.split("=", 1)[1], None)
                elif parte.startswith("speed_ref_ms="):
                    speed_ref_segmento = parse_num(parte.split("=", 1)[1], None)
                elif parte.startswith("reachable="):
                    reachable = int(parse_num(parte.split("=", 1)[1], 1)) == 1
                elif parte.startswith("depart_ms="):
                    depart_ms = int(parse_num(parte.split("=", 1)[1], 0))
                elif parte.startswith("arrival_ms="):
                    arrival_ms = int(parse_num(parte.split("=", 1)[1], 0))
                elif parte.startswith("dist_m="):
                    dist_m = parse_num(parte.split("=", 1)[1], 0.0)
                elif parte.startswith("dur_ms="):
                    dur_ms = int(parse_num(parte.split("=", 1)[1], 0))

            if edges_count <= 0:
                continue

            if speed_segmento is not None and speed_segmento > 0:
                speed = float(speed_segmento)
            else:
                speed = cargado_ms if operacion == "VIAJE_CARGADO" else vacio_ms

            ini = pos
            fin = min(pos + edges_count, len(edges))
            if fin <= ini:
                continue

            seg = {
                "seg_num": nseg,
                "param_key": sk,
                "operacion": operacion,
                "ruta": ruta_txt,
                "start_idx": ini,
                "end_idx": fin - 1,
                "edges_count": fin - ini,
                "speed_ms": speed,
                "speed_ref_ms": (
                    float(speed_ref_segmento)
                    if speed_ref_segmento is not None and speed_ref_segmento > 0
                    else (cargado_ms if operacion == "VIAJE_CARGADO" else vacio_ms)
                ),
                "reachable": reachable,
                "depart_ms": depart_ms,
                "arrival_ms": arrival_ms,
                "dur_ms": dur_ms if dur_ms > 0 else max(0, arrival_ms - depart_ms),
                "dist_m": dist_m,
                "edge_inicio": edges[ini],
                "edge_fin": edges[fin - 1],
            }
            segmentos.append(seg)

            for i in range(ini, fin):
                index_to_speed[i] = speed
                index_to_operacion[i] = operacion
                index_to_segment[i] = len(segmentos) - 1
            pos = fin

        # Asigna duracion del stop operacional posterior a cada segmento.
        # En el .rou.xml los stops quedan en orden: CARGA despues de VIAJE_VACIO
        # y DESCARGA despues de VIAJE_CARGADO. Puede existir una ESPERA_INACTIVA
        # extra despues de una descarga; esa espera se salta para no confundirla con
        # la carga del siguiente ciclo.
        stop_cursor = 0
        for idx_seg, seg_tmp in enumerate(segmentos):
            dur_stop = 0.0
            if stop_cursor < len(stop_durations_s):
                dur_stop = float(stop_durations_s[stop_cursor])
                stop_cursor += 1
            seg_tmp["stop_after_duration_s"] = dur_stop

            # Si luego de un VIAJE_CARGADO viene otro VIAJE_VACIO y hay una espera
            # inactiva entre ambos, se consume ese stop extra para mantener alineados
            # los stops operacionales de los ciclos siguientes.
            if seg_tmp.get("operacion") == "VIAJE_CARGADO" and idx_seg + 1 < len(segmentos):
                prox = segmentos[idx_seg + 1]
                gap_s = max(0.0, (float(prox.get("depart_ms", 0)) - float(seg_tmp.get("arrival_ms", 0))) / 1000.0)
                if stop_cursor < len(stop_durations_s) and gap_s > dur_stop + 1.0:
                    posible_espera = float(stop_durations_s[stop_cursor])
                    if abs(posible_espera - max(0.0, gap_s - dur_stop)) <= max(2.0, 0.10 * max(1.0, gap_s)):
                        seg_tmp["wait_after_duration_s"] = posible_espera
                        stop_cursor += 1

        for i in range(len(edges)):
            index_to_speed.setdefault(i, vacio_ms)
            index_to_operacion.setdefault(i, "VIAJE_VACIO")

        planes[veh_id] = {
            "vacio_ms": vacio_ms,
            "cargado_ms": cargado_ms,
            "index_to_speed": index_to_speed,
            "index_to_operacion": index_to_operacion,
            "index_to_segment": index_to_segment,
            "segmentos": segmentos,
            "edges": edges,
            "route_edges_count": len(edges),
            "stops": stops_plan,
            "estado_inicial": str(params.get("estado_inicial", "PROGRAMADO") or "PROGRAMADO").upper(),
            "parking_espera": str(params.get("parking_espera", "") or ""),
            "ubicacion_inicial": str(params.get("ubicacion_inicial", veh_id) or veh_id),
            "edge_inicial": str(params.get("edge_inicial", edges[0] if edges else "") or ""),
            "lane_inicial": str(params.get("lane_inicial", "") or ""),
            "pos_inicial": parse_num(params.get("pos_inicial", "0"), 0.0),
        }

    return planes



def calcular_total_jade_s(planes):
    """Duracion global del cronograma JADE: primer inicio hasta ultimo fin."""
    inicios = []
    fines = []
    for plan in planes.values():
        for seg in plan.get("segmentos", []):
            ini = int(seg.get("depart_ms", 0) or 0)
            fin = int(seg.get("arrival_ms", 0) or 0)
            if fin > ini:
                inicios.append(ini)
                fines.append(fin)
    if not inicios or not fines:
        return 0.0
    return max(0.0, (max(fines) - min(inicios)) / 1000.0)

def esta_en_stop_o_parking(traci, veh_id):
    try:
        if traci.vehicle.isStoppedParking(veh_id):
            return True
    except Exception:
        pass
    try:
        return bool(traci.vehicle.getStopState(veh_id))
    except Exception:
        return False


def distancia_segmento(traci, veh_id, plan, seg, route_index, lane_id):
    """
    Distancia aproximada desde la posicion actual hasta el final del segmento actual.

    Se usa para reducir velocidad SOLO al aproximarse al destino operacional
    del segmento actual. No reduce por pasar cerca de otros parkingArea.
    """
    try:
        route_index = int(route_index)
        end_idx = int(seg.get("end_idx", route_index))
        if route_index > end_idx:
            return 0.0

        restante = 0.0
        try:
            pos = float(traci.vehicle.getLanePosition(veh_id))
        except Exception:
            pos = 0.0

        try:
            lane_len = float(traci.lane.getLength(lane_id)) if lane_id else 0.0
        except Exception:
            lane_len = 0.0

        if lane_len > 0:
            restante += max(0.0, lane_len - pos)

        edges = plan.get("edges", [])
        for idx in range(route_index + 1, min(end_idx + 1, len(edges))):
            edge = edges[idx]
            # En esta red normalmente se usa lane _0. Si no existe, se ignora.
            # Esto es solo una aproximacion para anticipar frenado, no cambia la ruta.
            try:
                restante += float(traci.lane.getLength(edge + "_0"))
            except Exception:
                pass

        return restante
    except Exception:
        return 999999.0


def velocidad_aprox(speed_base, distancia_restante, args):
    """Limita velocidad de forma gradual cerca del destino del segmento."""
    if args.no_approach_slowdown:
        return speed_base, False, distancia_restante

    v = float(speed_base)
    aplicado = False

    # Orden: primero la zona amplia, luego zonas mas cercanas con limites menores.
    if distancia_restante <= float(args.approach_d1):
        v = min(v, float(args.approach_v1))
        aplicado = True
    if distancia_restante <= float(args.approach_d2):
        v = min(v, float(args.approach_v2))
        aplicado = True
    if distancia_restante <= float(args.approach_d3):
        v = min(v, float(args.approach_v3))
        aplicado = True

    return max(0.1, v), aplicado, distancia_restante


def cargar_eventos(path: Path):
    """Carga eventos.csv. El factor fac multiplica la velocidad objetivo."""
    eventos = []
    if not path.exists():
        print(f"[eventos] ADVERTENCIA: no existe {path}; se ejecutara sin eventos dinamicos")
        return eventos

    with open(path, "r", newline="", encoding="utf-8-sig") as f:
        for fila in csv.DictReader(f):
            try:
                evento = {
                    "id": str(fila.get("id", "")).strip() or "EVENTO",
                    "tipo": str(fila.get("tipo", "")).strip().lower(),
                    "ini": float(str(fila.get("ini", "0")).replace(",", ".")),
                    "fin": float(str(fila.get("fin", "0")).replace(",", ".")),
                    "zona": str(fila.get("zona", "red")).strip().lower(),
                    "edges": str(fila.get("edges", "todos")).strip(),
                    "fac": float(str(fila.get("fac", "1") or "1").replace(",", ".")),
                    "act": str(fila.get("act", "reducir")).strip().lower(),
                    "src": str(fila.get("src", "")).strip(),
                    "objetivo": str(fila.get("objetivo", "")).strip(),
                    "duracion_s": float(str(fila.get("duracion_s", "0") or "0").replace(",", ".")),
                    "factor_carga": float(str(fila.get("factor_carga", "1") or "1").replace(",", ".")),
                    "seed": int(float(str(fila.get("seed", "2026") or "2026").replace(",", "."))),
                }
            except Exception as exc:
                print(f"[eventos] fila invalida omitida: {fila} error={exc}")
                continue

            if evento["fin"] <= evento["ini"] or evento["fac"] <= 0:
                print(f"[eventos] evento invalido omitido: {evento}")
                continue
            eventos.append(evento)

    print(f"[eventos] cargados={len(eventos)} desde {path}")
    for ev in eventos:
        if ev["tipo"] in ("averia_camion", "falla_camion"):
            print(
                f"[eventos] {ev['id']} tipo={ev['tipo']} inicio={ev['ini']:.1f}s "
                f"reparacion={ev.get('duracion_s', 0):.1f}s objetivo={ev.get('objetivo','random')} "
                f"seed={ev.get('seed', 2026)}"
            )
        elif ev["tipo"] in ("averia_pala", "falla_pala"):
            print(
                f"[eventos] {ev['id']} tipo={ev['tipo']} intervalo={ev['ini']:.1f}-{ev['fin']:.1f}s "
                f"factor_carga={ev.get('factor_carga', 1.0):.3f} "
                f"objetivo={ev.get('objetivo','random')} seed={ev.get('seed', 2026)}"
            )
        else:
            reduccion = (1.0 - ev["fac"]) * 100.0
            print(f"[eventos] {ev['id']} tipo={ev['tipo']} intervalo={ev['ini']:.1f}-{ev['fin']:.1f}s "
                  f"factor={ev['fac']:.4f} reduccion={reduccion:.2f}% edges={ev['edges']}")
    return eventos

def seleccionar_eventos(eventos, seleccion_arg=""):
    """Permite elegir eventos por consola o mediante --evento.

    Valores admitidos:
      - 0 / ninguno / sin: ejecutar sin eventos.
      - todos / all: aplicar todos los eventos del CSV.
      - numero: seleccionar una fila del menu.
      - id o tipo: seleccionar por identificador o tipo.
      - lista separada por comas: por ejemplo E1,E3.
    """
    if not eventos:
        return []

    def resolver(seleccion):
        valor = str(seleccion or "").strip()
        valor_lower = valor.lower()

        if valor_lower in ("0", "ninguno", "ninguna", "sin", "none"):
            return []
        if valor_lower in ("todos", "todas", "all", "*"):
            return list(eventos)

        tokens = [t.strip() for t in valor.split(",") if t.strip()]
        elegidos = []
        for token in tokens:
            token_lower = token.lower()
            if token.isdigit():
                idx = int(token) - 1
                if 0 <= idx < len(eventos):
                    candidato = eventos[idx]
                    if candidato not in elegidos:
                        elegidos.append(candidato)
                continue

            for evento in eventos:
                if (str(evento.get("id", "")).lower() == token_lower
                        or str(evento.get("tipo", "")).lower() == token_lower):
                    if evento not in elegidos:
                        elegidos.append(evento)
        return elegidos

    if str(seleccion_arg or "").strip():
        elegidos = resolver(seleccion_arg)
        if not elegidos and str(seleccion_arg).strip().lower() not in ("0", "ninguno", "sin", "none"):
            raise ValueError(f"Seleccion de evento no valida: {seleccion_arg}")
        return elegidos

    print("\n===================================================")
    print("SELECCION DE EVENTO DINAMICO")
    print("===================================================")
    print("0) Ejecutar SIN evento dinamico")
    for i, evento in enumerate(eventos, start=1):
        tipo = str(evento.get("tipo", "") or "").lower()
        if tipo in ("averia_camion", "falla_camion"):
            detalle = (
                f"inicio={evento.get('ini',0):.0f}s "
                f"reparacion={evento.get('duracion_s',0):.0f}s "
                f"objetivo={evento.get('objetivo','random') or 'random'}"
            )
        elif tipo in ("averia_pala", "falla_pala"):
            detalle = (
                f"[{evento.get('ini',0):.0f}-{evento.get('fin',0):.0f}s] "
                f"factor_carga={evento.get('factor_carga',1.0):.2f} "
                f"objetivo={evento.get('objetivo','random') or 'random'}"
            )
        else:
            reduccion = max(0.0, (1.0 - float(evento.get("fac", 1.0))) * 100.0)
            detalle = (
                f"[{evento.get('ini',0):.0f}-{evento.get('fin',0):.0f}s] "
                f"factor={evento.get('fac',1.0):.4f} reduccion={reduccion:.2f}% "
                f"edges={evento.get('edges','todos')}"
            )
        print(f"{i}) {evento.get('id','')} - {tipo} {detalle}")
    print("T) Aplicar TODOS los eventos")
    print("También puedes escribir varios números o IDs separados por coma: 1,3 o E1,E3")

    while True:
        seleccion = input("Seleccione evento(s): ").strip()
        if seleccion.lower() == "t":
            seleccion = "todos"
        elegidos = resolver(seleccion)
        if elegidos or seleccion.lower() in ("0", "ninguno", "ninguna", "sin", "none"):
            return elegidos
        print("Seleccion no valida. Intente nuevamente.")

def ajustar_derrumbe(eventos_seleccionados, horas_arg=None):
    """Ajusta la duración de los derrumbes seleccionados.

    Si existe al menos un evento tipo derrumbe:
    - usa --derrumbe-horas cuando se entrega por comando;
    - en caso contrario solicita la cantidad de horas por consola.

    El instante de inicio configurado en eventos.csv se conserva y solo se
    recalcula el fin: fin = ini + horas * 3600.
    """
    derrumbes = [
        ev for ev in eventos_seleccionados
        if str(ev.get("tipo", "")).strip().lower() == "derrumbe"
    ]
    if not derrumbes:
        return eventos_seleccionados

    horas = horas_arg
    while horas is None or float(horas) <= 0:
        try:
            entrada = input(
                "Ingrese cuántas HORAS debe permanecer activo el derrumbe "
                "(se permiten decimales, por ejemplo 0.5): "
            ).strip().replace(",", ".")
            horas = float(entrada)
            if horas <= 0:
                print("La duración debe ser mayor que 0 horas.")
                horas = None
        except ValueError:
            print("Valor no válido. Ejemplos válidos: 1, 0.5, 2.25")
            horas = None

    duracion_s = float(horas) * 3600.0
    for evento in derrumbes:
        inicio_s = float(evento.get("ini", 0.0))
        evento["fin"] = inicio_s + duracion_s
        evento["duracion_horas"] = float(horas)
        print(
            f"[derrumbe] {evento.get('id','DERRUMBE')} activo por "
            f"{float(horas):.3f} h ({duracion_s:.1f} s), "
            f"intervalo={inicio_s:.1f}-{evento['fin']:.1f}s"
        )

    return eventos_seleccionados

def evento_aplica_en_edge(evento, sim_time, edge_actual):
    if not (evento["ini"] <= float(sim_time) < evento["fin"]):
        return False
    edges_txt = str(evento.get("edges", "todos")).strip().lower()
    if edges_txt in ("", "todos", "all", "red"):
        return True
    permitidos = {x.strip() for x in re.split(r"[;,|\s]+", edges_txt) if x.strip()}
    return str(edge_actual or "").strip() in permitidos

def aplicar_velocidad(speed_base, sim_time, edge_actual, eventos):
    """Retorna velocidad final y lista de eventos aplicados."""
    velocidad = float(speed_base)
    aplicados = []
    for evento in eventos:
        if not evento_aplica_en_edge(evento, sim_time, edge_actual):
            continue
        if evento.get("act") == "reducir":
            antes = velocidad
            velocidad = max(0.1, velocidad * float(evento.get("fac", 1.0)))
            aplicados.append((evento, antes, velocidad))
    return velocidad, aplicados

def obtener_net(sumocfg_path: Path) -> Path:
    root = ET.parse(sumocfg_path).getroot()
    net_file = root.find("./input/net-file")
    if net_file is None:
        raise ValueError(f"No se encontro <net-file> dentro de {sumocfg_path}")

    valor = str(net_file.get("value", "")).strip()
    if not valor:
        raise ValueError(f"El <net-file> de {sumocfg_path} no tiene value")

    net_path = Path(valor)
    if not net_path.is_absolute():
        net_path = (sumocfg_path.parent / net_path).resolve()

    if not net_path.exists():
        raise FileNotFoundError(f"No existe la red SUMO indicada en el sumocfg: {net_path}")
    return net_path

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true", help="Usar sumo-gui")
    parser.add_argument("--force-speed", action="store_true", help="Usa speedMode=0 solo para comparar; no recomendado como corrida final")
    parser.add_argument("--sumocfg", default=str(SUMOCFG_DEFAULT), help="Archivo .sumocfg")
    parser.add_argument("--rou", default=str(ROU_DEFAULT), help="Archivo .rou.xml")
    parser.add_argument("--step-length", default="1.0", help="Step length de SUMO; usa --step-length 0.1 para volver a 0.1 s")
    parser.add_argument("--segment-report", default=str(REPORTE_SEGMENTOS_DEFAULT), help="CSV de salida con tiempos por segmento")
    parser.add_argument("--event-report", default=str(EVENTOS_DINAMICOS_DEFAULT), help="CSV de salida con eventos dinamicos detectados")
    parser.add_argument("--collapse-report", default=str(REPORTE_DERRUMBE_DEFAULT), help="CSV de cierre y desvios provocados por derrumbes")
    parser.add_argument("--events-config", default=str(EVENTOS_CONFIG_DEFAULT), help="CSV de configuracion de eventos dinamicos")
    parser.add_argument("--evento", default="", help="Numero, id, tipo, varios separados por coma, todos o ninguno")
    parser.add_argument("--derrumbe-horas", type=float, default=None, help="Horas que permanece activo cada derrumbe seleccionado")
    parser.add_argument("--rescheduling-report", default=str(RESCHEDULING_DEFAULT), help="CSV de salida con solicitud de rescheduling global por retraso total de flota")
    parser.add_argument("--snapshot-report", default=str(SNAPSHOT_FLOTA_DEFAULT), help="CSV de salida con la fotografia de flota al activar rescheduling")
    parser.add_argument("--snapshot-json", default=str(SNAPSHOT_FLOTA_JSON_DEFAULT), help="JSON de salida con la ultima fotografia de flota")
    parser.add_argument("--monitoring-snapshot-report", default=str(SNAPSHOT_MONITOREO_DEFAULT), help="CSV acumulado con fotografias de monitoreo cada 10 segundos")
    parser.add_argument("--monitoring-snapshot-json", default=str(SNAPSHOT_MONITOREO_JSON_DEFAULT), help="JSON de salida con la ultima fotografia de monitoreo")
    parser.add_argument("--tol-jade", type=float, default=TOL_JADE, help="Tolerancia global: 0.10 equivale al 10% del cronograma JADE")
    parser.add_argument("--delay-threshold", type=float, default=None, help="Compatibilidad antigua: no se usa; se mantiene para no romper comandos anteriores")
    parser.add_argument("--server-endpoint", default=SERVER_ZMQ_ENDPOINT_DEFAULT, help="Endpoint ZMQ de server.py para registrar rescheduling")
    parser.add_argument("--no-register-server", action="store_true", help="No enviar REGISTER_RESCHEDULE_REQUEST al server.py")
    parser.add_argument("--no-approach-slowdown", action="store_true", help="Desactiva reduccion gradual al aproximarse al destino del segmento")
    parser.add_argument("--approach-d1", type=float, default=APROX_DIST_1_M, help="Distancia 1 de aproximacion al destino del segmento (m)")
    parser.add_argument("--approach-d2", type=float, default=APROX_DIST_2_M, help="Distancia 2 de aproximacion al destino del segmento (m)")
    parser.add_argument("--approach-d3", type=float, default=APROX_DIST_3_M, help="Distancia 3 de aproximacion al destino del segmento (m)")
    parser.add_argument("--approach-v1", type=float, default=APROX_SPEED_1_MS, help="Velocidad max si distancia restante <= d1 (m/s)")
    parser.add_argument("--approach-v2", type=float, default=APROX_SPEED_2_MS, help="Velocidad max si distancia restante <= d2 (m/s)")
    parser.add_argument("--approach-v3", type=float, default=APROX_SPEED_3_MS, help="Velocidad max si distancia restante <= d3 (m/s)")
    parser.add_argument(
        "--resource-interval",
        type=float,
        default=1.0,
        help="Intervalo en segundos para registrar CPU y memoria RAM",
    )
    parser.add_argument(
        "--no-metrics",
        action="store_true",
        help="Desactiva el calculo automatico de metricas de SUMO",
    )
    args = parser.parse_args()

    sumocfg = Path(args.sumocfg)
    rou = Path(args.rou)
    reporte_path = Path(args.segment_report)
    eventos_path = Path(args.event_report)
    reporte_derrumbe_path = Path(args.collapse_report)
    eventos_config_path = Path(args.events_config)
    rescheduling_path = Path(args.rescheduling_report)
    snapshot_path = Path(args.snapshot_report)
    snapshot_json_path = Path(args.snapshot_json)
    snapshot_monitoreo_path = Path(args.monitoring_snapshot_report)
    snapshot_monitoreo_json_path = Path(args.monitoring_snapshot_json)

    gestor_metricas = None
    if not args.no_metrics:
        gestor_metricas = GestorMetricas(
            escenario=ESCENARIO,
            output_dir=BASE_DIR,
            intervalo_recursos_s=max(0.2, float(args.resource_interval)),
        )
        gestor_metricas.iniciar()

    if not sumocfg.exists():
        raise FileNotFoundError(f"No existe sumocfg: {sumocfg}")
    if not rou.exists():
        raise FileNotFoundError(f"No existe rou: {rou}")

    if "SUMO_HOME" not in os.environ:
        print("ERROR: SUMO_HOME no esta definido. Ejecuta desde la consola de SUMO o configura SUMO_HOME.")
        sys.exit(1)

    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    if tools not in sys.path:
        sys.path.append(tools)

    import traci

    planes = cargar_plan_segmentos(rou)
    total_jade_s = calcular_total_jade_s(planes)
    umbral_global_s = total_jade_s * float(args.tol_jade)

    net_path = obtener_net(sumocfg)
    net = NetSumo(
        str(net_path),
        objects_path=str(OBJECTS_INPUT),
        trucks_path=str(TRUCKS_INPUT),
    )
    eventos_disponibles = cargar_eventos(eventos_config_path)
    eventos_configurados = seleccionar_eventos(eventos_disponibles, args.evento)
    eventos_configurados = ajustar_derrumbe(
        eventos_configurados,
        horas_arg=args.derrumbe_horas,
    )
    if eventos_configurados:
        print("[eventos] seleccionados: " + ", ".join(
            f"{ev.get('id','')}:{ev.get('tipo','')}" for ev in eventos_configurados
        ))
    else:
        print("[eventos] corrida SIN eventos dinamicos")

    binary = "sumo-gui.exe" if args.gui else "sumo.exe"
    cmd = [
        binary,
        "-c", str(sumocfg),
        "--step-length", str(args.step_length),
        "--time-to-teleport", "600",
    ]

    print("===================================================")
    print("SUMO TraCI usando velocidad objetivo por segmento")
    print("+ reporte_segmentos_sumo.csv")
    print("===================================================")
    print(f"sumocfg: {sumocfg}")
    print(f"rou:     {rou}")
    print(f"reporte: {reporte_path}")
    print(f"eventos: {eventos_path}")
    print(f"rescheduling: {rescheduling_path}")
    print(f"snapshot: {snapshot_path}")
    print(f"cronograma JADE: {total_jade_s:.1f}s")
    print(f"umbral global flota: {umbral_global_s:.1f}s")
    print(f"server rescheduling: {args.server_endpoint} activo={not args.no_register_server}")
    print(f"vehiculos con plan: {len(planes)}")
    print(f"step-length: {args.step_length}")
    print(f"force-speed: {args.force_speed}")
    print(f"metricas SUMO: {not args.no_metrics} intervalo_recursos={args.resource_interval:.2f}s")
    print(f"aproximacion a destino: {not args.no_approach_slowdown} "
          f"d=({args.approach_d1:.0f},{args.approach_d2:.0f},{args.approach_d3:.0f})m "
          f"v=({args.approach_v1:.1f},{args.approach_v2:.1f},{args.approach_v3:.1f})m/s")
    print("cmd:", " ".join(cmd))
    print("---------------------------------------------------")

    controlados = set()
    cambios = 0
    ultimo_estado = {}
    conectado = False
    final_time = 0.0
    resumen_metricas = None
    dynamic_manager = None
    activos = {}
    cerrados = set()
    eventos_registrados = set()
    solicitudes_rescheduling = {}
    retrasos_flota_actual = {}
    vivos_prev = set()

    # Seguimiento robusto de vehiculos que dejan temporalmente getIDList().
    # Un teleport de SUMO NO debe eliminar el ciclo activo ni la cola pendiente.
    # Una ausencia no explicada se confirma durante unos segundos antes de
    # considerarla una desaparicion real.
    vehiculos_ausentes_desde = {}
    vehiculos_en_teleport = set()
    TIEMPO_CONFIRMACION_AUSENCIA_S = 10.0

    rescheduling_activo = False
    # Se activa cuando server.py informa que JADE ya envió FINALIZE_RESCHEDULE.
    # Desde ese momento no pueden llegar ciclos nuevos.
    rescheduling_finalizado = False
    congelar_despues_s = {}
    disponibilidad_objetivo = {}
    # Truck -> {inicio_s, edge, duracion_s}. Permite confirmar la descarga por tiempo real.
    descarga_en_curso_observada = {}
    congelados = set()
    reschedule_version_aplicada = 0
    proxima_consulta_plan_s = 0.0
    ultimo_cycle_id_recibido = 0
    camiones_disponibilidad_informada = set()
    # Camiones que existen fisicamente en SUMO pero no tienen trabajo pendiente.
    # Pueden estar libres desde t=0 (sin scheduling) o después de su última descarga.
    camiones_disponibles = {}
    descarga_final_inicio_observada = {}
    ultimo_intento_truck_free_s = {}
    ciclos_pendientes_por_camion = {}
    ciclo_activo_por_camion = {}
    camiones_fin_turno = set()
    camiones_retirados = set()
    ultimo_cierre = -60.0
    # Evita repetir en cada step el mismo mensaje mientras un ciclo espera su hora JADE.
    esperas_hora_reportadas = set()

    # Monitoreo periódico: se toman fotografías cada 10 s para análisis,
    # pero solo una de ellas se congela como snapshot oficial del rescheduling.
    proximo_snapshot_monitoreo_s = 0.0
    proxima_evaluacion_rescheduling_s = SNAPSHOT_MONITOREO_INTERVAL_S
    contexto_retraso_actual = {}

    reporte_path.parent.mkdir(parents=True, exist_ok=True)
    csv_file = open(reporte_path, "w", newline="", encoding="utf-8-sig")
    fieldnames = [
        "camion", "tipo_plan", "cycle_id", "secuencia_ciclo",
        "seg_num", "operacion", "ruta", "edge_inicio", "edge_fin",
        "idx_inicio", "idx_fin", "edges_count", "dist_m",
        "speed_ref_ms", "speed_compensada_ms", "objetivo_alcanzable",
        "jade_inicio_s", "jade_fin_s", "jade_duracion_s",
        "sumo_inicio_s", "sumo_fin_s", "sumo_duracion_s",
        "diferencia_s", "error_relativo_pct",
        "time_loss_inicio_s", "time_loss_fin_s", "time_loss_segmento_s",
        "cierre_motivo"
    ]
    writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    writer.writeheader()

    eventos_path.parent.mkdir(parents=True, exist_ok=True)
    eventos_file = open(eventos_path, "w", newline="", encoding="utf-8-sig")
    eventos_fieldnames = [
        "tiempo_sumo_s", "camion", "seg_num", "operacion", "ruta",
        "jade_inicio_s", "jade_fin_s", "jade_duracion_s", "retraso_s", "total_jade_s", "umbral_s",
        "edge_actual", "lane_actual", "pos_actual", "velocidad_actual_s",
        "route_index", "evento", "accion"
    ]
    eventos_writer = csv.DictWriter(eventos_file, fieldnames=eventos_fieldnames)
    eventos_writer.writeheader()

    rescheduling_path.parent.mkdir(parents=True, exist_ok=True)
    rescheduling_fieldnames = [
        "tiempo_primera_deteccion_s", "tiempo_ultima_deteccion_s",
        "camion", "camion_disparador", "primer_segmento", "ultimo_segmento",
        "operacion_primera", "operacion_ultima",
        "ruta_primera", "ruta_ultima",
        "retraso_inicial_s", "retraso_max_s", "retraso_total_flota_s",
        "total_jade_s", "umbral_s", "cantidad_eventos", "camiones_con_retraso",
        "edge_actual", "lane_actual", "pos_actual", "velocidad_actual_s", "route_index_actual",
        "evento", "accion", "alcance", "estado", "aplicar_desde", "server_ok", "server_accion", "server_error"
    ]

    snapshot_fieldnames = [
        "tiempo_sumo_s", "camion", "operacion_planificada", "operacion_estimada",
        "seg_num", "ruta", "edge_actual", "lane_actual", "pos_actual",
        "velocidad_actual_s", "route_index", "distancia_restante_segmento_m",
        "edge_fin_segmento", "jade_inicio_s", "jade_fin_s", "accion_rescheduling",
        "disponibilidad", "ubicacion_referencia",
        "ubicacion_disponible", "edge_disponible", "tiempo_disponible_estimado_s",
        "aplicar_desde", "commitment_restante_s", "observacion"
    ]

    # Archivo acumulado de monitoreo: se reinicia en cada corrida y luego se agregan
    # fotografías periódicas. Este archivo NO es usado por JADE para renegociar.
    snapshot_monitoreo_path.parent.mkdir(parents=True, exist_ok=True)
    with open(snapshot_monitoreo_path, "w", newline="", encoding="utf-8-sig") as mf:
        mw = csv.DictWriter(mf, fieldnames=snapshot_fieldnames)
        mw.writeheader()

    def accion_estado(operacion_estimada, operacion_planificada, distancia_restante, args):
        op_est = str(operacion_estimada or "").upper()
        op_plan = str(operacion_planificada or "").upper()

        if op_est == "DESCARGA":
            return "TERMINAR_DESCARGA", "DESPUES_DESCARGA"

        if op_est == "CARGA":
            return "TERMINAR_CARGA_Y_LUEGO_DESCARGA", "DESPUES_DESCARGA"

        if op_plan == "VIAJE_CARGADO":
            return "CONTINUAR_HASTA_DESCARGA", "DESPUES_DESCARGA"

        if op_plan == "VIAJE_VACIO":
            return "CONTINUAR_A_PALA_CARGA_Y_DESCARGA", "DESPUES_DESCARGA"

        return "ESPERAR_NUEVO_SCHEDULE", "AHORA"

    def destino_logico_segmento(seg):
        """Obtiene el destino logico desde el texto ruta origen->destino."""
        ruta = str(seg.get("ruta", "") or "")
        if "->" in ruta:
            return ruta.split("->", 1)[1].strip()
        return str(seg.get("edge_fin", "") or "")

    def siguiente_segmento(plan, seg_idx, operacion):
        operacion = str(operacion or "").upper()
        segmentos = plan.get("segmentos", [])
        for j in range(max(0, int(seg_idx) + 1), len(segmentos)):
            if str(segmentos[j].get("operacion", "")).upper() == operacion:
                return j, segmentos[j]
        return None, None

    def indice_plan_actual(traci, veh_id, plan, route_index_sumo=None, edge_actual=None):
        """
        Traduce el routeIndex fisico de SUMO al indice logico del plan activo.

        En el scheduling inicial ambos indices coinciden. En rescheduling, setRoute()
        puede conservar/reiniciar un routeIndex que no parte en cero. Por eso cada
        ciclo guarda la base observada al instalar su ruta y un mapa entre las
        posiciones de la ruta aplicada y los indices originales del plan JADE.

        Si SUMO reajusta el routeIndex despues de setRoute(), la funcion se
        resincroniza usando el edge fisico actual.
        """
        try:
            idx_sumo = int(
                traci.vehicle.getRouteIndex(veh_id)
                if route_index_sumo is None
                else route_index_sumo
            )
        except Exception:
            return None

        if idx_sumo < 0:
            return None

        mapa = list(plan.get("_ruta_aplicada_plan_indices", []) or [])
        ruta_aplicada = list(plan.get("_ruta_aplicada_edges", []) or [])
        if not mapa or not ruta_aplicada:
            return idx_sumo

        if edge_actual is None:
            try:
                edge_actual = str(traci.vehicle.getRoadID(veh_id) or "").strip()
            except Exception:
                edge_actual = ""
        else:
            edge_actual = str(edge_actual or "").strip()

        try:
            base = int(plan.get("_route_index_base_sumo", idx_sumo))
        except Exception:
            base = idx_sumo

        pos_rel = idx_sumo - base
        pos = None

        if 0 <= pos_rel < len(mapa):
            if not edge_actual or str(ruta_aplicada[pos_rel]) == edge_actual:
                pos = pos_rel

        if pos is None and edge_actual and not edge_actual.startswith(":"):
            candidatos = [
                i for i, edge in enumerate(ruta_aplicada)
                if str(edge) == edge_actual
            ]
            if candidatos:
                try:
                    ultima_pos = int(plan.get("_ruta_pos_ultima", -1))
                except Exception:
                    ultima_pos = -1
                adelante = [i for i in candidatos if i >= max(0, ultima_pos)]
                bolsa = adelante or candidatos
                if 0 <= pos_rel < len(mapa):
                    pos = min(bolsa, key=lambda i: abs(i - pos_rel))
                else:
                    pos = bolsa[0]

                nueva_base = idx_sumo - pos
                if nueva_base != base:
                    plan["_route_index_base_sumo"] = int(nueva_base)
                    if not plan.get("_route_index_resync_reportado"):
                        print(
                            f"[rescheduling] Resincronizacion indice {veh_id}: "
                            f"base_anterior={base} base_nueva={nueva_base} "
                            f"idx_sumo={idx_sumo} pos_ruta={pos} edge={edge_actual}"
                        )
                        plan["_route_index_resync_reportado"] = True

        if pos is None:
            if 0 <= pos_rel < len(mapa):
                pos = pos_rel
            else:
                return None

        try:
            ultima_pos = int(plan.get("_ruta_pos_ultima", -1))
        except Exception:
            ultima_pos = -1
        if pos >= ultima_pos:
            plan["_ruta_pos_ultima"] = int(pos)

        if pos < 0 or pos >= len(mapa):
            return None
        return int(mapa[pos])

    def es_id_pala(valor):
        return bool(re.match(r"^PA\d+", str(valor or "").strip().upper()))

    def registrar_pala(disponibilidad, pala, tiempo_s, motivo, camion="", detalle=""):
        pala = str(pala or "").strip()
        if not pala or not es_id_pala(pala):
            return
        tiempo_s = max(0.0, float(tiempo_s or 0.0))
        existente = disponibilidad.get(pala)
        if existente is None or tiempo_s > float(existente.get("tiempo_disponible_s", 0.0) or 0.0):
            disponibilidad[pala] = {
                "pala": pala,
                "tiempo_disponible_s": round(tiempo_s, 3),
                "tiempo_disponible_ms": int(round(tiempo_s * 1000.0)),
                "motivo": motivo,
                "camion_comprometido": camion,
                "detalle": detalle,
            }

    def estado_palas(traci, sim_time, planes):
        """
        Calcula disponibilidad operacional de cada pala al momento del rescheduling.

        Regla usada:
        - Si una pala no tiene compromiso activo, queda disponible desde sim_time.
        - Si un camion ya va vacio hacia esa pala, la pala se reserva hasta que ese camion llegue y termine CARGA.
        - Si el camion ya esta en el stop de CARGA, la pala se reserva hasta terminar esa carga.
        - Viaje cargado/descarga ya no ocupan pala.

        Este calculo se ejecuta solo cuando se dispara el rescheduling, no en cada step.
        """
        disponibilidad = {}

        # Primero registra todas las palas conocidas como libres desde el tiempo de deteccion.
        for plan in planes.values():
            for seg_known in plan.get("segmentos", []):
                if str(seg_known.get("operacion", "")).upper() == "VIAJE_VACIO":
                    pala_known = destino_logico_segmento(seg_known)
                    if es_id_pala(pala_known):
                        registrar_pala(
                            disponibilidad,
                            pala_known,
                            float(sim_time),
                            "LIBRE_DESDE_RESCHEDULING",
                            "",
                            "sin_compromiso_activo_detectado"
                        )

        try:
            vehiculos = list(traci.vehicle.getIDList())
        except Exception:
            vehiculos = []

        for vid in sorted(vehiculos):
            plan = planes.get(vid)
            if not plan:
                continue

            try:
                route_index_sumo = int(traci.vehicle.getRouteIndex(vid))
            except Exception:
                route_index_sumo = -1
            route_index = indice_plan_actual(
                traci, vid, plan, route_index_sumo=route_index_sumo
            )
            if route_index is None:
                route_index = -1

            # Un camion ya disponible no mantiene ningun compromiso pendiente
            # con una pala. La fotografia detallada de la flota se construye en
            # tomar_snapshot_flota(); aqui solo se calcula disponibilidad de palas.
            if vid in camiones_disponibles:
                continue

            seg_idx = plan.get("index_to_segment", {}).get(route_index)
            if seg_idx is None or seg_idx < 0 or seg_idx >= len(plan.get("segmentos", [])):
                continue

            seg = plan["segmentos"][seg_idx]
            op_plan = str(seg.get("operacion", "")).upper()
            if op_plan != "VIAJE_VACIO":
                # Si ya va cargado o descargando, la pala ya no queda ocupada por ese camion.
                continue

            pala = destino_logico_segmento(seg)
            if not es_id_pala(pala):
                continue

            try:
                lane_actual = traci.vehicle.getLaneID(vid)
            except Exception:
                lane_actual = ""

            distancia_restante = distancia_segmento(traci, vid, plan, seg, route_index, lane_actual)
            velocidad_seg = max(0.1, float(seg.get("speed_ms", plan.get("vacio_ms", 1.0)) or 1.0))
            restante_viaje_s = max(0.0, float(distancia_restante or 0.0)) / velocidad_seg
            carga_s = float(seg.get("stop_after_duration_s", 0.0) or 0.0)

            en_stop = esta_en_stop_o_parking(traci, vid)
            if en_stop:
                # Aproximacion conservadora: si ya esta cargando, se reserva por la duracion de carga.
                # Si el plan permite estimar fin de carga mas cercano, se usa el mayor para no liberar antes de tiempo.
                carga_fin_plan_s = float(seg.get("arrival_ms", 0) or 0) / 1000.0 + carga_s
                tiempo_disponible = max(float(sim_time) + carga_s, carga_fin_plan_s)
                registrar_pala(
                    disponibilidad,
                    pala,
                    tiempo_disponible,
                    "CAMION_EN_CARGA",
                    vid,
                    f"carga_s={carga_s:.3f}"
                )
            else:
                tiempo_disponible = float(sim_time) + restante_viaje_s + carga_s
                registrar_pala(
                    disponibilidad,
                    pala,
                    tiempo_disponible,
                    "CAMION_VIAJE_VACIO_HACIA_PALA",
                    vid,
                    f"restante_viaje_s={restante_viaje_s:.3f};carga_s={carga_s:.3f}"
                )

        return disponibilidad

    def estado_rescheduling(plan, seg_idx, seg, op_est, op_plan, edge_actual, sim_time, distancia_restante):
        """
        Define desde donde y cuando JADE debe considerar disponible al camion.

        Regla operacional:
        - Si va vacio hacia una pala, termina pala->carga->botadero->descarga.
        - Si esta cargando, termina carga->botadero->descarga.
        - Si va cargado, llega al botadero y descarga.
        - Si esta descargando, termina la descarga.
        - Si no hay compromiso claro, queda disponible ahora en el edge actual.
        """
        op_est = str(op_est or "").upper()
        op_plan = str(op_plan or "").upper()
        segmentos = plan.get("segmentos", [])
        velocidad_seg = max(0.1, float(seg.get("speed_ms", plan.get("vacio_ms", 1.0)) or 1.0)) if seg else 1.0
        restante_viaje_s = max(0.0, float(distancia_restante or 0.0)) / max(0.1, velocidad_seg)

        def desde_viaje_cargado(seg_cargado, incluir_restante_actual=False):
            descarga_s = float(seg_cargado.get("stop_after_duration_s", 0.0) or 0.0)
            edge_disp = str(seg_cargado.get("edge_fin", "") or edge_actual)
            ubic_disp = destino_logico_segmento(seg_cargado)
            if incluir_restante_actual:
                compromiso_s = restante_viaje_s + descarga_s
            else:
                # Si es un viaje cargado futuro, se usa su duracion JADE completa.
                compromiso_s = float(seg_cargado.get("dur_ms", 0) or 0) / 1000.0 + descarga_s
            tiempo_disp = float(sim_time) + max(0.0, compromiso_s)
            return ubic_disp, edge_disp, tiempo_disp, compromiso_s

        if seg is None:
            return str(edge_actual), str(edge_actual), float(sim_time), 0.0, "AHORA"

        # Camion en descarga o llegando al botadero.
        if op_est == "DESCARGA" or op_plan == "VIAJE_CARGADO":
            ubic, edge, tdisp, comp = desde_viaje_cargado(seg, incluir_restante_actual=(op_plan == "VIAJE_CARGADO" and op_est != "DESCARGA"))
            if op_est == "DESCARGA":
                descarga_s = float(seg.get("stop_after_duration_s", 0.0) or 0.0)
                tdisp = float(sim_time) + descarga_s
                comp = descarga_s
            return ubic, edge, tdisp, comp, "DESPUES_DESCARGA"

        # Camion va vacio a pala o esta cargando: debe completar carga + viaje cargado + descarga.
        if op_plan == "VIAJE_VACIO" or op_est == "CARGA":
            idx_cargado, seg_cargado = siguiente_segmento(plan, seg_idx, "VIAJE_CARGADO")
            carga_s = float(seg.get("stop_after_duration_s", 0.0) or 0.0)
            if seg_cargado is not None:
                ubic, edge, t_cargado, comp_cargado = desde_viaje_cargado(seg_cargado, incluir_restante_actual=False)
                if op_est == "CARGA":
                    comp = carga_s + comp_cargado
                else:
                    comp = restante_viaje_s + carga_s + comp_cargado
                return ubic, edge, float(sim_time) + max(0.0, comp), comp, "DESPUES_DESCARGA"

            # Respaldo: no existe viaje cargado posterior en el plan. Se deja disponible en la pala.
            edge_pala = str(seg.get("edge_fin", "") or edge_actual)
            ubic_pala = destino_logico_segmento(seg)
            comp = (0.0 if op_est == "CARGA" else restante_viaje_s) + carga_s
            return ubic_pala, edge_pala, float(sim_time) + max(0.0, comp), comp, "DESPUES_CARGA_SIN_VIAJE_CARGADO"

        return str(edge_actual), str(edge_actual), float(sim_time), 0.0, "AHORA"

    def tomar_snapshot_flota(traci, sim_time, planes, args):
        """Toma una fotografia de TODOS los camiones vivos al momento del rescheduling."""
        snapshot = []
        try:
            vehiculos = list(traci.vehicle.getIDList())
        except Exception:
            vehiculos = []

        for vid in sorted(vehiculos):
            plan = planes.get(vid)
            if not plan:
                continue
            try:
                edge_actual = traci.vehicle.getRoadID(vid)
            except Exception:
                edge_actual = ""
            try:
                lane_actual = traci.vehicle.getLaneID(vid)
            except Exception:
                lane_actual = ""
            try:
                pos_actual = float(traci.vehicle.getLanePosition(vid))
            except Exception:
                pos_actual = 0.0
            try:
                velocidad_actual_s = float(traci.vehicle.getSpeed(vid))
            except Exception:
                velocidad_actual_s = 0.0
            try:
                route_index_sumo = int(traci.vehicle.getRouteIndex(vid))
            except Exception:
                route_index_sumo = -1
            route_index = indice_plan_actual(
                traci, vid, plan, route_index_sumo=route_index_sumo, edge_actual=edge_actual
            )
            if route_index is None:
                route_index = -1

            seg = None
            seg_idx = plan.get("index_to_segment", {}).get(route_index)
            if seg_idx is not None and 0 <= seg_idx < len(plan.get("segmentos", [])):
                seg = plan["segmentos"][seg_idx]

            # El routeIndex de SUMO puede permanecer durante algunos steps en el
            # ultimo edge de un segmento que ya fue cerrado. Esto ocurre, por
            # ejemplo, cuando el camion termina su PA_WAIT y comienza a salir del
            # botadero. En ese intervalo no se debe volver a considerar pendiente
            # la descarga anterior.
            en_stop = esta_en_stop_o_parking(traci, vid)
            primer_parking = ""
            try:
                stops_snapshot = list(traci.vehicle.getStops(vid))
                if stops_snapshot:
                    primer_parking = str(
                        getattr(stops_snapshot[0], "stoppingPlaceID", "") or ""
                    ).strip()
            except Exception:
                primer_parking = ""

            segmento_referenciado_cerrado = False
            segmento_ajustado_al_siguiente = False
            descarga_ya_finalizada_en_wait = False

            if seg is not None:
                key_snapshot = (
                    vid,
                    str(seg.get("tipo_plan", "SCHEDULING") or "SCHEDULING"),
                    int(seg.get("cycle_id", 0) or 0),
                    int(seg.get("seg_num", 0) or 0),
                )
                segmento_referenciado_cerrado = key_snapshot in cerrados

            if segmento_referenciado_cerrado:
                # PA_AUTO significa que el viaje ya termino, pero la descarga
                # fisica aun puede estar ejecutandose. En ese caso se conserva el
                # segmento anterior hasta que termine el stop operacional.
                en_descarga_auto = bool(
                    en_stop and primer_parking.startswith("PA_AUTO_")
                )

                # PA_WAIT posterior a la descarga confirma que el compromiso ya
                # termino y el camion puede ofrecerse inmediatamente a JADE.
                descarga_ya_finalizada_en_wait = bool(
                    en_stop and primer_parking.startswith("PA_WAIT_")
                )

                # Si ya no esta en PA_AUTO ni PA_WAIT, el camion comenzo a salir
                # hacia el siguiente segmento aunque routeIndex aun apunte al
                # anterior. Se avanza al primer segmento que no este cerrado.
                if (
                    not en_stop
                    and not en_descarga_auto
                    and not descarga_ya_finalizada_en_wait
                ):
                    segmentos_plan = plan.get("segmentos", [])
                    siguiente_idx = None
                    for idx_tmp in range(int(seg_idx or 0) + 1, len(segmentos_plan)):
                        seg_tmp = segmentos_plan[idx_tmp]
                        key_tmp = (
                            vid,
                            str(seg_tmp.get("tipo_plan", "SCHEDULING") or "SCHEDULING"),
                            int(seg_tmp.get("cycle_id", 0) or 0),
                            int(seg_tmp.get("seg_num", 0) or 0),
                        )
                        if key_tmp not in cerrados:
                            siguiente_idx = idx_tmp
                            break

                    if siguiente_idx is not None:
                        seg_idx = siguiente_idx
                        seg = segmentos_plan[siguiente_idx]
                        route_index = max(
                            int(route_index),
                            int(seg.get("start_idx", route_index) or route_index),
                        )
                        segmento_ajustado_al_siguiente = True

            if seg is None:
                op_plan = plan.get("index_to_operacion", {}).get(route_index, "")
                seg_num = -1
                ruta = ""
                edge_fin = ""
                jade_inicio_s = 0.0
                jade_fin_s = 0.0
                distancia_restante = 999999.0
            else:
                op_plan = str(seg.get("operacion", "")).upper()
                seg_num = int(seg.get("seg_num", -1))
                ruta = str(seg.get("ruta", ""))
                edge_fin = str(seg.get("edge_fin", ""))
                jade_inicio_s = float(seg.get("depart_ms", 0)) / 1000.0
                jade_fin_s = float(seg.get("arrival_ms", 0)) / 1000.0
                distancia_restante = distancia_segmento(traci, vid, plan, seg, route_index, lane_actual)

            if descarga_ya_finalizada_en_wait:
                op_est = "DESCARGA_FINALIZADA"
            elif en_stop and op_plan == "VIAJE_VACIO":
                op_est = "CARGA"
            elif en_stop and op_plan == "VIAJE_CARGADO":
                op_est = "DESCARGA"
            else:
                op_est = op_plan

            if descarga_ya_finalizada_en_wait and seg is not None:
                accion = "DISPONIBLE_TRAS_DESCARGA_COMPLETADA"
                disponibilidad = "INMEDIATA"
                ubicacion_disponible = destino_logico_segmento(seg)
                edge_disponible = str(seg.get("edge_fin", "") or edge_actual)
                tiempo_disponible_s = float(sim_time)
                commitment_restante_s = 0.0
                aplicar_desde = "AHORA"
            else:
                accion, disponibilidad = accion_estado(
                    op_est, op_plan, distancia_restante, args
                )
                ubicacion_disponible, edge_disponible, tiempo_disponible_s, commitment_restante_s, aplicar_desde = estado_rescheduling(
                    plan, seg_idx if seg_idx is not None else -1, seg, op_est, op_plan, edge_actual, sim_time, distancia_restante
                )

            # Una averia de camion tiene prioridad sobre el rescheduling. Mientras
            # el camion va a PA_SAFE o se repara, no se ofrece como disponible.
            breakdown = None
            if dynamic_manager is not None:
                try:
                    breakdown = dynamic_manager.get_truck_breakdown(vid)
                except Exception:
                    breakdown = None

            if breakdown:
                repair_end_s = breakdown.get("repair_end_s")
                compromiso_operacional_s = max(
                    0.0,
                    float(commitment_restante_s),
                )

                op_est = "AVERIA_CAMION"
                accion = "ESPERAR_REPARACION"
                disponibilidad = "DESPUES_DESCARGA"

                # PA_SAFE es solamente una detencion temporal para reparar el
                # camion. No reemplaza el destino operacional calculado arriba:
                # el camion sigue ocupado hasta completar su ciclo y descarga.
                observacion_averia = (
                    "averia_activa_pa_safe_temporal_"
                    "compromiso_hasta_descarga"
                )

                if repair_end_s is None:
                    # Todavia no llega al PA_SAFE: el instante real de reparacion
                    # es desconocido y no debe inventarse para JADE.
                    tiempo_disponible_s = HORIZONTE_TURNO_S + 1.0
                    commitment_restante_s = max(
                        0.0,
                        tiempo_disponible_s - float(sim_time),
                    )
                else:
                    # Despues de repararse todavia debe completar el compromiso
                    # operacional que ya tenia asignado. Por eso la disponibilidad
                    # no corresponde al fin de la reparacion ni a PA_SAFE.
                    tiempo_disponible_s = (
                        float(repair_end_s)
                        + compromiso_operacional_s
                    )
                    commitment_restante_s = max(
                        0.0,
                        tiempo_disponible_s - float(sim_time),
                    )
            ubicacion_ref = ubicacion_disponible
            observacion = "foto_rescheduling_global"
            if descarga_ya_finalizada_en_wait:
                observacion = "segmento_cerrado_descarga_completada_en_pa_wait"
            elif segmento_ajustado_al_siguiente:
                observacion = "route_index_atrasado_ajustado_al_siguiente_segmento_abierto"
            elif op_plan == "VIAJE_VACIO":
                observacion = "viaje_vacio_continua_a_pala_carga_botadero_descarga"
            elif op_est == "CARGA":
                observacion = "carga_continua_viaje_cargado_descarga"
            elif op_plan == "VIAJE_CARGADO":
                observacion = "viaje_cargado_continua_a_botadero_descarga"
            elif op_est == "DESCARGA":
                observacion = "descarga_termina_y_queda_disponible"
            if breakdown:
                observacion = observacion_averia

            snapshot.append({
                "tiempo_sumo_s": round(float(sim_time), 3),
                "camion": vid,
                "operacion_planificada": op_plan,
                "operacion_estimada": op_est,
                "seg_num": seg_num,
                "ruta": ruta,
                "edge_actual": edge_actual,
                "lane_actual": lane_actual,
                "pos_actual": round(float(pos_actual), 2),
                "velocidad_actual_s": round(float(velocidad_actual_s), 2),
                "route_index": route_index_sumo,
                "distancia_restante_segmento_m": round(float(distancia_restante), 2),
                "edge_fin_segmento": edge_fin,
                "jade_inicio_s": round(float(jade_inicio_s), 3),
                "jade_fin_s": round(float(jade_fin_s), 3),
                "accion_rescheduling": accion,
                "disponibilidad": disponibilidad,
                "ubicacion_referencia": ubicacion_ref,
                "ubicacion_disponible": ubicacion_disponible,
                "edge_disponible": edge_disponible,
                "tiempo_disponible_estimado_s": round(float(tiempo_disponible_s), 3),
                "aplicar_desde": aplicar_desde,
                "commitment_restante_s": round(float(commitment_restante_s), 3),
                "observacion": observacion,
            })
        return snapshot

    def escribir_snapshot_flota(snapshot, sim_time, disponibilidad_por_pala=None):
        if disponibilidad_por_pala is None:
            disponibilidad_por_pala = {}
        snapshot_path.parent.mkdir(parents=True, exist_ok=True)
        with open(snapshot_path, "w", newline="", encoding="utf-8-sig") as sf:
            sw = csv.DictWriter(sf, fieldnames=snapshot_fieldnames)
            sw.writeheader()
            for row in snapshot:
                sw.writerow({c: row.get(c, "") for c in snapshot_fieldnames})
        with open(snapshot_json_path, "w", encoding="utf-8") as jf:
            json.dump({
                "ok": True,
                "time": float(sim_time),
                "snapshot_flota": snapshot,
                "disponibilidad_por_pala": disponibilidad_por_pala,
                "cantidad_palas": len(disponibilidad_por_pala),
            }, jf, ensure_ascii=False, indent=2)

    def escribir_snapshot(snapshot, sim_time):
        """
        Guarda fotografías periódicas para análisis. No se envían a JADE y no
        modifican el snapshot oficial usado por el rescheduling.
        """
        snapshot_monitoreo_path.parent.mkdir(parents=True, exist_ok=True)
        with open(snapshot_monitoreo_path, "a", newline="", encoding="utf-8-sig") as mf:
            mw = csv.DictWriter(mf, fieldnames=snapshot_fieldnames)
            for row in snapshot:
                mw.writerow({c: row.get(c, "") for c in snapshot_fieldnames})

        with open(snapshot_monitoreo_json_path, "w", encoding="utf-8") as jf:
            json.dump({
                "ok": True,
                "tipo": "MONITOREO_PERIODICO",
                "intervalo_s": SNAPSHOT_MONITOREO_INTERVAL_S,
                "time": float(sim_time),
                "snapshot_flota": snapshot,
                "nota": "Este snapshot es solo monitoreo; JADE no lo usa para cambiar el schedule."
            }, jf, ensure_ascii=False, indent=2)

    zmq_context = None
    zmq_socket = None

    def conectar_server():
        """Conecta una vez al server.py para registrar solicitudes en tiempo casi real."""
        nonlocal zmq_context, zmq_socket
        if args.no_register_server:
            return None
        if zmq_socket is not None:
            return zmq_socket
        try:
            import zmq
            zmq_context = zmq.Context()
            zmq_socket = zmq_context.socket(zmq.REQ)
            zmq_socket.setsockopt(zmq.RCVTIMEO, SERVER_ZMQ_TIMEOUT_MS)
            zmq_socket.setsockopt(zmq.SNDTIMEO, SERVER_ZMQ_TIMEOUT_MS)
            zmq_socket.connect(args.server_endpoint)
            print(f"[rescheduling] conectado a server.py: {args.server_endpoint}")
            return zmq_socket
        except Exception as e:
            print(f"[rescheduling] WARN: no se pudo conectar a server.py: {e}")
            try:
                if zmq_socket is not None:
                    zmq_socket.close(0)
            except Exception:
                pass
            try:
                if zmq_context is not None:
                    zmq_context.term()
            except Exception:
                pass
            zmq_context = None
            zmq_socket = None
            return None

    def enviar_rescheduling(payload):
        """Envia REGISTER_RESCHEDULE_REQUEST al server.py. Si falla, mantiene el CSV local."""
        nonlocal zmq_socket, zmq_context
        sock = conectar_server()
        if sock is None:
            return {"ok": False, "error": "SIN_CONEXION_SERVER"}
        try:
            sock.send_string(json.dumps(payload, ensure_ascii=False))
            resp = sock.recv_string()
            try:
                data = json.loads(resp)
            except Exception:
                data = {"ok": False, "raw": resp}
            if data.get("ok"):
                accion = data.get("accion", "")
                print(f"[rescheduling] server.py registro {payload.get('camion')} accion={accion}")
            else:
                print(f"[rescheduling] WARN server.py no registro solicitud: {data}")
            return data
        except Exception as e:
            print(f"[rescheduling] WARN enviando REGISTER_RESCHEDULE_REQUEST: {e}")
            # El patrón REQ/REP queda desincronizado después de timeout/error; recreamos el socket.
            try:
                if zmq_socket is not None:
                    zmq_socket.close(0)
            except Exception:
                pass
            try:
                if zmq_context is not None:
                    zmq_context.term()
            except Exception:
                pass
            zmq_socket = None
            zmq_context = None
            return {"ok": False, "error": str(e)}

    def enviar_json_server(payload):
        """Envia cualquier comando REQ/REP al server y recrea el socket si falla."""
        nonlocal zmq_socket, zmq_context
        sock = conectar_server()
        if sock is None:
            return {"ok": False, "error": "SIN_CONEXION_SERVER"}
        try:
            sock.send_string(json.dumps(payload, ensure_ascii=False))
            raw = sock.recv_string()
            return json.loads(raw)
        except Exception as e:
            try:
                if zmq_socket is not None:
                    zmq_socket.close(0)
            except Exception:
                pass
            try:
                if zmq_context is not None:
                    zmq_context.term()
            except Exception:
                pass
            zmq_socket = None
            zmq_context = None
            return {"ok": False, "error": str(e)}


    def construir_plan(ciclo):
        """Construye un plan incremental compatible con el mismo control por segmentos del scheduling inicial."""
        eventos = list(ciclo.get("events", []))
        camion = str(ciclo.get("truck", ciclo.get("camion", "")) or "")
        cycle_id = int(float(ciclo.get("id", ciclo.get("cycleId", 0)) or 0))
        try:
            secuencia_ciclo = int(float(
                ciclo.get("secuenciaCiclo", ciclo.get("secuencia", ciclo.get("sequence", 0))) or 0
            ))
        except Exception:
            secuencia_ciclo = 0

        edges = []
        stops = []
        segmentos = []
        vacio_ms = 0.1
        cargado_ms = 0.1
        fin_ciclo_s = 0.0
        edge_descarga = ""
        ubicacion_descarga = ""
        parking_espera = ""

        for ev in eventos:
            op = str(ev.get("operacion", "") or "").upper()
            route_edges = [
                x for x in str(ev.get("routeEdges", "") or "").replace(",", " ").split()
                if x
            ]

            if op.startswith("VIAJE") and route_edges:
                inicio_idx = len(edges)
                for edge in route_edges:
                    if edge and (not edges or edges[-1] != edge):
                        edges.append(edge)
                fin_idx = max(inicio_idx, len(edges) - 1)
                segmentos.append({
                    "operacion": op,
                    "origenRuta": str(ev.get("origenRuta", "") or ""),
                    "destinoRuta": str(ev.get("destinoRuta", ev.get("ubicacion", "")) or ""),
                    "ubicacion": str(ev.get("ubicacion", "") or ""),
                    "start_idx": inicio_idx,
                    "end_idx": fin_idx,
                    "edge_fin": route_edges[-1],
                    "depart_ms": int(float(ev.get("horaInicio", 0) or 0)),
                    "arrival_ms": int(float(ev.get("horaFin", 0) or 0)),
                })

            try:
                if op == "VIAJE_VACIO":
                    vacio_ms = max(
                        vacio_ms,
                        float(str(ev.get("velocidadVacio", "0")).replace(",", ".")) / 3.6,
                    )
                elif op == "VIAJE_CARGADO":
                    cargado_ms = max(
                        cargado_ms,
                        float(str(ev.get("velocidadCargado", "0")).replace(",", ".")) / 3.6,
                    )
            except Exception:
                pass

            if op in ("CARGA", "DESCARGA"):
                try:
                    dur_s = max(
                        0.0,
                        (float(ev.get("horaFin", 0)) - float(ev.get("horaInicio", 0))) / 1000.0,
                    )
                except Exception:
                    dur_s = 0.0

                lane = str(ev.get("destinoLane", "") or "")
                edge_stop = str(ev.get("destinoEdge", "") or "")
                ubicacion_stop = str(ev.get("ubicacion", "") or "")
                parking_op = str(ev.get("parkingArea", "") or "")
                if not parking_op and lane:
                    parking_op = (
                        f"PA_AUTO_{id_xml_seguro(ubicacion_stop)}_"
                        f"{id_xml_seguro(lane)}"
                    )
                stop = {
                    "operacion": op,
                    "ubicacion": ubicacion_stop,
                    "duration_s": dur_s,
                    "parkingArea": parking_op,
                    "lane": lane,
                    "edge": edge_stop,
                    "endPos": float(ev.get("endPos", -1.0) or -1.0),
                }
                if dur_s > 0:
                    stops.append(stop)

                if op == "DESCARGA":
                    edge_descarga = edge_stop
                    ubicacion_descarga = stop["ubicacion"]
                    if lane:
                        parking_espera = (
                            f"PA_WAIT_{id_xml_seguro(ubicacion_descarga)}_"
                            f"{id_xml_seguro(lane)}"
                        )

            try:
                fin_ciclo_s = max(fin_ciclo_s, float(ev.get("horaFin", 0)) / 1000.0)
            except Exception:
                pass

        if not edge_descarga and edges:
            edge_descarga = edges[-1]

        duracion_descarga_s = 0.0
        for stop_tmp in stops:
            if str(stop_tmp.get("operacion", "") or "").upper() == "DESCARGA":
                duracion_descarga_s = max(
                    duracion_descarga_s,
                    float(stop_tmp.get("duration_s", 0.0) or 0.0),
                )

        # IMPORTANTE: los ciclos incrementales deben construir los mismos mapas
        # que usa el scheduling inicial. Sin estos mapas el runner hace continue
        # y el reporte_segmentos_sumo.csv no puede observar el rescheduling.
        index_to_speed = {}
        index_to_operacion = {}
        index_to_segment = {}

        for nseg, seg in enumerate(segmentos, start=1):
            ini = int(seg.get("start_idx", 0) or 0)
            fin = int(seg.get("end_idx", ini) or ini)
            ini = max(0, min(ini, max(0, len(edges) - 1))) if edges else 0
            fin = max(ini, min(fin, max(0, len(edges) - 1))) if edges else ini

            op = str(seg.get("operacion", "") or "").upper()
            speed = cargado_ms if op == "VIAJE_CARGADO" else vacio_ms
            speed = max(0.1, float(speed))
            edges_seg = edges[ini:fin + 1] if edges else []

            dist_m = 0.0
            for edge_id in edges_seg:
                try:
                    dist_m += float(net.edge_length.get(edge_id, 0.0))
                except Exception:
                    pass

            depart_ms = int(seg.get("depart_ms", 0) or 0)
            arrival_ms = int(seg.get("arrival_ms", 0) or 0)
            seg.update({
                "seg_num": nseg,
                "param_key": f"reschedule_{cycle_id}_{nseg}",
                "ruta": f"{seg.get('origenRuta','')}->{seg.get('destinoRuta','')}",
                "edge_inicio": edges_seg[0] if edges_seg else str(seg.get("origenRuta", "") or ""),
                "edge_fin": edges_seg[-1] if edges_seg else str(seg.get("edge_fin", "") or ""),
                "edges_count": len(edges_seg),
                "speed_ms": speed,
                "speed_ref_ms": speed,
                "reachable": True,
                "dur_ms": max(0, arrival_ms - depart_ms),
                "dist_m": dist_m,
                "tipo_plan": "RESCHEDULING",
                "cycle_id": cycle_id,
                "secuencia_ciclo": secuencia_ciclo,
            })

            for i in range(ini, fin + 1):
                index_to_speed[i] = speed
                index_to_operacion[i] = op
                index_to_segment[i] = nseg - 1

        # Cualquier edge no asociado explícitamente conserva velocidad vacía como respaldo,
        # igual que cargar_plan_segmentos() para el scheduling inicial.
        for i in range(len(edges)):
            index_to_speed.setdefault(i, max(0.1, vacio_ms))
            index_to_operacion.setdefault(i, "VIAJE_VACIO")

        plan = {
            "id": camion,
            "camion": camion,
            "edges": edges,
            "stops": stops,
            "vacio_ms": max(0.1, vacio_ms),
            "cargado_ms": max(0.1, cargado_ms),
            "segmentos": segmentos,
            "index_to_speed": index_to_speed,
            "index_to_operacion": index_to_operacion,
            "index_to_segment": index_to_segment,
            "fin_ciclo_s": fin_ciclo_s,
            "edge_descarga": edge_descarga,
            "ubicacion_descarga": ubicacion_descarga,
            "parking_espera": parking_espera,
            "duracion_descarga_s": duracion_descarga_s,
            "tipo_plan": "RESCHEDULING",
            "cycle_id": cycle_id,
            "secuencia_ciclo": secuencia_ciclo,
        }
        return camion, plan

    def _poner_stop_incremental(veh_id, stop):
        dur = float(stop.get("duration_s", 0.0) or 0.0)
        if dur <= 0:
            return True, ""
        parking = str(stop.get("parkingArea", "") or "")
        lane = str(stop.get("lane", "") or "")
        try:
            if parking:
                traci.vehicle.setParkingAreaStop(veh_id, parking, duration=dur)
            elif lane:
                edge_stop = lane.rsplit("_", 1)[0]
                lane_txt = lane.rsplit("_", 1)[1] if "_" in lane else "0"
                lane_idx = int(lane_txt) if lane_txt.isdigit() else 0
                pos = float(stop.get("endPos", -1.0) or -1.0)
                traci.vehicle.setStop(
                    veh_id, edge_stop, pos=pos, laneIndex=lane_idx, duration=dur
                )
            return True, ""
        except Exception as e:
            return False, str(e)

    def _poner_hold_incremental(veh_id, edge_hold, parking_hold=""):
        edge_hold = str(edge_hold or "").strip()
        parking_hold = str(parking_hold or "").strip()
        if not edge_hold or edge_hold.startswith(":"):
            return True, ""
        try:
            # Preferencia: PA_WAIT fuera de la calzada. Evita congelar el camión
            # a 1 m de la junction, que era una fuente de bloqueo/teleport.
            if parking_hold:
                traci.vehicle.setParkingAreaStop(
                    veh_id, parking_hold, duration=1000000000.0
                )
                return True, ""

            # Compatibilidad con archivos antiguos sin PA_WAIT.
            lane_hold = edge_hold + "_0"
            lane_len = float(traci.lane.getLength(lane_hold))
            pos_hold = max(1.0, lane_len * 0.50)
            traci.vehicle.setStop(
                veh_id,
                edge_hold,
                pos=pos_hold,
                laneIndex=0,
                duration=1000000000.0,
                flags=1,
            )
            return True, ""
        except Exception as e:
            return False, str(e)

    def habilitar_final(veh_id):
    

        plan = planes.get(veh_id, {})

        if not plan.get("_ciclo_cerrado"):
            return True

        if plan.get("_final_diferido_aplicado"):
            return True

        origen_ciclo = str(
            plan.get("_origen_ciclo_edge", "") or ""
        ).strip()

        if not origen_ciclo:
            return False

        try:
            edge_actual = str(
                traci.vehicle.getRoadID(veh_id) or ""
            ).strip()
        except Exception:
            return False

        # Mientras siga en el botadero de origen no se puede instalar
        # el stop final porque origen y destino utilizan el mismo edge.
        if not edge_actual or edge_actual.startswith(":"):
            return False

        if edge_actual == origen_ciclo:
            return False

        # ------------------------------------------------------------
        # A partir de aquí el camion YA abandono físicamente el origen.
        # El regreso al mismo edge ya no es ambiguo.
        # ------------------------------------------------------------

        def parking_programado(parking_id):
            parking_id = str(parking_id or "").strip()

            if not parking_id:
                return False

            try:
                for stop_data in traci.vehicle.getStops(veh_id):
                    stopping_id = str(
                        getattr(
                            stop_data,
                            "stoppingPlaceID",
                            ""
                        ) or ""
                    ).strip()

                    if stopping_id == parking_id:
                        return True
            except Exception:
                pass

            return False

        # ------------------------------------------------------------
        # 1. Instalar la DESCARGA final
        # ------------------------------------------------------------

        pendientes = plan.get(
            "_stops_finales_diferidos",
            []
        )

        for stop in list(pendientes):

            parking_stop = str(
                stop.get("parkingArea", "") or ""
            ).strip()

            # Evita duplicar el stop si ya fue agregado en un intento previo.
            if parking_stop and parking_programado(parking_stop):
                continue

            ok_stop, error_stop = _poner_stop_incremental(
                veh_id,
                stop
            )

            if not ok_stop:
                print(
                    f"[rescheduling] Stop final diferido aun no aplicable "
                    f"para {veh_id}: {error_stop}"
                )
                return False

        pendientes.clear()

        # ------------------------------------------------------------
        # 2. Instalar el PA_WAIT que mantiene vivo al camion
        # ------------------------------------------------------------

        if plan.get("_hold_final_diferido"):

            parking_hold = str(
                plan.get("parking_espera", "") or ""
            ).strip()

            # No volver a agregarlo si ya está en la cola de stops.
            if not parking_programado(parking_hold):

                ok_hold, error_hold = _poner_hold_incremental(
                    veh_id,
                    plan.get("edge_descarga", ""),
                    parking_hold,
                )

                if not ok_hold:
                    print(
                        f"[rescheduling] Hold final diferido aun no aplicable "
                        f"para {veh_id}: {error_hold}"
                    )
                    return False

            # Confirmar realmente que SUMO conservó el PA_WAIT.
            if parking_hold and not parking_programado(parking_hold):
                print(
                    f"[rescheduling] PA_WAIT final no confirmado para "
                    f"{veh_id}: {parking_hold}"
                )
                return False

        plan["_final_diferido_aplicado"] = True

        print(
            f"[rescheduling] {veh_id} abandono el origen {origen_ciclo}; "
            "DESCARGA final y PA_WAIT quedaron asegurados."
        )

        return True

    def aplicar_plan(nuevos_planes, version, sim_time):
        """Reemplaza rutas pendientes de vehículos vivos y reactiva los congelados."""
        aplicados = []
        errores = []
        vivos_actuales = set(traci.vehicle.getIDList())
        for veh_id, nuevo in nuevos_planes.items():
            if veh_id not in vivos_actuales:
                errores.append(f"{veh_id}:NO_EXISTE_EN_SUMO")
                continue

            # Nunca eliminar PA_SAFE ni reemplazar la ruta mientras E3 siga
            # activo. El ciclo permanece en la cola y se reintenta al repararse.
            if dynamic_manager is not None:
                try:
                    breakdown = dynamic_manager.get_truck_breakdown(veh_id)
                except Exception:
                    breakdown = None
                if breakdown:
                    errores.append(
                        f"{veh_id}:AVERIA_ACTIVA:"
                        f"{breakdown.get('parking_id', '')}"
                    )
                    continue
            try:
                edge_actual = traci.vehicle.getRoadID(veh_id)
                edges_nuevos = list(nuevo.get("edges", []))
                if not edges_nuevos:
                    errores.append(f"{veh_id}:PLAN_SIN_EDGES")
                    continue

                # El server calcula el ciclo desde el edge real informado por TRUCK_FREE.
                # Nunca se concatena manualmente un edge desconectado.
                if not edge_actual or edge_actual.startswith(":"):
                    errores.append(f"{veh_id}:EDGE_ACTUAL_INVALIDO:{edge_actual}")
                    continue
                if edge_actual not in edges_nuevos:
                    errores.append(
                        f"{veh_id}:ORIGEN_CICLO_NO_COINCIDE:"
                        f"{edge_actual}->{edges_nuevos[0]}"
                    )
                    continue
                idx = edges_nuevos.index(edge_actual)
                ruta_aplicar = edges_nuevos[idx:]

                # Evita duplicados consecutivos antes de setRoute y conserva
                # la correspondencia posicion-ruta -> indice del plan JADE.
                ruta_limpia = []
                ruta_plan_indices = []
                for plan_idx, edge in enumerate(ruta_aplicar, start=idx):
                    if edge and (not ruta_limpia or ruta_limpia[-1] != edge):
                        ruta_limpia.append(edge)
                        ruta_plan_indices.append(int(plan_idx))


                # ============================================================
                # LIBERACION SEGURA DEL PA_WAIT / STOP DEL CICLO ANTERIOR
                # ============================================================

                try:
                    stops_actuales = list(
                        traci.vehicle.getStops(veh_id)
                    )
                except Exception:
                    stops_actuales = []

                # Si todavía existen stops del plan anterior, eliminarlos.
                if stops_actuales:
                    try:
                        for _ in range(256):
                            stops_restantes = list(
                                traci.vehicle.getStops(veh_id)
                            )

                            if not stops_restantes:
                                break

                            traci.vehicle.replaceStop(
                                veh_id,
                                0,
                                "",
                                0.0,
                                0,
                                0.0,
                            )

                    except Exception as e:
                        errores.append(
                            f"{veh_id}:LIMPIEZA_STOP:{e}"
                        )
                        continue


                # Si el camion sigue detenido en parking/stop,
                # solicitar resume pero NO cambiar la ruta en este mismo step.
                try:
                    sigue_detenido = esta_en_stop_o_parking(
                        traci,
                        veh_id,
                    )
                except Exception:
                    sigue_detenido = False


                if sigue_detenido:
                    try:
                        traci.vehicle.resume(veh_id)

                        print(
                            f"[rescheduling] {veh_id} liberando stop anterior; "
                            f"cycle_id={version}. "
                            "La ruta se aplicara en el siguiente step."
                        )

                        errores.append(
                            f"{veh_id}:ESPERANDO_LIBERACION_STOP"
                        )

                        continue

                    except Exception as e:
                        errores.append(
                            f"{veh_id}:RESUME_STOP:{e}"
                        )
                        continue


                # ============================================================
                # EL CAMION YA ESTA LIBRE: APLICAR NUEVA RUTA
                # ============================================================

                try:
                    traci.vehicle.setRoute(
                        veh_id,
                        ruta_limpia,
                    )
                except Exception as e:
                    errores.append(
                        f"{veh_id}:SET_ROUTE:{e}"
                    )
                    continue

                
                try:
                    route_index_base_sumo = int(traci.vehicle.getRouteIndex(veh_id))
                except Exception:
                    route_index_base_sumo = 0
                nuevo["_route_index_base_sumo"] = int(route_index_base_sumo)
                nuevo["_ruta_aplicada_edges"] = list(ruta_limpia)
                nuevo["_ruta_aplicada_plan_indices"] = list(ruta_plan_indices)
                nuevo["_ruta_pos_ultima"] = 0
                nuevo["_route_index_resync_reportado"] = False
                idx_plan_base = ruta_plan_indices[0] if ruta_plan_indices else 0
                print(
                    f"[rescheduling] Base de ruta {veh_id}: "
                    f"idx_sumo={route_index_base_sumo} idx_plan={idx_plan_base} "
                    f"edge={edge_actual} cycle_id={version}"
                )

                edge_hold = str(nuevo.get("edge_descarga", "") or "").strip()
                ciclo_cerrado = bool(
                    len(ruta_limpia) > 1
                    and edge_hold
                    and str(edge_actual) == edge_hold
                    and ruta_limpia[0] == str(edge_actual)
                    and ruta_limpia[-1] == edge_hold
                )
                nuevo["_ciclo_cerrado"] = ciclo_cerrado
                nuevo["_origen_ciclo_edge"] = str(edge_actual or "")
                nuevo["_edge_carga_ciclo_cerrado"] = ""
                nuevo["_stops_finales_diferidos"] = []
                nuevo["_hold_final_diferido"] = False
                nuevo["_final_diferido_aplicado"] = not ciclo_cerrado

                # Se conservan los stops originales. En un ciclo cerrado se instala
                # la CARGA normalmente, pero la DESCARGA final se difiere hasta que
                # el camion llegue fisicamente a esa CARGA. Esto evita que el stop/hold
                # del botadero final capture al camion en el botadero de origen.
                for stop in nuevo.get("stops", []):
                    op_stop = str(stop.get("operacion", "") or "").upper()
                    edge_stop = str(stop.get("edge", "") or "").strip()
                    if not edge_stop:
                        lane_stop = str(stop.get("lane", "") or "").strip()
                        if lane_stop and "_" in lane_stop:
                            edge_stop = lane_stop.rsplit("_", 1)[0]

                    if ciclo_cerrado and op_stop == "CARGA" and edge_stop:
                        nuevo["_edge_carga_ciclo_cerrado"] = edge_stop

                    es_descarga_final_ambigua = bool(
                        ciclo_cerrado
                        and op_stop == "DESCARGA"
                        and (not edge_stop or edge_stop == edge_hold)
                    )
                    if es_descarga_final_ambigua:
                        nuevo["_stops_finales_diferidos"].append(dict(stop))
                        continue

                    ok_stop, error_stop = _poner_stop_incremental(veh_id, stop)
                    if not ok_stop:
                        errores.append(f"{veh_id}:STOP:{error_stop}")

                # Mantiene vivo al MISMO vehículo después de la descarga.
                # En ciclo cerrado el hold también se difiere para no capturar el origen.
                if ciclo_cerrado:
                    nuevo["_hold_final_diferido"] = bool(
                        edge_hold and not edge_hold.startswith(":")
                    )
                else:
                    ok_hold, error_hold = _poner_hold_incremental(
                        veh_id, edge_hold, nuevo.get("parking_espera", "")
                    )
                    if not ok_hold:
                        errores.append(f"{veh_id}:STOP_CONSERVACION:{error_hold}")

                traci.vehicle.setSpeed(veh_id, -1)
                traci.vehicle.setMaxSpeed(
                    veh_id,
                    max(nuevo.get("vacio_ms", 0.1), nuevo.get("cargado_ms", 0.1)),
                )
                planes[veh_id] = nuevo
                camiones_disponibles.pop(veh_id, None)
                descarga_final_inicio_observada.pop(veh_id, None)
                congelados.discard(veh_id)
                congelar_despues_s.pop(veh_id, None)
                controlados.discard(veh_id)
                activos.pop(veh_id, None)
                aplicados.append(veh_id)
            except Exception as e:
                errores.append(f"{veh_id}:{e}")

        # Un ciclo solo puede considerarse APLICADO si ruta y TODOS sus stops
        # quedaron instalados correctamente. Un parkingArea desconocido no es
        # una advertencia: deja el ciclo físicamente incompleto en SUMO.
        ok = bool(aplicados) and not errores
        print(f"[rescheduling] Plan version={version} aplicado={len(aplicados)} errores={len(errores)}")
        if errores:
            print("[rescheduling] Detalle aplicación:", "; ".join(errores[:20]))
        return ok, aplicados, errores

    def guardar_rescheduling():
        # Se reescribe completo para mantener una sola solicitud global.
        with open(rescheduling_path, "w", newline="", encoding="utf-8-sig") as rf:
            rw = csv.DictWriter(rf, fieldnames=rescheduling_fieldnames)
            rw.writeheader()
            for key in sorted(solicitudes_rescheduling.keys()):
                rw.writerow(solicitudes_rescheduling[key])

    # El runner mantiene su reporte local. No sobreescribe un archivo existente
    # que pueda contener el historial exportado por server.py.
    if not rescheduling_path.exists():
        guardar_rescheduling()

    def clave_segmento(veh_id, seg):
        return (
            veh_id,
            str(seg.get("tipo_plan", "SCHEDULING") or "SCHEDULING"),
            int(seg.get("cycle_id", 0) or 0),
            int(seg.get("seg_num", 0) or 0),
        )

    def cerrar_segmento(veh_id, sim_time, motivo):
        activo = activos.pop(veh_id, None)
        if not activo:
            return
        seg = activo["seg"]
        key = clave_segmento(veh_id, seg)
        if key in cerrados:
            return
        cerrados.add(key)
        sumo_ini = activo["sumo_inicio_s"]
        sumo_fin = float(sim_time)
        dur_sumo = max(0.0, sumo_fin - sumo_ini)
        dur_jade = float(seg.get("dur_ms", 0)) / 1000.0
        error_relativo = calcular_error_relativo(dur_sumo, dur_jade)

        time_loss_inicio = float(activo.get("time_loss_inicio_s", 0.0) or 0.0)
        time_loss_fin = float(activo.get("time_loss_ultimo_s", time_loss_inicio) or time_loss_inicio)
        try:
            time_loss_fin = float(traci.vehicle.getTimeLoss(veh_id))
        except Exception:
            pass
        time_loss_segmento = max(0.0, time_loss_fin - time_loss_inicio)

        writer.writerow({
            "camion": veh_id,
            "tipo_plan": str(seg.get("tipo_plan", "SCHEDULING") or "SCHEDULING"),
            "cycle_id": int(seg.get("cycle_id", 0) or 0),
            "secuencia_ciclo": int(seg.get("secuencia_ciclo", 0) or 0),
            "seg_num": seg["seg_num"],
            "operacion": seg["operacion"],
            "ruta": seg["ruta"],
            "edge_inicio": seg["edge_inicio"],
            "edge_fin": seg["edge_fin"],
            "idx_inicio": seg["start_idx"],
            "idx_fin": seg["end_idx"],
            "edges_count": seg["edges_count"],
            "dist_m": f"{float(seg.get('dist_m', 0)):.3f}",
            "speed_ref_ms": f"{float(seg.get('speed_ref_ms', 0)):.6f}",
            "speed_compensada_ms": f"{float(seg.get('speed_ms', 0)):.6f}",
            "objetivo_alcanzable": "SI" if seg.get("reachable", True) else "NO",
            "jade_inicio_s": f"{float(seg.get('depart_ms', 0)) / 1000.0:.3f}",
            "jade_fin_s": f"{float(seg.get('arrival_ms', 0)) / 1000.0:.3f}",
            "jade_duracion_s": f"{dur_jade:.3f}",
            "sumo_inicio_s": f"{sumo_ini:.3f}",
            "sumo_fin_s": f"{sumo_fin:.3f}",
            "sumo_duracion_s": f"{dur_sumo:.3f}",
            "diferencia_s": f"{dur_sumo - dur_jade:.3f}",
            "error_relativo_pct": "" if error_relativo is None else f"{error_relativo:.6f}",
            "time_loss_inicio_s": f"{time_loss_inicio:.6f}",
            "time_loss_fin_s": f"{time_loss_fin:.6f}",
            "time_loss_segmento_s": f"{time_loss_segmento:.6f}",
            "cierre_motivo": motivo,
        })
        csv_file.flush()

    def finalizar_vehiculo(veh_id, sim_time, llegada_normal):
        """
        Limpia de forma definitiva el estado de un vehiculo que realmente dejo SUMO.

        llegada_normal=True:
        - SUMO lo reporto en getArrivedIDList();
        - un ciclo activo se marca EJECUTADO.

        llegada_normal=False:
        - la ausencia persistio y no fue identificada como teleport;
        - el ciclo activo se marca ERROR_VEHICULO_TERMINO;
        - los ciclos pendientes se descartan como ERROR_VEHICULO_NO_DISPONIBLE.

        Esta funcion NO debe llamarse para teleports temporales.
        """
        cerrar_segmento(
            veh_id,
            sim_time,
            "vehiculo_termino" if llegada_normal else "vehiculo_ausente_confirmado",
        )

        activo_finalizado = ciclo_activo_por_camion.pop(veh_id, None)
        if activo_finalizado:
            cycle_id_finalizado = int(
                activo_finalizado.get("cycle_id", 0) or 0
            )

            estado_ack = (
                "EJECUTADO"
                if llegada_normal
                else "ERROR_VEHICULO_TERMINO"
            )

            ack_fin = enviar_json_server({
                "type": "ACK_CYCLE",
                "id": cycle_id_finalizado,
                "camion": veh_id,
                "time": float(sim_time),
                "estado": estado_ack,
            })

            if llegada_normal:
                print(
                    f"[rescheduling] Ciclo {cycle_id_finalizado} ejecutado por "
                    f"{veh_id} al finalizar normalmente en SUMO; "
                    f"ACK_EJECUTADO={ack_fin.get('ok')}"
                )
            else:
                print(
                    f"[rescheduling] ADVERTENCIA: ausencia definitiva confirmada "
                    f"para {veh_id} con ciclo activo {cycle_id_finalizado}; "
                    f"ACK_ERROR={ack_fin.get('ok')}"
                )

        # Si el vehiculo realmente dejo SUMO, ya no puede ejecutar ciclos
        # todavía pendientes. No se hace esta limpieza durante un teleport.
        cola_huerfana = ciclos_pendientes_por_camion.pop(veh_id, [])
        if cola_huerfana:
            for item_huerfano in list(cola_huerfana):
                cycle_id_huerfano = int(
                    item_huerfano.get("cycle_id", 0) or 0
                )
                ack_huerfano = enviar_json_server({
                    "type": "ACK_CYCLE",
                    "id": cycle_id_huerfano,
                    "camion": veh_id,
                    "time": float(sim_time),
                    "estado": "ERROR_VEHICULO_NO_DISPONIBLE",
                })
                print(
                    f"[rescheduling] ADVERTENCIA: ciclo pendiente "
                    f"{cycle_id_huerfano} descartado porque {veh_id} "
                    f"ya no existe definitivamente en SUMO; "
                    f"ACK_ERROR={ack_huerfano.get('ok')}"
                )

        congelar_despues_s.pop(veh_id, None)
        disponibilidad_objetivo.pop(veh_id, None)
        descarga_en_curso_observada.pop(veh_id, None)
        descarga_final_inicio_observada.pop(veh_id, None)
        congelados.discard(veh_id)
        esperas_hora_reportadas.discard(veh_id)
        vehiculos_ausentes_desde.pop(veh_id, None)
        vehiculos_en_teleport.discard(veh_id)

    def retirar_camion(veh_id, sim_time):
        """Retira un camion ya descargado solo cuando su plan esta cerrado."""
        if veh_id in camiones_retirados:
            return True
        plan_cerrado = (
            rescheduling_finalizado if rescheduling_activo
            else float(sim_time) >= HORIZONTE_TURNO_S
        )
        if not plan_cerrado:
            return False
        if ciclo_activo_por_camion.get(veh_id) or ciclos_pendientes_por_camion.get(veh_id):
            return False
        # Estos estados solo se asignan tras terminar el compromiso operacional.
        if veh_id not in camiones_disponibles and veh_id not in congelados:
            return False
        try:
            # 2 = NOTIFICATION_ARRIVED: retiro por fin de trabajo, no teleport.
            traci.vehicle.remove(veh_id, reason=2)
        except Exception as exc:
            print(f"[turno] No se pudo retirar {veh_id}: {exc}; se reintentara.")
            return False
        finalizar_vehiculo(veh_id, sim_time, llegada_normal=True)
        camiones_disponibles.pop(veh_id, None)
        camiones_fin_turno.add(veh_id)
        camiones_retirados.add(veh_id)
        print(f"[turno] {veh_id} RETIRO_FIN_PLAN t={sim_time:.1f}s; sin ciclos pendientes.")
        return True

    def actualizar_rescheduling(veh_id, seg, sim_time, retraso_actual_s, retraso_total_flota_s, total_jade_s, umbral_s, edge_actual, lane_actual, pos_actual, velocidad_actual_s, route_index, camiones_con_retraso, snapshot_flota=None, disponibilidad_por_pala=None):
        """Crea/actualiza UNA solicitud de rescheduling global por retraso total de la flota."""
        if snapshot_flota is None:
            snapshot_flota = []
        if disponibilidad_por_pala is None:
            disponibilidad_por_pala = {}
        aplicar_desde = "REPLANIFICACION_GLOBAL_DESDE_TIEMPO_ACTUAL"
        desvio = (float(retraso_total_flota_s) / float(total_jade_s) * 100.0) if float(total_jade_s) > 0 else 0.0
        key_global = "GLOBAL"
        existente = solicitudes_rescheduling.get(key_global)

        # Solo se registra en server.py cuando se crea por primera vez. Luego se actualiza el CSV local.
        respuesta_server = {"ok": False, "accion": "NO_ENVIADO", "error": "SOLICITUD_GLOBAL_YA_EXISTENTE"}
        if not existente:
            payload_server = {
                "type": "REGISTER_RESCHEDULE_REQUEST",
                "camion": "GLOBAL",
                "camion_disparador": veh_id,
                "motivo": "RETRASO_TOTAL_FLOTA_SUPERA_10",
                "retraso_s": float(retraso_actual_s),
                "retraso_total_flota_s": float(retraso_total_flota_s),
                "desvio": float(desvio),
                "total_jade_s": float(total_jade_s),
                "umbral_s": float(umbral_s),
                "time": float(sim_time),
                "segmento": int(seg["seg_num"]),
                "operacion_actual": seg["operacion"],
                "ruta": seg["ruta"],
                "origen": str(seg.get("ruta", "")).split("->", 1)[0] if "->" in str(seg.get("ruta", "")) else "",
                "destino": str(seg.get("ruta", "")).split("->", 1)[1] if "->" in str(seg.get("ruta", "")) else "",
                "edge_actual": edge_actual,
                "lane_actual": lane_actual,
                "pos_actual": float(pos_actual),
                "velocidad_actual_s": float(velocidad_actual_s),
                "route_index": int(route_index),
                "alcance": "GLOBAL_FLOTA",
                "aplicar_desde": aplicar_desde,
                "camiones_con_retraso": camiones_con_retraso,
                "snapshot_tiempo_s": float(sim_time),
                "snapshot_flota": snapshot_flota,
                "disponibilidad_por_pala": disponibilidad_por_pala,
            }
            respuesta_server = enviar_rescheduling(payload_server)

        server_ok = bool(respuesta_server.get("ok")) if not existente else solicitudes_rescheduling[key_global].get("server_ok", "NO") == "SI"
        server_accion = str(respuesta_server.get("accion", "")) if not existente else solicitudes_rescheduling[key_global].get("server_accion", "")
        server_error = str(respuesta_server.get("error", "")) if not existente else solicitudes_rescheduling[key_global].get("server_error", "")

        if not existente:
            solicitudes_rescheduling[key_global] = {
                "tiempo_primera_deteccion_s": f"{float(sim_time):.3f}",
                "tiempo_ultima_deteccion_s": f"{float(sim_time):.3f}",
                "camion": "GLOBAL",
                "camion_disparador": veh_id,
                "primer_segmento": seg["seg_num"],
                "ultimo_segmento": seg["seg_num"],
                "operacion_primera": seg["operacion"],
                "operacion_ultima": seg["operacion"],
                "ruta_primera": seg["ruta"],
                "ruta_ultima": seg["ruta"],
                "retraso_inicial_s": f"{float(retraso_total_flota_s):.3f}",
                "retraso_max_s": f"{float(retraso_total_flota_s):.3f}",
                "retraso_total_flota_s": f"{float(retraso_total_flota_s):.3f}",
                "total_jade_s": f"{float(total_jade_s):.3f}",
                "umbral_s": f"{float(umbral_s):.3f}",
                "cantidad_eventos": 1,
                "camiones_con_retraso": camiones_con_retraso,
                "edge_actual": edge_actual,
                "lane_actual": lane_actual,
                "pos_actual": f"{float(pos_actual):.2f}",
                "velocidad_actual_s": f"{float(velocidad_actual_s):.2f}",
                "route_index_actual": route_index,
                "evento": "RETRASO_TOTAL_FLOTA_SUPERA_10",
                "accion": "RESCHEDULE_GLOBAL_REQUEST",
                "alcance": "GLOBAL_FLOTA",
                "estado": "PENDIENTE",
                "aplicar_desde": aplicar_desde,
                "server_ok": "SI" if server_ok else "NO",
                "server_accion": server_accion,
                "server_error": server_error,
            }
            print(
                f"[rescheduling] REQUEST GLOBAL: retraso_total_flota={retraso_total_flota_s:.1f}s "
                f"umbral={umbral_s:.1f}s disparador={veh_id} alcance=GLOBAL_FLOTA"
            )
        else:
            existente["tiempo_ultima_deteccion_s"] = f"{float(sim_time):.3f}"
            existente["camion_disparador"] = veh_id
            existente["ultimo_segmento"] = seg["seg_num"]
            existente["operacion_ultima"] = seg["operacion"]
            existente["ruta_ultima"] = seg["ruta"]
            existente["cantidad_eventos"] = int(existente.get("cantidad_eventos", 0)) + 1
            existente["camiones_con_retraso"] = camiones_con_retraso
            existente["edge_actual"] = edge_actual
            existente["lane_actual"] = lane_actual
            existente["pos_actual"] = f"{float(pos_actual):.2f}"
            existente["velocidad_actual_s"] = f"{float(velocidad_actual_s):.2f}"
            existente["route_index_actual"] = route_index
            existente["retraso_total_flota_s"] = f"{float(retraso_total_flota_s):.3f}"
            existente["umbral_s"] = f"{float(umbral_s):.3f}"

            retraso_max_anterior = parse_num(existente.get("retraso_max_s", 0.0), 0.0)
            if float(retraso_total_flota_s) > retraso_max_anterior:
                existente["retraso_max_s"] = f"{float(retraso_total_flota_s):.3f}"
                existente["total_jade_s"] = f"{float(total_jade_s):.3f}"

        guardar_rescheduling()

    def registrar_retraso(veh_id, seg, sim_time, edge_actual, lane_actual, pos_actual, velocidad_actual_s, route_index):
        """
        Registra el retraso actual por camión en memoria de este step.
        La decisión global de rescheduling se evalúa aparte, cada 10 segundos,
        para evitar revisar/actualizar el snapshot en cada paso de SUMO.
        """
        jade_fin_s = float(seg.get("arrival_ms", 0)) / 1000.0
        retraso_actual_s = max(0.0, float(sim_time) - jade_fin_s)

        retrasos_flota_actual[veh_id] = retraso_actual_s
        contexto_retraso_actual[veh_id] = {
            "veh_id": veh_id,
            "seg": seg,
            "sim_time": float(sim_time),
            "edge_actual": edge_actual,
            "lane_actual": lane_actual,
            "pos_actual": float(pos_actual),
            "velocidad_actual_s": float(velocidad_actual_s),
            "route_index": int(route_index),
            "retraso_actual_s": float(retraso_actual_s),
        }

    def evaluar_rescheduling(traci, sim_time):
        """
        Evalúa el umbral global solo cada 10 segundos.
        Si se supera el 10% por primera vez:
        - congela snapshot oficial de rescheduling;
        - calcula disponibilidad de camiones y palas;
        - crea una única solicitud global.
        Si el rescheduling ya está activo, no actualiza la foto oficial.
        """
        nonlocal rescheduling_activo, rescheduling_finalizado, congelar_despues_s, disponibilidad_objetivo, proxima_evaluacion_rescheduling_s

        if float(sim_time) + 1e-9 < float(proxima_evaluacion_rescheduling_s):
            return

        while float(proxima_evaluacion_rescheduling_s) <= float(sim_time) + 1e-9:
            proxima_evaluacion_rescheduling_s += SNAPSHOT_MONITOREO_INTERVAL_S

        if rescheduling_activo:
            return

        key = ("GLOBAL", "RETRASO_TOTAL_FLOTA_SUPERA_10")
        if key in eventos_registrados:
            return

        umbral_s = float(umbral_global_s)
        retrasos_positivos = {k: v for k, v in retrasos_flota_actual.items() if float(v) > 0.0}
        retraso_total_flota_s = sum(float(v) for v in retrasos_positivos.values())

        if retraso_total_flota_s <= umbral_s:
            return

        if not retrasos_positivos:
            return

        # Camión disparador referencial: el que tiene mayor retraso positivo en la foto.
        veh_id = max(retrasos_positivos, key=lambda k: float(retrasos_positivos[k]))
        ctx = contexto_retraso_actual.get(veh_id)
        if not ctx:
            return

        seg = ctx["seg"]
        jade_inicio_s = float(seg.get("depart_ms", 0)) / 1000.0
        jade_fin_s = float(seg.get("arrival_ms", 0)) / 1000.0
        jade_duracion_s = float(seg.get("dur_ms", 0)) / 1000.0
        if jade_duracion_s <= 0:
            jade_duracion_s = max(0.0, jade_fin_s - jade_inicio_s)

        edge_actual = ctx["edge_actual"]
        lane_actual = ctx["lane_actual"]
        pos_actual = ctx["pos_actual"]
        velocidad_actual_s = ctx["velocidad_actual_s"]
        route_index = ctx["route_index"]
        retraso_actual_s = ctx["retraso_actual_s"]

        camiones_con_retraso = ";".join(
            f"{k}:{v:.2f}" for k, v in sorted(retrasos_positivos.items())
        )

        eventos_registrados.add(key)
        eventos_writer.writerow({
            "tiempo_sumo_s": f"{float(sim_time):.3f}",
            "camion": "GLOBAL",
            "seg_num": seg["seg_num"],
            "operacion": seg["operacion"],
            "ruta": seg["ruta"],
            "jade_inicio_s": f"{jade_inicio_s:.3f}",
            "jade_fin_s": f"{jade_fin_s:.3f}",
            "jade_duracion_s": f"{jade_duracion_s:.3f}",
            "retraso_s": f"{retraso_total_flota_s:.3f}",
            "total_jade_s": f"{float(total_jade_s):.3f}",
            "umbral_s": f"{umbral_s:.3f}",
            "edge_actual": edge_actual,
            "lane_actual": lane_actual,
            "pos_actual": f"{float(pos_actual):.2f}",
            "velocidad_actual_s": f"{float(velocidad_actual_s):.2f}",
            "route_index": route_index,
            "evento": "RETRASO_TOTAL_FLOTA_SUPERA_10",
            "accion": "RESCHEDULE_GLOBAL_REQUEST",
        })
        eventos_file.flush()

        snapshot_flota = tomar_snapshot_flota(traci, sim_time, planes, args)
        disponibilidad_por_pala = estado_palas(traci, sim_time, planes)

        # Snapshot oficial: este sí queda fijo para JADE.
        escribir_snapshot_flota(snapshot_flota, sim_time, disponibilidad_por_pala)

        rescheduling_activo = True
        rescheduling_finalizado = False
        congelar_despues_s = {
            str(row.get("camion", "")): float(row.get("tiempo_disponible_estimado_s", sim_time) or sim_time)
            for row in snapshot_flota if isinstance(row, dict) and row.get("camion")
        }
        disponibilidad_objetivo = {
            str(row.get("camion", "")): {
                "edge_disponible": str(row.get("edge_disponible", "") or ""),
                "ubicacion_disponible": str(row.get("ubicacion_disponible", "") or ""),
            }
            for row in snapshot_flota if isinstance(row, dict) and row.get("camion")
        }
        descarga_en_curso_observada.clear()

        # En el instante del trigger NO se modifica la cola de stops.
        # Cada camion debe terminar primero el compromiso que estaba ejecutando.
        # El corte del scheduling viejo y la espera larga se instalan de forma
        # atomica cuando descarga_finalizada() confirma ese fin fisico.
        print(f"[rescheduling] Snapshot oficial congelado en t={sim_time:.1f}s. JADE usara esta foto para renegociar.")
        print(f"[rescheduling] Modo rescheduling activo. Camiones se congelan al terminar compromiso actual: {len(congelar_despues_s)}")
        print(f"[rescheduling] Disponibilidad de palas calculada: {len(disponibilidad_por_pala)}")

        actualizar_rescheduling(
            veh_id, seg, sim_time, retraso_actual_s, retraso_total_flota_s,
            total_jade_s, umbral_s, edge_actual, lane_actual, pos_actual,
            velocidad_actual_s, route_index, camiones_con_retraso, snapshot_flota, disponibilidad_por_pala
        )

        print(
            f"[evento] RETRASO_TOTAL_FLOTA_SUPERA_10 disparador={veh_id} "
            f"retraso_total_flota={retraso_total_flota_s:.1f}s "
            f"umbral_global={umbral_s:.1f}s camiones={len(retrasos_positivos)}"
        )


    def asegurar_truck_free(veh_id, sim_time):
        """Envía una sola vez al server el estado real de un camión ya disponible."""
        if veh_id in camiones_disponibilidad_informada:
            return True
        row = camiones_disponibles.get(veh_id)
        if not isinstance(row, dict):
            return False
        ultimo = float(ultimo_intento_truck_free_s.get(veh_id, -999999.0))
        if float(sim_time) - ultimo < 5.0:
            return False
        ultimo_intento_truck_free_s[veh_id] = float(sim_time)
        payload = {
            "type": "TRUCK_FREE",
            "truck": veh_id,
            "time_ms": int(round(float(row.get("tiempo_disponible_s", sim_time)) * 1000.0)),
            "time": float(row.get("tiempo_disponible_s", sim_time)),
            "place": str(row.get("place", "") or ""),
            "edge": str(row.get("edge", "") or ""),
            "lane": str(row.get("lane", "") or ""),
            "pos": float(row.get("pos", 0.0) or 0.0),
            "estado_disponibilidad": str(row.get("estado", "DISPONIBLE") or "DISPONIBLE"),
            "parkingArea": str(row.get("parkingArea", "") or ""),
        }
        resp = enviar_json_server(payload)
        if resp.get("ok"):
            camiones_disponibilidad_informada.add(veh_id)
            print(
                f"[disponibilidad] {veh_id} informado como {row.get('estado')} "
                f"desde t={float(row.get('tiempo_disponible_s', sim_time)):.1f}s "
                f"edge={row.get('edge','')}"
            )
            return True
        return False


    def mantener_espera_segura(veh_id):
        """
        Mantiene al camion disponible sin congelarlo sobre una lane normal.

        Si ya esta dentro de un parkingArea, el propio stop de SUMO lo mantiene
        detenido. Si todavia va camino al PA_WAIT, se libera el control manual
        de velocidad para que pueda llegar fisicamente al parking.
        """
        try:
            if traci.vehicle.isStoppedParking(veh_id):
                return True
        except Exception:
            pass

        try:
            plan_mov = planes.get(veh_id, {})
            vmax = max(
                0.1,
                float(plan_mov.get("vacio_ms", 0.1) or 0.1),
                float(plan_mov.get("cargado_ms", 0.1) or 0.1),
            )
            # -1 devuelve a SUMO el control normal de la velocidad.
            # No usar setSpeed(0) aqui: si el PA_WAIT esta mas adelante,
            # congelar el camion en la lane provoca teleports por jam cada 300 s.
            traci.vehicle.setSpeed(veh_id, -1)
            traci.vehicle.setMaxSpeed(veh_id, vmax)
        except Exception:
            pass
        return False


    def registrar_camion(veh_id, sim_time, estado, tiempo_disponible_s=None, place=""):
        """Mantiene al camión vivo/estacionado y lo habilita para un futuro rescheduling."""
        if veh_id in camiones_disponibles:
            asegurar_truck_free(veh_id, sim_time)
            return camiones_disponibles[veh_id]
        plan = planes.get(veh_id, {})
        try:
            edge = str(traci.vehicle.getRoadID(veh_id) or "")
            lane = str(traci.vehicle.getLaneID(veh_id) or "")
            pos = float(traci.vehicle.getLanePosition(veh_id))
        except Exception:
            edge = str(plan.get("edge_inicial", "") or "")
            lane = str(plan.get("lane_inicial", "") or "")
            pos = float(plan.get("pos_inicial", 0.0) or 0.0)
        if not place:
            place = str(plan.get("ubicacion_inicial", "") or edge)
        if tiempo_disponible_s is None:
            tiempo_disponible_s = float(sim_time)
        row = {
            "estado": str(estado or "DISPONIBLE"),
            "tiempo_disponible_s": max(0.0, float(tiempo_disponible_s)),
            "place": str(place or ""),
            "edge": edge,
            "lane": lane,
            "pos": pos,
            "parkingArea": str(plan.get("parking_espera", "") or ""),
        }
        camiones_disponibles[veh_id] = row
        # No congelar el camion sobre la lane. Si aun no llego al PA_WAIT,
        # debe poder avanzar hasta el parking; una vez estacionado, el stop
        # de parkingArea lo mantiene detenido sin interferir con la via.
        mantener_espera_segura(veh_id)
        asegurar_truck_free(veh_id, sim_time)
        print(
            f"[disponibilidad] {veh_id} -> {row['estado']} "
            f"en {row['edge']} t={row['tiempo_disponible_s']:.1f}s"
        )
        return row


    def agregar_espera(veh_id, edge_objetivo):
        """
        Corta definitivamente el scheduling anterior y deja al camion esperando
        en el PA_WAIT de la descarga donde termino su compromiso actual.

        Esta funcion se llama SOLO despues de que descarga_finalizada()
        confirma fisicamente el fin del compromiso. Por eso ya no se intenta
        editar una cola futura mientras el camion aun esta viajando/cargando.

        Reglas:
        - El camion debe estar fisicamente en edge_objetivo.
        - Se localiza el PA_WAIT de ESE mismo edge.
        - Se eliminan TODOS los stops restantes del scheduling viejo.
        - Se instala un unico PA_WAIT largo.
        - Solo si esa espera queda realmente programada se permite publicar
          TRUCK_FREE o marcar al camion como congelado.
        """
        edge_objetivo = str(edge_objetivo or "").strip()
        if not edge_objetivo or edge_objetivo.startswith(":"):
            return False

        def parking_en_edge(parking_id):
            parking_id = str(parking_id or "").strip()
            if not parking_id:
                return False
            try:
                lane_parking = str(traci.parkingarea.getLaneID(parking_id) or "").strip()
                return (
                    lane_parking == edge_objetivo
                    or lane_parking.startswith(edge_objetivo + "_")
                )
            except Exception:
                return edge_objetivo in parking_id

        try:
            edge_actual = str(traci.vehicle.getRoadID(veh_id) or "").strip()
        except Exception:
            edge_actual = ""
        try:
            lane_actual = str(traci.vehicle.getLaneID(veh_id) or "").strip()
        except Exception:
            lane_actual = ""

        en_objetivo = (
            edge_actual == edge_objetivo
            or lane_actual == edge_objetivo
            or lane_actual.startswith(edge_objetivo + "_")
        )
        if not en_objetivo:
            print(
                f"[rescheduling] Corte pendiente para {veh_id}: "
                f"edge_actual={edge_actual or lane_actual} "
                f"objetivo={edge_objetivo}"
            )
            return False

        plan_hold = planes.get(veh_id, {})
        parking_area = ""

        try:
            for parking_tmp in traci.parkingarea.getIDList():
                parking_tmp = str(parking_tmp or "").strip()
                if (
                    parking_tmp.startswith("PA_WAIT_")
                    and parking_en_edge(parking_tmp)
                ):
                    parking_area = parking_tmp
                    break
        except Exception:
            pass

        if not parking_area:
            for st in plan_hold.get("stops", []):
                parking_tmp = str(st.get("parkingArea", "") or "").strip()
                if (
                    parking_tmp.startswith("PA_WAIT_")
                    and parking_en_edge(parking_tmp)
                ):
                    parking_area = parking_tmp
                    break

        if not parking_area:
            parking_global = str(plan_hold.get("parking_espera", "") or "").strip()
            if (
                parking_global.startswith("PA_WAIT_")
                and parking_en_edge(parking_global)
            ):
                parking_area = parking_global

        if not parking_area:
            print(
                f"[rescheduling] No existe PA_WAIT compatible para {veh_id} "
                f"en {edge_objetivo}; no se publica TRUCK_FREE."
            )
            return False

        # En el handoff inicial el PA_WAIT puede haber quedado preparado mientras
        # el PA_AUTO de descarga seguia activo. Si SUMO ya avanzo a ese PA_WAIT,
        # el corte esta fisicamente completo: no hay que borrar/recrear el stop.
        try:
            stops_actuales = list(traci.vehicle.getStops(veh_id))
            if len(stops_actuales) == 1:
                stopping_id = str(
                    getattr(stops_actuales[0], "stoppingPlaceID", "") or ""
                ).strip()
                if stopping_id == parking_area:
                    print(
                        f"[rescheduling] Corte DEFINITIVO de scheduling para {veh_id}: "
                        f"edge={edge_objetivo}; PA_WAIT={parking_area}; "
                        "handoff_preparado=true"
                    )
                    return True
        except Exception:
            pass

        try:
            eliminados = 0
            for _ in range(256):
                stops_restantes = list(traci.vehicle.getStops(veh_id))
                if not stops_restantes:
                    break
                traci.vehicle.replaceStop(
                    veh_id, 0, "", 0.0, 0, 0.0
                )
                eliminados += 1

            if traci.vehicle.getStops(veh_id):
                print(
                    f"[rescheduling] No se pudo vaciar por completo la cola vieja "
                    f"de {veh_id}; se reintentara."
                )
                return False

            traci.vehicle.setParkingAreaStop(
                veh_id,
                parking_area,
                duration=1000000000.0,
            )

            espera_confirmada = False
            try:
                for stop_data in traci.vehicle.getStops(veh_id):
                    stopping_id = str(
                        getattr(stop_data, "stoppingPlaceID", "") or ""
                    ).strip()
                    if stopping_id == parking_area:
                        espera_confirmada = True
                        break
            except Exception:
                espera_confirmada = False

            if not espera_confirmada:
                print(
                    f"[rescheduling] PA_WAIT no confirmado para {veh_id}: "
                    f"{parking_area}; se reintentara."
                )
                return False

            print(
                f"[rescheduling] Corte DEFINITIVO de scheduling para {veh_id}: "
                f"edge={edge_objetivo}; PA_WAIT={parking_area}; "
                f"stops_viejos_eliminados={eliminados}"
            )
            return True

        except Exception as e:
            print(
                f"[rescheduling] No se pudo fijar corte definitivo para "
                f"{veh_id} en {edge_objetivo}: {e}"
            )
            return False


    def preparar_handoff(veh_id, edge_objetivo, sim_time):
        """
        Prepara el corte scheduling -> rescheduling SIN volver a contar la descarga.

        Se usa solo para el compromiso que el camion traia del scheduling inicial
        cuando se disparo el rescheduling:
        - espera a que el camion este realmente detenido en el PA_AUTO de descarga;
        - conserva ese stop operativo actual;
        - elimina solamente los stops FUTUROS del scheduling viejo;
        - agrega a continuacion un PA_WAIT largo en el mismo edge;
        - devuelve True recien cuando SUMO ya termino la descarga y el camion esta
          efectivamente detenido en ese PA_WAIT.

        De esta forma Truck33/Truck47 no pueden escapar hacia el siguiente ciclo
        antiguo aunque entre la descarga actual y el proximo ciclo no exista una
        espera intermedia suficiente.
        """
        edge_objetivo = str(edge_objetivo or "").strip()
        if not edge_objetivo or edge_objetivo.startswith(":"):
            return False

        def parking_en_edge(parking_id):
            parking_id = str(parking_id or "").strip()
            if not parking_id:
                return False
            try:
                lane_parking = str(traci.parkingarea.getLaneID(parking_id) or "").strip()
                return (
                    lane_parking == edge_objetivo
                    or lane_parking.startswith(edge_objetivo + "_")
                )
            except Exception:
                return edge_objetivo in parking_id

        plan_actual = planes.get(veh_id, {})
        parking_wait = ""

        # Preferir el PA_WAIT fisico del mismo edge donde termina el compromiso.
        try:
            for parking_tmp in traci.parkingarea.getIDList():
                parking_tmp = str(parking_tmp or "").strip()
                if parking_tmp.startswith("PA_WAIT_") and parking_en_edge(parking_tmp):
                    parking_wait = parking_tmp
                    break
        except Exception:
            pass

        if not parking_wait:
            for st in plan_actual.get("stops", []):
                parking_tmp = str(st.get("parkingArea", "") or "").strip()
                if parking_tmp.startswith("PA_WAIT_") and parking_en_edge(parking_tmp):
                    parking_wait = parking_tmp
                    break

        if not parking_wait:
            parking_global = str(plan_actual.get("parking_espera", "") or "").strip()
            if parking_global.startswith("PA_WAIT_") and parking_en_edge(parking_global):
                parking_wait = parking_global

        if not parking_wait:
            print(
                f"[rescheduling] Handoff pendiente para {veh_id}: "
                f"no existe PA_WAIT en {edge_objetivo}."
            )
            return False

        try:
            stops_actuales = list(traci.vehicle.getStops(veh_id))
        except Exception:
            return False
        if not stops_actuales:
            return False

        def stop_parking_id(stop_data):
            return str(getattr(stop_data, "stoppingPlaceID", "") or "").strip()

        primer_parking = stop_parking_id(stops_actuales[0])

        # La descarga ya termino naturalmente y SUMO avanzo al PA_WAIT preparado.
        if primer_parking == parking_wait:
            obs = descarga_en_curso_observada.get(veh_id, {})
            if not isinstance(obs, dict):
                obs = {}
            if not obs.get("handoff_completado_reportado"):
                print(
                    f"[rescheduling] Handoff completado para {veh_id}: "
                    f"descarga terminada y detenido en {parking_wait} t={float(sim_time):.1f}s"
                )
                obs["handoff_completado_reportado"] = True
                descarga_en_curso_observada[veh_id] = obs
            return True

        # Todavia no esta ejecutando la descarga operacional que debemos conservar.
        if not (
            primer_parking.startswith("PA_AUTO_")
            and parking_en_edge(primer_parking)
            and esta_en_stop_o_parking(traci, veh_id)
        ):
            return False

        obs = descarga_en_curso_observada.get(veh_id, {})
        if not isinstance(obs, dict):
            obs = {}

        # Si ya preparamos el corte, NO tocamos nuevamente el stop de descarga.
        # Esperamos a que SUMO lo termine y pase naturalmente al PA_WAIT.
        if obs.get("corte_handoff_preparado") and obs.get("parking_wait") == parking_wait:
            return False

        try:
            eliminados = 0

            # getStops() mantiene el stop actual en la posicion 0. Se conserva ese
            # PA_AUTO de descarga y se eliminan SOLO los stops que vienen despues.
            for _ in range(256):
                stops_tmp = list(traci.vehicle.getStops(veh_id))
                if len(stops_tmp) <= 1:
                    break
                actual_id = stop_parking_id(stops_tmp[0])
                if actual_id != primer_parking:
                    print(
                        f"[rescheduling] Corte handoff abortado para {veh_id}: "
                        f"cambio el stop activo {primer_parking}->{actual_id}."
                    )
                    return False
                traci.vehicle.replaceStop(veh_id, 1, "", 0.0, 0, 0.0)
                eliminados += 1

            stops_tmp = list(traci.vehicle.getStops(veh_id))
            if len(stops_tmp) != 1 or stop_parking_id(stops_tmp[0]) != primer_parking:
                print(
                    f"[rescheduling] No se pudo conservar de forma aislada la descarga "
                    f"actual de {veh_id}; se reintentara."
                )
                return False

            # El PA_WAIT se agrega DESPUES del PA_AUTO actual. No se resume el camion,
            # no se cambia su ruta y no se modifica la duracion de la descarga en curso.
            traci.vehicle.setParkingAreaStop(
                veh_id,
                parking_wait,
                duration=1000000000.0,
            )

            stops_verif = list(traci.vehicle.getStops(veh_id))
            ids_verif = [stop_parking_id(st) for st in stops_verif]
            if len(ids_verif) < 2 or ids_verif[0] != primer_parking or parking_wait not in ids_verif[1:]:
                print(
                    f"[rescheduling] PA_WAIT posterior no confirmado para {veh_id}: "
                    f"actual={primer_parking} wait={parking_wait}; se reintentara."
                )
                return False

            descarga_en_curso_observada[veh_id] = {
                "inicio_s": float(sim_time),
                "edge": edge_objetivo,
                "corte_handoff_preparado": True,
                "parking_descarga": primer_parking,
                "parking_wait": parking_wait,
                "stops_futuros_eliminados": int(eliminados),
            }
            print(
                f"[rescheduling] Corte PREPARADO para {veh_id}: "
                f"conserva_descarga={primer_parking}; "
                f"stops_futuros_eliminados={eliminados}; "
                f"PA_WAIT_siguiente={parking_wait}; t={float(sim_time):.1f}s"
            )
            return False

        except Exception as e:
            print(
                f"[rescheduling] No se pudo preparar handoff de {veh_id} "
                f"en {edge_objetivo}: {e}"
            )
            return False

    
    def parking_actual_vehiculo(veh_id):
    
        try:
            stops = list(traci.vehicle.getStops(veh_id))
        except Exception:
            return ""

        if not stops:
            return ""

        try:
            return str(
                getattr(stops[0], "stoppingPlaceID", "") or ""
            ).strip()
        except Exception:
            return ""

    def descarga_finalizada(veh_id, sim_time):
   
        if veh_id not in congelar_despues_s:
            return False

        objetivo = disponibilidad_objetivo.get(veh_id, {})
        edge_objetivo = str(
            objetivo.get("edge_disponible", "") or ""
        ).strip()

        if not edge_objetivo:
            return False

        try:
            edge_actual = str(
                traci.vehicle.getRoadID(veh_id) or ""
            ).strip()
        except Exception:
            edge_actual = ""

        try:
            lane_actual = str(
                traci.vehicle.getLaneID(veh_id) or ""
            ).strip()
        except Exception:
            lane_actual = ""

        llego_fisicamente = (
            edge_actual == edge_objetivo
            or lane_actual == edge_objetivo
            or lane_actual.startswith(edge_objetivo + "_")
        )

        if not llego_fisicamente:
            return False

        ciclo_activo = ciclo_activo_por_camion.get(veh_id)

        # =========================================================
        # 1. HANDOFF DEL SCHEDULING INICIAL
        # =========================================================
        if not ciclo_activo:
            return preparar_handoff(
                veh_id,
                edge_objetivo,
                sim_time,
            )

        # =========================================================
        # 2. CICLO DE RESCHEDULING
        # =========================================================
        plan_activo = planes.get(veh_id, {})

        # En ciclos cerrados BOTADERO->PALA->MISMO BOTADERO,
        # la descarga final debe haberse habilitado primero.
        if plan_activo.get("_ciclo_cerrado"):
            if not plan_activo.get("_final_diferido_aplicado"):
                return False

        # ---------------------------------------------------------
        # CASO SEGURO:
        # SUMO ya avanzo al siguiente stop PA_WAIT.
        #
        # PA_WAIT se programa DESPUES de la DESCARGA. Por lo tanto,
        # ser el siguiente stop indica que la descarga anterior termino.
        # No debe volver a contarse su duracion.
        # ---------------------------------------------------------
        parking_actual = parking_actual_vehiculo(veh_id)

        if parking_actual.startswith("PA_WAIT_"):
            obs = descarga_en_curso_observada.get(veh_id, {})
            if not isinstance(obs, dict):
                obs = {}

            if not obs.get("fin_por_pa_wait_reportado"):
                print(
                    f"[rescheduling] {veh_id} ciclo finalizado fisicamente: "
                    f"descarga terminada; siguiente stop {parking_actual} en {edge_objetivo} "
                    f"t={float(sim_time):.1f}s"
                )
                obs["fin_por_pa_wait_reportado"] = True
                obs["edge"] = edge_objetivo
                descarga_en_curso_observada[veh_id] = obs

            return True

        # Todavia debe encontrarse realmente detenido para considerar
        # que esta realizando la descarga.
        if not esta_en_stop_o_parking(traci, veh_id):
            return False

        # ---------------------------------------------------------
        # Para los ciclos no cerrados se conserva la comprobacion
        # de que alcanzo el final logico de la ruta.
        # ---------------------------------------------------------
        if not plan_activo.get("_ciclo_cerrado"):
            try:
                idx_sumo = int(
                    traci.vehicle.getRouteIndex(veh_id)
                )

                indice_actual = indice_plan_actual(
                    traci,
                    veh_id,
                    plan_activo,
                    route_index_sumo=idx_sumo,
                    edge_actual=edge_actual,
                )

                mapa_idx = list(
                    plan_activo.get(
                        "_ruta_aplicada_plan_indices",
                        [],
                    ) or []
                )

                ultimo_indice_plan = (
                    int(mapa_idx[-1])
                    if mapa_idx
                    else max(
                        0,
                        len(plan_activo.get("edges", [])) - 1,
                    )
                )

                if (
                    indice_actual is not None
                    and indice_actual < ultimo_indice_plan
                ):
                    return False

            except Exception:
                # No abortar solamente porque SUMO haya reajustado
                # el routeIndex. Ya se comprobo que fisicamente esta
                # en el edge final y detenido.
                pass

        # ---------------------------------------------------------
        # Determinar duracion real de DESCARGA.
        # ---------------------------------------------------------
        duracion_descarga_s = float(
            plan_activo.get(
                "duracion_descarga_s",
                0.0,
            ) or 0.0
        )

        if duracion_descarga_s <= 0:
            for seg_tmp in reversed(
                plan_activo.get("segmentos", [])
            ):
                if (
                    str(
                        seg_tmp.get(
                            "operacion",
                            "",
                        ) or ""
                    ).upper()
                    == "VIAJE_CARGADO"
                ):
                    duracion_descarga_s = float(
                        seg_tmp.get(
                            "stop_after_duration_s",
                            0.0,
                        ) or 0.0
                    )
                    break

        obs = descarga_en_curso_observada.get(veh_id)

        if (
            not isinstance(obs, dict)
            or obs.get("edge") != edge_objetivo
            or obs.get("parking") != parking_actual
        ):
            descarga_en_curso_observada[veh_id] = {
                "inicio_s": float(sim_time),
                "edge": edge_objetivo,
                "parking": parking_actual,
                "duracion_s": max(
                    0.0,
                    duracion_descarga_s,
                ),
            }

            print(
                f"[rescheduling] {veh_id} inicio DESCARGA real "
                f"en {edge_objetivo} "
                f"parking={parking_actual or 'SIN_ID'} "
                f"t={float(sim_time):.1f}s "
                f"duracion={duracion_descarga_s:.1f}s"
            )

            return False

        inicio_s = float(
            obs.get(
                "inicio_s",
                sim_time,
            ) or sim_time
        )

        duracion_s = float(
            obs.get(
                "duracion_s",
                duracion_descarga_s,
            ) or 0.0
        )

        tiempo_descargando_s = max(
            0.0,
            float(sim_time) - inicio_s,
        )

        if (
            tiempo_descargando_s + 1e-9
            < duracion_s
        ):
            return False

        # El temporizador planificado no autoriza a acortar un stop que SUMO
        # todavia mantiene activo (p. ej., por un evento dinamico).
        # En el siguiente step, PA_WAIT como proximo stop confirma que SUMO
        # termino la descarga; no exige entrar fisicamente a ese estacionamiento.
        return False

    def inicio_ciclo_s(plan_ciclo):
        """
        Retorna la hora absoluta JADE del primer VIAJE_VACIO del ciclo.

        El ciclo solo puede activarse cuando:
        - el camion ya termino el ciclo anterior; y
        - el reloj de SUMO alcanzo esta hora planificada.
        """
        inicios = []
        for seg in plan_ciclo.get("segmentos", []):
            if str(seg.get("operacion", "") or "").upper() != "VIAJE_VACIO":
                continue
            try:
                inicios.append(float(seg.get("depart_ms", 0) or 0) / 1000.0)
            except Exception:
                continue
        return min(inicios) if inicios else 0.0


    def activar_ciclo(veh_id, sim_time):
        """
        Estados posibles:
        - SIN_CICLO: no hay un ciclo recibido para el camion.
        - ESPERANDO_HORA: existe un ciclo, pero su horaInicio JADE aun no llega.
        - APLICADO: el ciclo fue activado correctamente.
        - ERROR_APLICACION: SUMO todavia no pudo aplicar la ruta/los stops.
        """
        cola = ciclos_pendientes_por_camion.get(veh_id, [])
        if not cola:
            esperas_hora_reportadas.discard(veh_id)
            return "SIN_CICLO"

        # Primero se consulta el ciclo sin retirarlo de la cola.
        item = cola[0]
        cycle_id = int(item.get("cycle_id", 0))
        plan_ciclo = item.get("plan", {})
        inicio_planificado_s = inicio_ciclo_s(plan_ciclo)

        if float(sim_time) + 1e-9 < float(inicio_planificado_s):
            if veh_id not in esperas_hora_reportadas:
                espera_s = max(0.0, float(inicio_planificado_s) - float(sim_time))
                print(
                    f"[rescheduling] Ciclo {cycle_id} de {veh_id} recibido, "
                    f"pero espera hora JADE={inicio_planificado_s:.1f}s; "
                    f"SUMO={float(sim_time):.1f}s; espera={espera_s:.1f}s"
                )
                esperas_hora_reportadas.add(veh_id)
            return "ESPERANDO_HORA"

        # Solo se retira de la cola cuando corresponde activarlo.
        item = cola.pop(0)
        esperas_hora_reportadas.discard(veh_id)

        ok_aplicacion, _, errores = aplicar_plan(
            {veh_id: plan_ciclo},
            cycle_id,
            sim_time,
        )
        if not ok_aplicacion:
            cola.insert(0, item)
            print(f"[rescheduling] Ciclo {cycle_id} aun no aplicable a {veh_id}: {errores}")
            return "ERROR_APLICACION"

        ciclo_activo_por_camion[veh_id] = item
        congelar_despues_s[veh_id] = float(plan_ciclo.get("fin_ciclo_s", sim_time))
        disponibilidad_objetivo[veh_id] = {
            "edge_disponible": str(plan_ciclo.get("edge_descarga", "") or ""),
            "ubicacion_disponible": str(plan_ciclo.get("ubicacion_descarga", "") or ""),
        }
        descarga_en_curso_observada.pop(veh_id, None)
        ack_aplicado = enviar_json_server({
            "type": "ACK_CYCLE",
            "id": cycle_id,
            "camion": veh_id,
            "time": float(sim_time),
            "estado": "APLICADO",
        })
        print(
            f"[rescheduling] Ciclo {cycle_id} activado para {veh_id} "
            f"en t={float(sim_time):.1f}s (hora JADE={inicio_planificado_s:.1f}s); "
            f"quedan={len(cola)} ACK_APLICADO={ack_aplicado.get('ok')}"
        )
        return "APLICADO"

    def registrar_evento(payload):
        if args.no_register_server:
            return {"ok": False, "error": "REGISTRO_SERVER_DESACTIVADO"}
        return enviar_json_server(payload)

    try:
        # Los output de SUMO definidos con nombres relativos (tripinfo, summary,
        # vehroute y emissions) se guardan en la carpeta del escenario.
        os.chdir(BASE_DIR)
        traci.start(cmd)
        conectado = True

        dynamic_manager = DynamicEventManager(
            traci=traci,
            net=net,
            planes=planes,
            report_path=reporte_derrumbe_path,
            min_affected_vehicles=2,
            event_callback=registrar_evento,
            plan_index_resolver=indice_plan_actual,
        )

        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            sim_time = traci.simulation.getTime()

            if dynamic_manager is not None:
                dynamic_manager.update(
                    sim_time=sim_time,
                    configured_events=eventos_configurados,
                )

                # Una reparacion terminada solo vuelve a ofrecer el camion si el
                # rescheduling sigue recibiendo ciclos y queda horizonte. Si ya
                # existe un ciclo en cola, basta con desbloquear su aplicacion.
                for repair in dynamic_manager.pop_repaired_trucks():
                    veh_repaired = str(repair.get("truck", "") or "")
                    if not veh_repaired:
                        continue

                    if float(sim_time) + 1e-9 >= HORIZONTE_TURNO_S:
                        print(
                            f"[rescheduling] {veh_repaired} fue REPARADO en "
                            f"t={sim_time:.1f}s, pero no queda horizonte; "
                            "no se solicitaran nuevos ciclos."
                        )
                        continue

                    if (
                        ciclo_activo_por_camion.get(veh_repaired)
                        or ciclos_pendientes_por_camion.get(veh_repaired)
                    ):
                        print(
                            f"[rescheduling] {veh_repaired} fue REPARADO en "
                            f"t={sim_time:.1f}s; sus ciclos pendientes vuelven "
                            "a ser aplicables."
                        )
                        continue

                    # PA_SAFE es solo la detencion temporal de la averia. Si el
                    # camion aun conserva el compromiso que estaba ejecutando
                    # cuando se activo el rescheduling, la reparacion no lo deja
                    # libre: debe retomar el ciclo y completar su descarga.
                    if veh_repaired in congelar_despues_s:
                        objetivo_reparado = disponibilidad_objetivo.get(
                            veh_repaired,
                            {},
                        )
                        print(
                            f"[rescheduling] {veh_repaired} fue REPARADO en "
                            f"t={sim_time:.1f}s; retoma su compromiso original "
                            "y quedara disponible despues de la descarga en "
                            f"{objetivo_reparado.get('ubicacion_disponible', '')}."
                        )
                        continue

                    if rescheduling_activo and not rescheduling_finalizado:
                        try:
                            edge_repaired = traci.vehicle.getRoadID(veh_repaired)
                            lane_repaired = traci.vehicle.getLaneID(veh_repaired)
                            pos_repaired = traci.vehicle.getLanePosition(veh_repaired)
                        except Exception:
                            edge_repaired = str(repair.get("parking_edge", "") or "")
                            lane_repaired = ""
                            pos_repaired = 0.0

                        repaired_response = enviar_json_server({
                            "type": "TRUCK_FREE",
                            "truck": veh_repaired,
                            "time_ms": int(round(float(sim_time) * 1000.0)),
                            "time": float(sim_time),
                            "place": str(repair.get("parking_id", "") or ""),
                            "edge": edge_repaired,
                            "lane": lane_repaired,
                            "pos": float(pos_repaired),
                            "estado_disponibilidad": "REPARADO",
                            "motivo": "FIN_AVERIA_CAMION",
                        })
                        print(
                            f"[rescheduling] Disponibilidad de {veh_repaired} "
                            f"tras reparacion informada en t={sim_time:.1f}s; "
                            f"respuesta={repaired_response.get('ok')}."
                        )
                    elif rescheduling_activo and rescheduling_finalizado:
                        print(
                            f"[rescheduling] {veh_repaired} fue REPARADO en "
                            f"t={sim_time:.1f}s, pero el rescheduling ya finalizo "
                            "y no tiene ciclos pendientes."
                        )

            # Flujo incremental: JADE entrega cada ciclo apenas un camion negocia.
            # Tras FINALIZE_RESCHEDULE no se vuelve a consultar GET_CYCLES.
            # La respuesta que trae no_more_cycles todavía puede incluir los últimos
            # ciclos ya generados; esos se procesan en esa misma iteración y luego se cierra.
            if (
                rescheduling_activo
                and not rescheduling_finalizado
                and float(sim_time) + 1e-9 >= float(proxima_consulta_plan_s)
            ):
                respuesta_ciclos = enviar_json_server({
                    "type": "GET_CYCLES",
                    "after": int(ultimo_cycle_id_recibido),
                })
                proxima_consulta_plan_s = float(sim_time) + 1.0
                if respuesta_ciclos.get("ok"):
                    if (
                        respuesta_ciclos.get("rescheduling_finalizado")
                        or respuesta_ciclos.get("no_more_cycles")
                    ):
                        if not rescheduling_finalizado:
                            print(
                                "[rescheduling] FINALIZE_RESCHEDULE recibido: "
                                "no llegarán más ciclos; se vaciarán activos y colas."
                            )
                        rescheduling_finalizado = True

                    for ciclo in respuesta_ciclos.get("cycles", []):
                        cycle_id = int(ciclo.get("id", 0) or 0)
                        camion_ciclo, plan_ciclo = construir_plan(ciclo)
                        if not camion_ciclo or not plan_ciclo.get("edges"):
                            continue
                        item = {"cycle_id": cycle_id, "plan": plan_ciclo}
                        cola = ciclos_pendientes_por_camion.setdefault(camion_ciclo, [])
                        if not any(int(x.get("cycle_id", 0)) == cycle_id for x in cola) and int(ciclo_activo_por_camion.get(camion_ciclo, {}).get("cycle_id", 0)) != cycle_id:
                            cola.append(item)
                            cola.sort(key=lambda x: (float(x["plan"].get("segmentos", [{}])[0].get("depart_ms", 0)), int(x.get("cycle_id", 0))))
                            print(f"[rescheduling] Ciclo {cycle_id} encolado para {camion_ciclo}; pendientes={len(cola)}")
                        ultimo_cycle_id_recibido = max(ultimo_cycle_id_recibido, cycle_id)
                        ack = enviar_json_server({
                            "type": "ACK_CYCLE", "id": cycle_id, "camion": camion_ciclo,
                            "time": float(sim_time), "estado": "ENCOLADO",
                        })
                        print(f"[rescheduling] Ciclo {cycle_id} recibido/encolado. ACK={ack.get('ok')}")

            vivos = set(traci.vehicle.getIDList())

            # SUMO informa explícitamente llegadas y teleports del step.
            # Un vehiculo en teleport puede desaparecer temporalmente de getIDList()
            # y luego ser reinsertado; por eso no debe eliminarse su ciclo.
            try:
                llegados_step = set(
                    traci.simulation.getArrivedIDList()
                )
            except Exception:
                llegados_step = set()

            try:
                teleport_inicio_step = set(
                    traci.simulation.getStartingTeleportIDList()
                )
            except Exception:
                teleport_inicio_step = set()

            try:
                teleport_fin_step = set(
                    traci.simulation.getEndingTeleportIDList()
                )
            except Exception:
                teleport_fin_step = set()

            retrasos_flota_actual.clear()
            contexto_retraso_actual.clear()

            if float(sim_time) + 1e-9 >= float(proximo_snapshot_monitoreo_s):
                snapshot_monitoreo = tomar_snapshot_flota(traci, sim_time, planes, args)
                escribir_snapshot(snapshot_monitoreo, sim_time)
                print(f"[monitoreo] Snapshot de monitoreo tomado en t={sim_time:.1f}s vehiculos={len(snapshot_monitoreo)}")
                while float(proximo_snapshot_monitoreo_s) <= float(sim_time) + 1e-9:
                    proximo_snapshot_monitoreo_s += SNAPSHOT_MONITOREO_INTERVAL_S

            # ============================================================
            # VEHICULOS QUE DEJARON DE APARECER EN ESTE STEP
            # ============================================================
            for veh_id in sorted(vivos_prev - vivos):
                if veh_id in camiones_retirados:
                    continue

                # Llegada normal: el vehiculo termino realmente su ruta.
                if veh_id in llegados_step:
                    finalizar_vehiculo(
                        veh_id,
                        sim_time,
                        llegada_normal=True,
                    )
                    continue

                # Teleport: SUMO lo retiro temporalmente de la red.
                # Se conserva COMPLETO su estado logico.
                if veh_id in teleport_inicio_step:
                    vehiculos_en_teleport.add(veh_id)
                    vehiculos_ausentes_desde.pop(veh_id, None)
                    print(
                        f"[SUMO] {veh_id} inicio TELEPORT "
                        f"en t={float(sim_time):.1f}s; "
                        "se conserva ciclo activo y cola pendiente."
                    )
                    continue

                # Ausencia no explicada: no destruir el ciclo en un solo step.
                # Se inicia un periodo corto de confirmacion.
                if veh_id not in vehiculos_ausentes_desde:
                    vehiculos_ausentes_desde[veh_id] = float(sim_time)
                    print(
                        f"[SUMO] ADVERTENCIA: {veh_id} dejo getIDList() "
                        f"en t={float(sim_time):.1f}s sin ARRIVED ni TELEPORT; "
                        f"se esperaran {TIEMPO_CONFIRMACION_AUSENCIA_S:.1f}s "
                        "antes de considerarlo perdido."
                    )

            # ============================================================
            # VEHICULOS REINSERTADOS DESPUES DE TELEPORT
            # ============================================================
            for veh_id in sorted(teleport_fin_step):
                vehiculos_en_teleport.discard(veh_id)
                vehiculos_ausentes_desde.pop(veh_id, None)
                print(
                    f"[SUMO] {veh_id} termino TELEPORT y fue reinsertado "
                    f"en t={float(sim_time):.1f}s; conserva su ciclo."
                )

            # Si una ausencia no explicada reaparece antes del margen,
            # se cancela sin modificar el ciclo ni su cola.
            for veh_id in list(vehiculos_ausentes_desde.keys()):
                if veh_id in vivos:
                    inicio_ausencia = float(
                        vehiculos_ausentes_desde.pop(veh_id, sim_time)
                    )
                    print(
                        f"[SUMO] {veh_id} reaparecio en t={float(sim_time):.1f}s "
                        f"tras {float(sim_time) - inicio_ausencia:.1f}s; "
                        "se conserva su ciclo activo y cola pendiente."
                    )

            # ============================================================
            # CONFIRMACION DE AUSENCIAS REALES
            # ============================================================
            for veh_id, inicio_ausencia in list(
                vehiculos_ausentes_desde.items()
            ):
                if veh_id in vivos or veh_id in vehiculos_en_teleport:
                    continue

                # Si SUMO reporta llegada normal durante la ventana,
                # se procesa como EJECUTADO.
                if veh_id in llegados_step:
                    finalizar_vehiculo(
                        veh_id,
                        sim_time,
                        llegada_normal=True,
                    )
                    continue

                # Si SUMO identifica el teleport en un step posterior,
                # se conserva el ciclo y deja de considerarse ausencia anomala.
                if veh_id in teleport_inicio_step:
                    vehiculos_en_teleport.add(veh_id)
                    vehiculos_ausentes_desde.pop(veh_id, None)
                    print(
                        f"[SUMO] {veh_id} fue identificado posteriormente "
                        f"como TELEPORT en t={float(sim_time):.1f}s; "
                        "se conserva su estado."
                    )
                    continue

                tiempo_ausente = (
                    float(sim_time) - float(inicio_ausencia)
                )
                if tiempo_ausente < TIEMPO_CONFIRMACION_AUSENCIA_S:
                    continue

                # getIDList() solo contiene vehiculos visibles. Al liberar un
                # parking, el camion puede seguir cargado en SUMO mientras espera
                # reincorporarse a la via. El plazo de 10 s no prueba su salida.
                # Consultar vehicle.getLoadedIDList(), NO simulation.getLoadedIDList()
                # (esta ultima solo enumera las incorporaciones del step).
                try:
                    cargados_sumo = traci.vehicle.getLoadedIDList()
                except Exception:
                    # Sin una consulta fiable no destruir el ciclo ni su cola.
                    continue
                if veh_id in cargados_sumo:
                    continue

                # Ausente tanto de la lista visible como del registro de SUMO.
                print(
                    f"[SUMO] ADVERTENCIA: {veh_id} continua ausente "
                    f"durante {tiempo_ausente:.1f}s y no fue reportado "
                    "como ARRIVED ni TELEPORT, y ya no esta cargado en SUMO. "
                    "Se confirma desaparicion real."
                )
                finalizar_vehiculo(
                    veh_id,
                    sim_time,
                    llegada_normal=False,
                )

            vivos_prev = vivos

            for veh_id in vivos:
                plan = planes.get(veh_id)
                if not plan:
                    continue

                # Los camiones sin scheduling inicial existen desde t=0 y permanecen
                # en su parkingArea. No generan retraso, pero pueden renegociar después.
                if (
                    not rescheduling_activo
                    and str(plan.get("estado_inicial", "")).upper() == ESTADO_SIN_SCHEDULE
                    and veh_id not in camiones_disponibles
                ):
                    # Primero deja que SUMO inserte físicamente el vehículo en su
                    # parkingArea inicial. Mientras entra al parking no participa en
                    # métricas ni genera retraso.
                    if esta_en_stop_o_parking(traci, veh_id):
                        registrar_camion(
                            veh_id, sim_time, ESTADO_SIN_SCHEDULE,
                            tiempo_disponible_s=0.0,
                            place=str(plan.get("ubicacion_inicial", veh_id) or veh_id),
                        )
                    else:
                        try:
                            traci.vehicle.setMaxSpeed(veh_id, 2.0)
                        except Exception:
                            pass
                        continue

                # Un camión ya libre sigue presente en SUMO sin interferir con la vía.
                # Si llega un ciclo de rescheduling, se retira su parking stop y se reactiva.
                if veh_id in camiones_disponibles:
                    if retirar_camion(veh_id, sim_time):
                        continue
                    asegurar_truck_free(veh_id, sim_time)
                    if rescheduling_activo:
                        estado_activacion = activar_ciclo(veh_id, sim_time)
                        if estado_activacion == "APLICADO":
                            camiones_disponibles.pop(veh_id, None)
                            camiones_fin_turno.discard(veh_id)
                    if veh_id in camiones_disponibles:
                        mantener_espera_segura(veh_id)
                    continue

                # Un camión congelado ya terminó su compromiso anterior.
                # Antes del FINALIZE puede reactivarse si llega un ciclo tardío.
                # Después del FINALIZE, con cola vacía, queda en FIN_TURNO.
                if rescheduling_activo and veh_id in congelados:
                    estado_activacion = activar_ciclo(veh_id, sim_time)
                    if estado_activacion == "APLICADO":
                        congelados.discard(veh_id)
                        camiones_fin_turno.discard(veh_id)
                    elif estado_activacion == "SIN_CICLO" and rescheduling_finalizado:
                        retirar_camion(veh_id, sim_time)
                    # ESPERANDO_HORA mantiene el camion detenido hasta horaInicio.
                    continue

                # Respaldo de cierre: si FINALIZE_RESCHEDULE ya confirmo que no
                # llegaran mas ciclos y el camion alcanzo el PA_WAIT de su ultimo
                # segmento realmente cerrado, se corrige un objetivo antiguo que
                # haya quedado congelado durante la transicion entre segmentos.
                # La comprobacion fisica posterior sigue realizandose mediante
                # descarga_finalizada(); aqui solo se actualiza el objetivo.
                if (
                    rescheduling_activo
                    and rescheduling_finalizado
                    and veh_id in congelar_despues_s
                    and not ciclo_activo_por_camion.get(veh_id)
                    and not ciclos_pendientes_por_camion.get(veh_id)
                ):
                    segmentos_actuales = plan.get("segmentos", [])
                    if segmentos_actuales:
                        ultimo_segmento = segmentos_actuales[-1]
                        key_ultimo = (
                            veh_id,
                            str(
                                ultimo_segmento.get(
                                    "tipo_plan",
                                    "SCHEDULING",
                                )
                                or "SCHEDULING"
                            ),
                            int(ultimo_segmento.get("cycle_id", 0) or 0),
                            int(ultimo_segmento.get("seg_num", 0) or 0),
                        )
                        edge_final = str(
                            ultimo_segmento.get("edge_fin", "") or ""
                        ).strip()
                        parking_final = parking_actual_vehiculo(veh_id)
                        objetivo_actual = str(
                            disponibilidad_objetivo.get(veh_id, {}).get(
                                "edge_disponible",
                                "",
                            )
                            or ""
                        ).strip()

                        try:
                            edge_fisico = str(
                                traci.vehicle.getRoadID(veh_id) or ""
                            ).strip()
                            lane_fisica = str(
                                traci.vehicle.getLaneID(veh_id) or ""
                            ).strip()
                        except Exception:
                            edge_fisico = ""
                            lane_fisica = ""

                        esta_en_edge_final = bool(
                            edge_final
                            and (
                                edge_fisico == edge_final
                                or lane_fisica == edge_final
                                or lane_fisica.startswith(edge_final + "_")
                            )
                        )

                        if (
                            key_ultimo in cerrados
                            and parking_final.startswith("PA_WAIT_")
                            and esta_en_edge_final
                            and objetivo_actual != edge_final
                        ):
                            ubicacion_final = destino_logico_segmento(
                                ultimo_segmento
                            )
                            disponibilidad_objetivo[veh_id] = {
                                "edge_disponible": edge_final,
                                "ubicacion_disponible": ubicacion_final,
                            }
                            congelar_despues_s[veh_id] = float(sim_time)
                            print(
                                f"[rescheduling] ADVERTENCIA: {veh_id} alcanzo "
                                "el PA_WAIT de su ultimo segmento, pero conservaba "
                                f"el objetivo anterior {objetivo_actual or 'VACIO'}. "
                                f"Objetivo corregido a {edge_final} para finalizar."
                            )

                if rescheduling_activo and ciclo_activo_por_camion.get(veh_id):
                    habilitar_final(veh_id)

                if rescheduling_activo and descarga_finalizada(veh_id, sim_time):
                    # Si terminó un ciclo de rescheduling activo, primero se registra
                    # como EJECUTADO y después se intenta activar el siguiente ciclo.
                    activo_finalizado = ciclo_activo_por_camion.pop(veh_id, None)
                    if activo_finalizado:
                        cycle_id_finalizado = int(activo_finalizado.get("cycle_id", 0) or 0)
                        ack_ejecutado = enviar_json_server({
                            "type": "ACK_CYCLE",
                            "id": cycle_id_finalizado,
                            "camion": veh_id,
                            "time": float(sim_time),
                            "estado": "EJECUTADO",
                        })
                        print(
                            f"[rescheduling] Ciclo {cycle_id_finalizado} ejecutado por {veh_id}; "
                            f"ACK_EJECUTADO={ack_ejecutado.get('ok')}"
                        )

                    # Si existe otro ciclo, solo se activa cuando SUMO alcanza
                    # la horaInicio absoluta definida por JADE.
                    estado_activacion = activar_ciclo(veh_id, sim_time)
                    if estado_activacion == "APLICADO":
                        congelados.discard(veh_id)
                        continue

                    # La descarga termino y su ACK ya se envio. Retirar antes del PA_WAIT.
                    if estado_activacion == "SIN_CICLO" and rescheduling_finalizado:
                        congelados.add(veh_id)
                        retirar_camion(veh_id, sim_time)
                        continue

                    objetivo_espera = disponibilidad_objetivo.get(veh_id, {})

                    # Si el siguiente ciclo aun no puede activarse, el handoff
                    # scheduling -> rescheduling solo se considera completo cuando
                    # el scheduling viejo fue cortado y el PA_WAIT correcto quedo
                    # realmente instalado. Hasta entonces NO se publica TRUCK_FREE.
                    if estado_activacion in ("ESPERANDO_HORA", "SIN_CICLO"):
                        espera_ok = agregar_espera(
                            veh_id,
                            objetivo_espera.get("edge_disponible", ""),
                        )
                        if not espera_ok:
                            print(
                                f"[rescheduling] {veh_id} termino su descarga, "
                                "pero el corte fisico aun no quedo asegurado; "
                                "se reintentara antes de liberarlo."
                            )
                            continue

                    if estado_activacion == "ESPERANDO_HORA":
                        mantener_espera_segura(veh_id)
                        congelados.add(veh_id)
                        camiones_fin_turno.discard(veh_id)
                        continue

                    if estado_activacion == "SIN_CICLO" and rescheduling_finalizado:
                        mantener_espera_segura(veh_id)
                        congelados.add(veh_id)
                        camiones_fin_turno.add(veh_id)
                        descarga_en_curso_observada.pop(veh_id, None)
                        print(
                            f"[rescheduling] {veh_id} terminó su último ciclo. "
                            f"FIN_TURNO en t={sim_time:.1f}s"
                        )
                    else:
                        mantener_espera_segura(veh_id)
                        if veh_id not in congelados:
                            print(
                                f"[rescheduling] {veh_id} termino compromiso actual. "
                                f"Queda esperando nuevo schedule desde t={sim_time:.1f}s"
                            )
                        congelados.add(veh_id)

                    # TRUCK_FREE se emite solo para inicializar este camion una vez.
                    # Tras completar un ciclo de rescheduling, espera en SUMO hasta que llegue
                    # el siguiente ciclo ya calculado por JADE.
                    if activo_finalizado:
                        continue

                    if veh_id not in camiones_disponibilidad_informada:
                        try:
                            edge_disp = traci.vehicle.getRoadID(veh_id)
                            lane_disp = traci.vehicle.getLaneID(veh_id)
                            pos_disp = traci.vehicle.getLanePosition(veh_id)
                        except Exception:
                            edge_disp, lane_disp, pos_disp = "", "", 0.0

                        # La ubicación lógica corresponde al objetivo de descarga
                        # confirmado para este compromiso/ciclo.
                        objetivo = disponibilidad_objetivo.get(veh_id, {})
                        ubicacion_disp = str(
                            objetivo.get("ubicacion_disponible", "") or ""
                        )

                        resp_disp = enviar_json_server({
                            "type": "TRUCK_FREE",
                            "truck": veh_id,
                            "time_ms": int(round(float(sim_time) * 1000.0)),
                            "time": float(sim_time),
                            "place": ubicacion_disp,
                            "edge": edge_disp,
                            "lane": lane_disp,
                            "pos": float(pos_disp),
                        })
                        if resp_disp.get("ok"):
                            camiones_disponibilidad_informada.add(veh_id)
                            print(f"[rescheduling] Disponibilidad REAL de {veh_id} enviada a JADE/server en t={sim_time:.1f}s")
                    continue

                if veh_id not in controlados:
                    traci.vehicle.setMaxSpeed(veh_id, max(plan["vacio_ms"], plan["cargado_ms"]))
                    if args.force_speed:
                        traci.vehicle.setSpeedMode(veh_id, 0)
                    controlados.add(veh_id)

                edge_actual = traci.vehicle.getRoadID(veh_id)
                if not edge_actual or edge_actual.startswith(":"):
                    continue

                route_index_sumo = traci.vehicle.getRouteIndex(veh_id)
                if route_index_sumo is None or route_index_sumo < 0:
                    continue
                route_index = indice_plan_actual(
                    traci, veh_id, plan,
                    route_index_sumo=route_index_sumo, edge_actual=edge_actual
                )
                if route_index is None or route_index < 0:
                    continue

                speed = plan["index_to_speed"].get(route_index)
                if speed is None:
                    continue

                # La velocidad compensada nunca supera el límite del lane actual.
                try:
                    lane_id = traci.vehicle.getLaneID(veh_id)
                    lane_limit = traci.lane.getMaxSpeed(lane_id) if lane_id else speed
                except Exception:
                    lane_id = ""
                    lane_limit = speed

                seg_idx = plan["index_to_segment"].get(route_index)
                seg_actual = None
                if seg_idx is not None and seg_idx < len(plan["segmentos"]):
                    seg_actual = plan["segmentos"][seg_idx]

                # Scheduling inicial: al terminar la ÚLTIMA descarga el vehículo no
                # desaparece. El .rou.xml ya contiene un hold en parkingArea; aquí
                # solo se cambia su estado lógico y se publica TRUCK_FREE.
                if (
                    not rescheduling_activo
                    and seg_actual is not None
                    and plan.get("segmentos")
                    and seg_actual is plan["segmentos"][-1]
                    and str(seg_actual.get("operacion", "")).upper() == "VIAJE_CARGADO"
                    and route_index >= int(seg_actual.get("end_idx", 0))
                    and esta_en_stop_o_parking(traci, veh_id)
                ):
                    inicio_desc = descarga_final_inicio_observada.setdefault(veh_id, float(sim_time))
                    dur_desc = max(0.0, float(seg_actual.get("stop_after_duration_s", 0.0) or 0.0))
                    if float(sim_time) + 1e-9 >= float(inicio_desc) + dur_desc:
                        registrar_camion(
                            veh_id, sim_time, ESTADO_FIN_SCHEDULE,
                            tiempo_disponible_s=float(sim_time),
                            place=destino_logico_segmento(seg_actual),
                        )
                        continue

                speed_aplicada = max(0.1, min(speed, lane_limit))

                speed_aplicada, eventos_velocidad_aplicados = aplicar_velocidad(
                    speed_aplicada,
                    sim_time,
                    edge_actual,
                    eventos_configurados,
                )

                # Reduccion progresiva SOLO cuando el camion se acerca al final de SU segmento actual.
                # Esto representa aproximacion a su proximo stop/pala/botadero, no una reduccion por
                # pasar cerca de cualquier parkingArea. Ayuda a evitar emergency braking antes del stop.
                aprox_aplicada = False
                dist_restante = 999999.0
                if seg_actual is not None:
                    dist_restante = distancia_segmento(traci, veh_id, plan, seg_actual, route_index, lane_id)
                    speed_aplicada, aprox_aplicada, dist_restante = velocidad_aprox(
                        speed_aplicada,
                        dist_restante,
                        args,
                    )

                traci.vehicle.setMaxSpeed(veh_id, speed_aplicada)
                if args.force_speed:
                    traci.vehicle.setSpeed(veh_id, speed_aplicada)
                cambios += 1

                if seg_actual is not None:
                    seg = seg_actual
                    key = clave_segmento(veh_id, seg)
                    activo = activos.get(veh_id)
                    activo_key = clave_segmento(veh_id, activo["seg"]) if activo else None

                    if key not in cerrados and activo_key != key:
                        if activo:
                            cerrar_segmento(veh_id, sim_time, "cambio_segmento")
                        try:
                            time_loss_inicio = float(traci.vehicle.getTimeLoss(veh_id))
                        except Exception:
                            time_loss_inicio = 0.0
                        activos[veh_id] = {
                            "seg": seg,
                            "sumo_inicio_s": float(sim_time),
                            "time_loss_inicio_s": time_loss_inicio,
                            "time_loss_ultimo_s": time_loss_inicio,
                        }

                    if activos.get(veh_id) and clave_segmento(veh_id, activos[veh_id]["seg"]) == key:
                        try:
                            activos[veh_id]["time_loss_ultimo_s"] = float(
                                traci.vehicle.getTimeLoss(veh_id)
                            )
                        except Exception:
                            pass
                        try:
                            pos_actual = traci.vehicle.getLanePosition(veh_id)
                        except Exception:
                            pos_actual = 0.0
                        try:
                            velocidad_actual_s = traci.vehicle.getSpeed(veh_id)
                        except Exception:
                            velocidad_actual_s = speed_aplicada
                        registrar_retraso(veh_id, seg, sim_time, edge_actual, lane_id, pos_actual, velocidad_actual_s, route_index_sumo)
                        if route_index >= seg["end_idx"] and esta_en_stop_o_parking(traci, veh_id):
                            cerrar_segmento(veh_id, sim_time, "llego_a_stop")

                operacion = plan["index_to_operacion"].get(route_index, "")
                estado = (
                    int(route_index_sumo), int(route_index), operacion,
                    round(speed_aplicada, 4), bool(aprox_aplicada)
                )
                if ultimo_estado.get(veh_id) != estado:
                    extra_aprox = ""
                    if aprox_aplicada:
                        extra_aprox = f" aprox_destino=SI dist_restante={dist_restante:.1f}m"
                    print(
                        f"[velocidad] {veh_id} idx_plan={route_index} "
                        f"idx_sumo={route_index_sumo} op={operacion} "
                        f"compensada={speed:.3f} aplicada={speed_aplicada:.3f} m/s "
                        f"edge={edge_actual}"
                        f"{extra_aprox}"
                    )
                    ultimo_estado[veh_id] = estado

            evaluar_rescheduling(traci, sim_time)

            # Si nunca se activa el rescheduling, los camiones disponibles no
            # mantienen la simulación abierta indefinidamente. Se espera al horizonte
            # de 6 h y, si toda la flota ya está disponible, se cierra el turno.
            if (
                not rescheduling_activo
                and float(sim_time) + 1e-9 >= HORIZONTE_TURNO_S
                and len(set(camiones_disponibles) | camiones_retirados) >= len(planes)
            ):
                print(
                    f"[turno] Horizonte alcanzado t={sim_time:.1f}s; "
                    f"camiones disponibles={len(camiones_disponibles)}/{len(planes)}. "
                    "Cierre SUMO sin rescheduling."
                )
                break

            # Cierre global controlado: FINALIZE_RESCHEDULE confirma que JADE no
            # enviará más ciclos. SUMO termina cuando ya no existen ciclos activos
            # ni pendientes. Los vehículos estacionados no mantienen abierto el reloj.
            if rescheduling_activo and rescheduling_finalizado:
                hay_ciclos_activos = bool(ciclo_activo_por_camion)
                hay_ciclos_pendientes = any(
                    bool(cola) for cola in ciclos_pendientes_por_camion.values()
                )

                camiones_pendientes_fisicos = (
                    set(congelar_despues_s.keys())
                    | set(disponibilidad_objetivo.keys())
                    | set(descarga_en_curso_observada.keys())
                    | set(vehiculos_en_teleport)
                    | set(camiones_disponibles.keys())
                    | set(congelados)
                )

                # Los camiones retirados correctamente ya completaron su plan.
                camiones_pendientes_fisicos.difference_update(camiones_retirados)
                hay_compromisos_fisicos = bool(camiones_pendientes_fisicos)

                if (
                    not hay_ciclos_activos
                    and not hay_ciclos_pendientes
                    and not hay_compromisos_fisicos
                ):
                    print(
                        "[rescheduling] Todos los ciclos y compromisos físicos "
                        f"fueron completados. Cierre global SUMO en t={sim_time:.1f}s"
                    )
                    break

                # Informa una vez por minuto qué camiones impiden el cierre,
                # sin llenar la consola con un mensaje en cada step.
                if (
                    not hay_ciclos_activos
                    and not hay_ciclos_pendientes
                    and hay_compromisos_fisicos
                    and float(sim_time) - float(ultimo_cierre) >= 60.0
                ):
                    ultimo_cierre = float(sim_time)
                    print(
                        "[rescheduling] Los ciclos nuevos terminaron, pero SUMO "
                        "continúa esperando compromisos físicos: "
                        + ", ".join(sorted(camiones_pendientes_fisicos))
                        + f" en t={sim_time:.1f}s"
                    )


    finally:
        if dynamic_manager is not None:
            try:
                dynamic_manager.close()
            except Exception as exc:
                print(f"[eventos] advertencia cerrando DynamicEventManager: {exc}")

        if conectado:
            try:
                final_time = traci.simulation.getTime()
            except Exception:
                final_time = 0.0
            for veh_id in list(activos.keys()):
                cerrar_segmento(veh_id, final_time, "fin_simulacion")
            traci.close()
        csv_file.close()
        eventos_file.close()
        try:
            if zmq_socket is not None:
                zmq_socket.close(0)
        except Exception:
            pass
        try:
            if zmq_context is not None:
                zmq_context.term()
        except Exception:
            pass

        if gestor_metricas is not None:
            try:
                resumen_metricas = gestor_metricas.finalizar(
                    tiempo_sumo_final_s=final_time,
                    reporte_segmentos_path=reporte_path,
                    tripinfo_path=BASE_DIR / "tripinfo.xml",
                )
                print(f"[metricas] Resumen generado: {BASE_DIR / 'metricas_resumen.json'}")
            except Exception as exc:
                print(f"[metricas] ERROR generando resumen: {exc}")

    print("---------------------------------------------------")
    print(f"Vehiculos controlados: {len(controlados)}")
    print(f"Aplicaciones de velocidad: {cambios}")
    print(f"Reporte segmentos: {reporte_path}")
    print(f"Eventos dinamicos: {eventos_path}")
    print(f"Solicitudes rescheduling: {rescheduling_path}")
    print(f"Eventos de retraso registrados: {len(eventos_registrados)}")
    print(f"Solicitudes globales de rescheduling: {len(solicitudes_rescheduling)}")
    print(f"Camiones congelados esperando nuevo schedule: {len(congelados)}")
    if resumen_metricas is not None:
        indicadores = resumen_metricas.get("indicadores", {})
        print(f"Error relativo promedio: {float(indicadores.get('error_relativo_promedio_pct', 0.0)):.3f}%")
        print(f"Tiempo perdido promedio: {float(indicadores.get('tiempo_perdido_promedio_s', 0.0)):.3f}s")
        print(f"CPU promedio: {float(indicadores.get('cpu_promedio_pct', 0.0)):.3f}%")
        print(f"RAM promedio: {float(indicadores.get('ram_promedio_mb', 0.0)):.3f}MB")
        print(f"Duracion real: {float(indicadores.get('duracion_real_ejecucion_s', 0.0)):.3f}s")
    print("Simulacion finalizada.")


if __name__ == "__main__":
    main()

'''
    contenido = contenido.replace("__ESCENARIO_ACTIVO__", ESCENARIO)
    contenido = contenido.replace("__SUMOCFG_DEFAULT__", os.path.basename(SUMOCFG_GENERADO))
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(contenido)
    print(f"[server] Runner TraCI generado: {path}")

def generar_archivos_sumo(memoria: MemoriaCronograma) -> Dict[str, Any]:
    validacion = memoria.validar()
    if validacion.get("tiempos_invalidos"):
        return {
            "ok": False,
            "error": "No se generan XML porque hay tiempos invalidos.",
            "validacion": validacion,
        }

    trips, advertencias, vtypes = construir_trips(memoria)
    if not trips:
        return {
            "ok": False,
            "error": "No se genero ningun vehicle. Revisa VIAJE_* y ubicaciones compatibles con net.net.xml.",
            "validacion": validacion,
            "advertencias": advertencias,
        }

    generar_rou_xml_directo(trips, ROU_XML, vtypes)
    generar_trips(trips, TRIPS_XML, vtypes)
    objects_info = generar_object_generado(trips, OBJECTS_GENERADO_XML, memoria.net.lane_lengths)
    generar_sumocfg(SUMOCFG_GENERADO, memoria.net.path)
    generar_runner(TRACI_RUNNER_GENERADO)

    return {
        "ok": True,
        "eventos": len(memoria.eventos),
        "camiones_o_propietarios": len(memoria.por_camion),
        "vehiculos": len(trips),
        "stops": sum(len(t.get("stops", [])) for t in trips),
        "vtypes": len(vtypes),
        "cronograma_csv": CRONOGRAMA_CSV,
        "cronograma_json": CRONOGRAMA_JSON,
        "trips_xml": TRIPS_XML,
        "rou_xml": ROU_XML,
        "objects_add": OBJECTS_GENERADO_XML,
        "sumocfg_generado": SUMOCFG_GENERADO,
        "traci_runner": TRACI_RUNNER_GENERADO,
        "parking_areas": objects_info.get("parking_areas", 0),
        "validacion": validacion,
        "advertencias": advertencias,
        "rutas": "SUMO_NET_DESDE_CRONOGRAMA_JADE",
    }
