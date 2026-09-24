# -*- coding: utf-8 -*-
"""
routing_service.py
Servicio de rutas seguro basado en la red SUMO.

Idea principal:
- NO calcula rutas sobre junctions como si fueran ubicaciones operativas.
- Los camiones, palas y destinos deben resolverse a edge/lane/pos.
- El grafo de rutas se construye con <connection from="edgeA" to="edgeB"> del net.net.xml.
- Si no existe una connection real, la ruta se rechaza.
"""

import heapq
import os
import re
import xml.etree.ElementTree as ET
from collections import defaultdict
from typing import Any, Dict, List, Set, Tuple

# Directorio real del proyecto: contiene server.py, net.net.xml y los XML
# del escenario pequeño. No depende del directorio desde el que se ejecute Python.
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
BASE_CONFIG = os.path.join(PROJECT_ROOT, "config")

ESCENARIOS_VALIDOS = ("small", "mediano", "grande")
ESCENARIOS_ALIAS = {
    "small": "small",
    "pequeno": "small",
    "pequeño": "small",
    "mediano": "mediano",
    "medium": "mediano",
    "grande": "grande",
    "large": "grande",
}

ESCENARIO = "small"
ESCENARIO_INPUT_DIR = PROJECT_ROOT
ESCENARIO_OUTPUT_DIR = os.path.join(BASE_CONFIG, ESCENARIO)
# Alias conservado para compatibilidad con código anterior.
SMALL_DIR = ESCENARIO_OUTPUT_DIR

# Las listas se actualizan sin reemplazar el objeto. Así, otros módulos que las
# importaron mediante ``from routing_service import ...`` reciben los cambios.
NET_FILE_CANDIDATOS: List[str] = []
OBJECTS_FILE_CANDIDATOS: List[str] = []
TRUCKS_FILE_CANDIDATOS: List[str] = []


def normalizar_escenario(escenario: Any = "small") -> str:
    valor = str(escenario or "small").strip().lower()
    canonico = ESCENARIOS_ALIAS.get(valor, valor)
    if canonico not in ESCENARIOS_VALIDOS:
        raise ValueError(
            f"Escenario no valido: {escenario!r}. "
            "Usa small, mediano o grande."
        )
    return canonico


def directorio_entrada(escenario: Any = "small") -> str:
    escenario = normalizar_escenario(escenario)
    # El escenario pequeño usa objects.xml y trucks.xml de la raíz.
    if escenario == "small":
        return PROJECT_ROOT
    # Mediano y grande usan sus carpetas hermanas de server.py.
    return os.path.join(PROJECT_ROOT, escenario)


def directorio_salida(escenario: Any = "small") -> str:
    escenario = normalizar_escenario(escenario)
    return os.path.join(BASE_CONFIG, escenario)


def configurar_escenario(escenario: Any = "small") -> Dict[str, str]:
    """
    Selecciona los XML de entrada y la carpeta de salida.

    Entradas:
    - small:   net.net.xml + objects.xml + trucks.xml en PROJECT_ROOT.
    - mediano: red común en PROJECT_ROOT + XML en PROJECT_ROOT/mediano.
    - grande:  red común en PROJECT_ROOT + XML en PROJECT_ROOT/grande.

    Salidas:
    - PROJECT_ROOT/config/<escenario>.
    """
    global ESCENARIO, ESCENARIO_INPUT_DIR, ESCENARIO_OUTPUT_DIR, SMALL_DIR

    ESCENARIO = normalizar_escenario(escenario)
    ESCENARIO_INPUT_DIR = directorio_entrada(ESCENARIO)
    ESCENARIO_OUTPUT_DIR = directorio_salida(ESCENARIO)
    SMALL_DIR = ESCENARIO_OUTPUT_DIR

    net_path = os.path.join(PROJECT_ROOT, "net.net.xml")
    objects_path = os.path.join(ESCENARIO_INPUT_DIR, "objects.xml")

    NET_FILE_CANDIDATOS[:] = [net_path]
    OBJECTS_FILE_CANDIDATOS[:] = [objects_path]
    # Se conserva el respaldo trucks_enriquecido.xml dentro de la misma carpeta
    # del escenario, pero nunca se mezclan camiones de otro escenario.
    TRUCKS_FILE_CANDIDATOS[:] = [
        os.path.join(ESCENARIO_INPUT_DIR, "trucks.xml"),
        os.path.join(ESCENARIO_INPUT_DIR, "trucks_enriquecido.xml"),
    ]

    trucks_path = archivo_existente(TRUCKS_FILE_CANDIDATOS, obligatorio=False)
    return {
        "escenario": ESCENARIO,
        "project_root": PROJECT_ROOT,
        "input_dir": ESCENARIO_INPUT_DIR,
        "output_dir": ESCENARIO_OUTPUT_DIR,
        "net": net_path,
        "objects": objects_path,
        "trucks": trucks_path or TRUCKS_FILE_CANDIDATOS[0],
    }


# Configuración predeterminada. server.py puede cambiarla antes de crear NetSumo.
# La llamada se realiza después de definir archivo_existente().

VELOCIDAD_DEF_VACIO_KMH = 24.0
VELOCIDAD_DEF_CARGADO_KMH = 18.0
VELOCIDAD_MAXIMA_SEGURA_MS = 35.0
ESPERA_INACTIVA_MIN_MS = 30000

# Seguridad:
# True = solo permite rutas entre edges conectados con <connection>.
# False = permitiría reconstruir por junction como respaldo. Para tu caso debe quedar True.
STRICT_CONNECTIONS = True

# True = si una ubicación viene solo como junction, el servicio falla.
# Para tu caso debe quedar False, porque NO queremos volver al problema antiguo.
ALLOW_JUNCTION_FALLBACK = False


def normalizar_id(valor: Any) -> str:
    return str(valor or "").replace("#", "").strip()


def convertir_int(valor: Any, default: int = 0) -> int:
    try:
        return int(float(str(valor).replace(",", ".").strip()))
    except Exception:
        return default


def ms_a_seg(valor_ms: Any) -> float:
    return convertir_int(valor_ms, 0) / 1000.0


def tomar(dic: Dict[str, Any], claves: List[str], default: Any = "") -> Any:
    for c in claves:
        if c in dic and dic.get(c) not in (None, ""):
            return dic.get(c)
    lower = {str(k).lower(): k for k in dic.keys()}
    for c in claves:
        real = lower.get(c.lower())
        if real and dic.get(real) not in (None, ""):
            return dic.get(real)
    return default


def archivo_existente(candidatos: List[str], obligatorio: bool = True) -> str:
    for path in candidatos:
        if os.path.exists(path):
            return path
    if obligatorio:
        raise FileNotFoundError("No se encontro ninguno de estos archivos: " + str(candidatos))
    return ""


configurar_escenario(os.environ.get("SUMO_SCENARIO", "small"))


def indent_xml(elem: ET.Element, level: int = 0) -> None:
    i = "\n" + level * "    "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "    "
        last_child = None
        for child in elem:
            indent_xml(child, level + 1)
            last_child = child
        if last_child is not None and (not last_child.tail or not last_child.tail.strip()):
            last_child.tail = i
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = i


def id_xml_seguro(valor: str) -> str:
    limpio = re.sub(r"[^A-Za-z0-9_\-]", "_", str(valor or ""))
    return limpio or "X"


def parsear_velocidad_ms(valor: Any, default_kmh: float) -> float:
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


def es_operacion_viaje(operacion: str) -> bool:
    return str(operacion or "").upper().startswith("VIAJE")


def es_carga_descarga(operacion: str) -> bool:
    op = str(operacion or "").upper()
    return op in ("CARGA", "DESCARGA")


def es_camion(id_maquina: str) -> bool:
    return normalizar_id(id_maquina).startswith("Truck")


def _float(valor: Any, default: float = 0.0) -> float:
    try:
        return float(str(valor).replace(",", "."))
    except Exception:
        return default


class NetSumo:
    def __init__(self, path: str, objects_path: str = "", trucks_path: str = ""):
        self.path = path
        self.objects_path = objects_path or archivo_existente(OBJECTS_FILE_CANDIDATOS, obligatorio=False)
        self.trucks_path = trucks_path or archivo_existente(TRUCKS_FILE_CANDIDATOS, obligatorio=False)

        self.edge_ids = set()
        self.junction_ids = set()

        self.edge_from: Dict[str, str] = {}
        self.edge_to: Dict[str, str] = {}
        self.edge_length: Dict[str, float] = {}
        self.edge_speed: Dict[str, float] = {}

        self.edge_to_lane: Dict[str, str] = {}
        self.lane_to_edge: Dict[str, str] = {}
        self.lane_lengths: Dict[str, float] = {}
        self.lane_speeds: Dict[str, float] = {}

        # Grafo seguro: edge -> edge, construido con <connection>.
        self.edge_graph: Dict[str, List[Tuple[str, float]]] = defaultdict(list)
        self.connection_set = set()

        # Grafo viejo por junction solo queda para diagnóstico, no para rutas si STRICT_CONNECTIONS=True.
        self.graph: Dict[str, List[Tuple[str, str, float]]] = defaultdict(list)

        # Ubicaciones lógicas: PA01, CS03, Truck0, CA100, etc.
        self.locations: Dict[str, Dict[str, Any]] = {}

        # Diagnósticos
        self.location_errors: List[str] = []
        self.connection_count = 0

        # Cache compartido de rutas ya calculadas.
        #
        # Clave:
        #   (edge_origen, edge_destino, tuple(edges_excluidos_ordenados))
        #
        # Valor:
        #   (lista_edges_ruta, estado)
        #
        # Esto evita repetir Dijkstra para una combinación de ruta que ya fue
        # resuelta anteriormente. La firma pública de los métodos de routing no
        # cambia, por lo que server.py y los demás módulos siguen funcionando igual.
        self.route_cache: Dict[
            Tuple[str, str, Tuple[str, ...]],
            Tuple[List[str], str]
        ] = {}

        self.cargar()

    def cargar(self) -> None:
        root = ET.parse(self.path).getroot()

        for junction in root.findall("junction"):
            jid = normalizar_id(junction.get("id", ""))
            if jid and not jid.startswith(":"):
                self.junction_ids.add(jid)

        for edge in root.findall("edge"):
            edge_id = normalizar_id(edge.get("id", ""))
            if not edge_id or edge_id.startswith(":"):
                continue

            from_node = normalizar_id(edge.get("from", ""))
            to_node = normalizar_id(edge.get("to", ""))
            if not from_node or not to_node:
                continue

            length = 1.0
            speed = 10.0
            lanes = edge.findall("lane")
            lane_id = ""

            if lanes:
                # SUMO puede tener más de un lane por edge. Para ubicación inicial usamos el primero,
                # pero registramos todos los lanes para poder resolver lane -> edge.
                for i, lane in enumerate(lanes):
                    lid = normalizar_id(lane.get("id", ""))
                    llen = max(0.001, _float(lane.get("length", "1"), 1.0))
                    lspd = max(0.1, _float(lane.get("speed", "10"), 10.0))
                    if lid:
                        self.lane_to_edge[lid] = edge_id
                        self.lane_lengths[lid] = llen
                        self.lane_speeds[lid] = lspd
                        if i == 0:
                            lane_id = lid
                            length = llen
                            speed = lspd

            self.edge_ids.add(edge_id)
            self.edge_from[edge_id] = from_node
            self.edge_to[edge_id] = to_node
            self.edge_length[edge_id] = max(0.001, length)
            self.edge_speed[edge_id] = max(0.1, speed)
            if lane_id:
                self.edge_to_lane[edge_id] = lane_id

            # Solo para diagnóstico.
            self.graph[from_node].append((to_node, edge_id, max(0.001, length)))

        self._cargar_connections(root)
        self._cargar_objects()
        self._cargar_trucks()

        print(f"[routing] net cargado desde {self.path}: junctions={len(self.junction_ids)} edges={len(self.edge_ids)}")
        print(f"[routing] conexiones reales <connection>: {self.connection_count}")
        if STRICT_CONNECTIONS:
            print("[routing] modo seguro: rutas SOLO por <connection> edge->edge")
        if not ALLOW_JUNCTION_FALLBACK:
            print("[routing] modo seguro: junction NO se usa como ubicacion operativa")
        if self.objects_path:
            print(f"[routing] objects cargado: {self.objects_path}")
        if self.trucks_path:
            print(f"[routing] trucks cargado: {self.trucks_path}")
        if self.location_errors:
            print("[routing] advertencias de ubicaciones:")
            for err in self.location_errors[:20]:
                print("  -", err)
            if len(self.location_errors) > 20:
                print(f"  ... {len(self.location_errors) - 20} advertencias mas")

    def _cargar_connections(self, root: ET.Element) -> None:
        for conn in root.findall("connection"):
            from_edge = normalizar_id(conn.get("from", ""))
            to_edge = normalizar_id(conn.get("to", ""))

            # Ignorar conexiones internas o inválidas.
            if not from_edge or not to_edge:
                continue
            if from_edge.startswith(":") or to_edge.startswith(":"):
                continue
            if from_edge not in self.edge_ids or to_edge not in self.edge_ids:
                continue

            cost = self.edge_length.get(to_edge, 1.0)
            self.edge_graph[from_edge].append((to_edge, cost))
            self.connection_set.add((from_edge, to_edge))
            self.connection_count += 1

        if STRICT_CONNECTIONS and self.connection_count == 0:
            raise ValueError(
                "El net.net.xml no contiene <connection> válidas entre edges. "
                "Con STRICT_CONNECTIONS=True no se puede calcular rutas seguras."
            )

        # Solo si explícitamente se desactiva seguridad se permite fallback por from/to.
        if not STRICT_CONNECTIONS and not self.edge_graph:
            by_from = defaultdict(list)
            for e, frm in self.edge_from.items():
                by_from[frm].append(e)
            for e, to_node in self.edge_to.items():
                for nxt in by_from.get(to_node, []):
                    self.edge_graph[e].append((nxt, self.edge_length.get(nxt, 1.0)))
                    self.connection_set.add((e, nxt))

    def _registrar_location(self, key: str, data: Dict[str, Any]) -> None:
        key = normalizar_id(key)
        if key:
            self.locations[key] = data

    def _leer_location(self, loc: ET.Element, source: str, obj_id: str = "") -> Dict[str, Any]:
        edge = normalizar_id(loc.get("edge", ""))
        lane = normalizar_id(loc.get("lane", ""))

        if lane and lane not in self.lane_to_edge:
            self.location_errors.append(f"{source}:{obj_id} lane no existe en net: {lane}")
            lane = ""

        if edge and edge not in self.edge_ids:
            self.location_errors.append(f"{source}:{obj_id} edge no existe en net: {edge}")
            edge = ""

        if not edge and lane:
            edge = self.lane_to_edge.get(lane, "")

        if not lane and edge:
            lane = self.edge_to_lane.get(edge, "")

        start_pos = _float(loc.get("startPos", loc.get("departPos", 0)), 0.0)
        end_pos = _float(loc.get("endPos", loc.get("startPos", loc.get("departPos", 0))), start_pos)
        depart_pos = _float(loc.get("departPos", loc.get("startPos", 0)), start_pos)

        if edge:
            edge_len = self.edge_length.get(edge, 0.0)
            start_pos = max(0.0, min(start_pos, edge_len))
            end_pos = max(0.0, min(end_pos, edge_len))
            depart_pos = max(0.0, min(depart_pos, edge_len))

        return {
            "id": obj_id,
            "source": source,
            "original": normalizar_id(loc.get("original", "")),
            "junction": normalizar_id(loc.get("junction", "")),
            "edge": edge,
            "lane": lane,
            "startPos": start_pos,
            "endPos": end_pos,
            "departPos": depart_pos,
            "departLane": normalizar_id(loc.get("departLane", "0")),
            "stopType": normalizar_id(loc.get("stopType", "")),
            "roadsideCapacity": normalizar_id(loc.get("roadsideCapacity", "")),
            "friendlyPos": normalizar_id(loc.get("friendlyPos", "true")),
            "valid": bool(edge and lane),
        }

    def _cargar_objects(self) -> None:
        if not self.objects_path or not os.path.exists(self.objects_path):
            return

        root = ET.parse(self.objects_path).getroot()
        for obj in root.findall("object"):
            obj_id = normalizar_id(obj.get("id", ""))
            loc = obj.find("location")
            if not obj_id or loc is None:
                continue

            data = self._leer_location(loc, "objects", obj_id)
            data["objectType"] = normalizar_id(obj.get("type", ""))

            if not data.get("valid"):
                self.location_errors.append(
                    f"objects:{obj_id} sin edge/lane valido. No se usara como ubicacion operativa."
                )

            self._registrar_location(obj_id, data)
            if data.get("original"):
                self._registrar_location(data["original"], data)
            # No registramos junction como alias operativo si ALLOW_JUNCTION_FALLBACK=False.
            if ALLOW_JUNCTION_FALLBACK and data.get("junction"):
                self._registrar_location(data["junction"], data)

    def _cargar_trucks(self) -> None:
        if not self.trucks_path or not os.path.exists(self.trucks_path):
            return

        root = ET.parse(self.trucks_path).getroot()
        for truck in root.findall("truck"):
            jade_id = normalizar_id(truck.get("jadeId", ""))
            sumo_id = normalizar_id(truck.get("sumoId", ""))

            loc = truck.find("location")
            if loc is None:
                self.location_errors.append(f"trucks:{jade_id or sumo_id} sin <location>")
                continue

            data = self._leer_location(loc, "trucks", jade_id)
            data["jadeId"] = jade_id
            data["sumoId"] = sumo_id

            for tag in ["emptySpeed", "loadedSpeed", "capacity", "dischargeTime", "spottingTime", "operationTime"]:
                child = truck.find(tag)
                if child is not None and child.text is not None:
                    data[tag] = child.text.strip()
                    data[tag + "Unit"] = normalizar_id(child.get("unit", ""))

            if not data.get("valid"):
                self.location_errors.append(
                    f"trucks:{jade_id or sumo_id} no tiene edge/lane valido. "
                    "Debes enriquecer trucks.xml; el junction queda solo como referencia."
                )

            self._registrar_location(jade_id, data)
            self._registrar_location(sumo_id, data)
            if data.get("original"):
                self._registrar_location(data["original"], data)
            # No registramos junction como alias operativo.

    def resolver_ubicacion(self, ubicacion: str, como_destino: bool = False) -> Dict[str, Any]:
        u = normalizar_id(ubicacion)
        if not u:
            return {"id": "", "source": "empty", "edge": "", "lane": "", "valid": False}

        if u in self.locations:
            data = self.locations[u]
            if data.get("edge") and data.get("lane"):
                return data
            return {**data, "valid": False}

        if u in self.edge_ids:
            lane = self.edge_to_lane.get(u, "")
            return {
                "id": u,
                "source": "edge_direct",
                "edge": u,
                "lane": lane,
                "startPos": 0.0,
                "endPos": self.edge_length.get(u, 0.0),
                "departPos": 0.0,
                "valid": bool(lane),
            }

        if u in self.lane_to_edge:
            edge = self.lane_to_edge[u]
            return {
                "id": u,
                "source": "lane_direct",
                "edge": edge,
                "lane": u,
                "startPos": 0.0,
                "endPos": self.lane_lengths.get(u, 0.0),
                "departPos": 0.0,
                "valid": True,
            }

        if u in self.junction_ids:
            if not ALLOW_JUNCTION_FALLBACK:
                return {
                    "id": u,
                    "source": "junction_rechazado",
                    "junction": u,
                    "edge": "",
                    "lane": "",
                    "valid": False,
                    "error": "JUNCTION_NO_ES_UBICACION_OPERATIVA",
                }

            # Modo no recomendado. Solo si se activa ALLOW_JUNCTION_FALLBACK.
            salientes = [e for e, frm in self.edge_from.items() if frm == u]
            entrantes = [e for e, to in self.edge_to.items() if to == u]
            chosen = entrantes[0] if como_destino and entrantes else salientes[0] if salientes else ""
            return {
                "id": u,
                "source": "junction_fallback",
                "junction": u,
                "edge": chosen,
                "lane": self.edge_to_lane.get(chosen, ""),
                "startPos": 0.0,
                "endPos": self.edge_length.get(chosen, 0.0),
                "departPos": 0.0,
                "valid": bool(chosen),
            }

        return {"id": u, "source": "unknown", "edge": "", "lane": "", "valid": False, "error": "UBICACION_DESCONOCIDA"}

    def validar_route_edges(self, route_edges: List[str]) -> Tuple[bool, str]:
        if not route_edges:
            return False, "ROUTE_VACIA"

        for e in route_edges:
            if e not in self.edge_ids:
                return False, f"EDGE_NO_EXISTE:{e}"

        for a, b in zip(route_edges, route_edges[1:]):
            if a == b:
                continue
            if (a, b) not in self.connection_set:
                return False, f"SIN_CONNECTION_REAL:{a}->{b}"

        return True, "OK"

    def validar_ruta_excluida(
        self,
        route_edges: List[str],
        excluded_edges: Set[str] | None = None,
    ) -> Tuple[bool, str]:
        """Valida conexiones reales y confirma que la ruta no use edges bloqueados."""
        excluded = {normalizar_id(e) for e in (excluded_edges or set()) if normalizar_id(e)}
        ok, detail = self.validar_route_edges(route_edges)
        if not ok:
            return ok, detail
        for edge in route_edges:
            if normalizar_id(edge) in excluded:
                return False, f"EDGE_BLOQUEADO_EN_RUTA:{normalizar_id(edge)}"
        return True, "OK"

    def ruta_entre_edges(
        self,
        origen_edge: str,
        destino_edge: str,
        excluded_edges: Set[str] | None = None,
    ) -> Tuple[List[str], str]:
        origen_edge = normalizar_id(origen_edge)
        destino_edge = normalizar_id(destino_edge)
        excluded_edges = {normalizar_id(e) for e in (excluded_edges or set()) if normalizar_id(e)}

        if origen_edge not in self.edge_ids:
            return [], f"ORIGEN_EDGE_NO_EXISTE:{origen_edge}"
        if destino_edge not in self.edge_ids:
            return [], f"DESTINO_EDGE_NO_EXISTE:{destino_edge}"

        if origen_edge in excluded_edges:
            return [], f"ORIGEN_EDGE_BLOQUEADO:{origen_edge}"
        if destino_edge in excluded_edges:
            return [], f"DESTINO_EDGE_BLOQUEADO:{destino_edge}"

        if origen_edge == destino_edge:
            return [origen_edge], "OK_MISMO_EDGE"

        # La misma pareja origen/destino puede solicitarse muchas veces durante
        # las negociaciones JADE. Si ya fue calculada con el mismo conjunto de
        # edges excluidos, se reutiliza el resultado.
        clave_cache = (
            origen_edge,
            destino_edge,
            tuple(sorted(excluded_edges)),
        )

        if clave_cache in self.route_cache:
            ruta_cache, estado_cache = self.route_cache[clave_cache]

            # Se devuelve una copia para impedir que otro módulo modifique la
            # lista almacenada en cache accidentalmente.
            return list(ruta_cache), estado_cache

        # Si la ruta aún no existe en cache se conserva exactamente el algoritmo
        # original de camino mínimo.
        dist = {origen_edge: 0.0}
        prev: Dict[str, str] = {}
        pq = [(0.0, origen_edge)]

        while pq:
            actual_dist, edge = heapq.heappop(pq)
            if actual_dist > dist.get(edge, float("inf")):
                continue
            if edge == destino_edge:
                break

            for to_edge, cost in self.edge_graph.get(edge, []):
                if to_edge in excluded_edges:
                    continue
                nd = actual_dist + cost
                if nd < dist.get(to_edge, float("inf")):
                    dist[to_edge] = nd
                    prev[to_edge] = edge
                    heapq.heappush(pq, (nd, to_edge))

        if destino_edge not in dist:
            return [], f"SIN_RUTA_EDGE_CONNECTION:{origen_edge}->{destino_edge}"

        route = []
        cur = destino_edge
        while cur != origen_edge:
            route.append(cur)
            cur = prev.get(cur, "")
            if not cur:
                return [], f"SIN_RECONSTRUIR_EDGE:{origen_edge}->{destino_edge}"
        route.append(origen_edge)
        route.reverse()

        ok, motivo = self.validar_route_edges(route)
        if not ok:
            return [], motivo

        if any(e in excluded_edges for e in route):
            return [], "RUTA_CONTIENE_EDGE_EXCLUIDO"

        estado = "OK_CONNECTIONS"
        if excluded_edges:
            estado += ":EXCLUYENDO=" + ",".join(sorted(excluded_edges))

        # Solo se almacenan rutas válidas. Los errores no se cachean para no
        # ocultar posibles cambios posteriores de estado o configuración.
        self.route_cache[clave_cache] = (
            list(route),
            estado,
        )

        return route, estado

    def shortest_edges(
        self,
        origen: str,
        destino: str,
        excluded_edges: Set[str] | None = None,
    ) -> Tuple[List[str], str]:
        ro = self.resolver_ubicacion(origen, como_destino=False)
        rd = self.resolver_ubicacion(destino, como_destino=True)

        if not ro.get("valid"):
            return [], f"ORIGEN_INVALIDO:{origen}:{ro.get('error', ro.get('source', ''))}"
        if not rd.get("valid"):
            return [], f"DESTINO_INVALIDO:{destino}:{rd.get('error', rd.get('source', ''))}"

        origen_edge = ro.get("edge", "")
        destino_edge = rd.get("edge", "")

        if not origen_edge or not destino_edge:
            return [], f"ORIGEN_DESTINO_SIN_EDGE:{origen}->{destino}"

        return self.ruta_entre_edges(
            origen_edge,
            destino_edge,
            excluded_edges=excluded_edges,
        )

    def distancia_edges(self, route_edges: List[str]) -> float:
        ok, motivo = self.validar_route_edges(route_edges)
        if not ok:
            # No inventamos distancia si la ruta no es ejecutable.
            return 0.0
        return sum(float(self.edge_length.get(e, 0.0)) for e in route_edges)

    def diagnostico_ubicacion(self, ubicacion: str) -> Dict[str, Any]:
        return self.resolver_ubicacion(ubicacion)

    def diagnostico(self) -> Dict[str, Any]:
        trucks_invalidos = []
        objects_invalidos = []

        for key, data in self.locations.items():
            src = data.get("source", "")
            if data.get("valid", False):
                continue
            if src == "trucks":
                ident = data.get("jadeId") or data.get("sumoId") or key
                if ident not in trucks_invalidos:
                    trucks_invalidos.append(ident)
            elif src == "objects":
                ident = data.get("id") or key
                if ident not in objects_invalidos:
                    objects_invalidos.append(ident)

        return {
            "net": self.path,
            "objects": self.objects_path,
            "trucks": self.trucks_path,
            "edges": len(self.edge_ids),
            "junctions": len(self.junction_ids),
            "connections": self.connection_count,
            "strict_connections": STRICT_CONNECTIONS,
            "allow_junction_fallback": ALLOW_JUNCTION_FALLBACK,
            "locations": len(self.locations),
            "trucks_invalidos": sorted(set(trucks_invalidos)),
            "objects_invalidos": sorted(set(objects_invalidos)),
            "location_errors": self.location_errors,
        }
