from __future__ import annotations

import csv
import os
import random
from collections import Counter
from copy import deepcopy
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Set, Tuple

from routing_service import NetSumo, normalizar_id


# eventos.csv se mantiene en la misma carpeta que dynamic_events.py.
# El runner generado puede importar esta ruta y usarla como configuración predeterminada.
EVENTS_CSV_DEFAULT = Path(__file__).resolve().with_name("eventos.csv")

# Un vehiculo puede desaparecer de getIDList() durante una transicion interna
# de SUMO. La averia solo se considera perdida si la ausencia persiste.
TRUCK_BREAKDOWN_MISSING_GRACE_S = 10.0


class DynamicEventManager:
    def __init__(
        self,
        traci,
        net: NetSumo,
        planes: Dict[str, Dict[str, Any]],
        report_path: Optional[Path] = None,
        min_affected_vehicles: int = 2,
        event_callback=None,
        plan_index_resolver=None,
    ) -> None:
        self.traci = traci
        self.net = net
        self.planes = planes
        self.min_affected_vehicles = max(1, int(min_affected_vehicles))
        self.event_callback = event_callback
        # El runner conoce cómo traducir el routeIndex físico de SUMO al
        # índice lógico del plan, especialmente después de un rescheduling.
        # E1 usa el mismo resolvedor para no confundir VIAJE_VACIO/VIAJE_CARGADO.
        self.plan_index_resolver = plan_index_resolver
        # active_events se conserva exclusivamente para los derrumbes, porque
        # _enforce_closures() trabaja con edges bloqueados.
        self.active_events: Dict[str, Dict[str, Any]] = {}
        self.finished_events: Set[str] = set()

        # Estados de las nuevas averias.
        self.active_truck_breakdowns: Dict[str, Dict[str, Any]] = {}
        self.active_shovel_breakdowns: Dict[str, Dict[str, Any]] = {}
        self.adjusted_loads: Set[Tuple[Any, ...]] = set()
        # Notificaciones consumidas por el runner cuando una reparacion termina.
        self._repaired_trucks: List[Dict[str, Any]] = []

        # Diagnóstico de E3: evita imprimir SIN_CANDIDATO en cada step y permite
        # distinguir entre "nunca hubo candidato" y "hubo candidato pero falló
        # la inserción del stop".
        self.truck_no_candidate: Set[str] = set()
        self.truck_candidate_seen: Set[str] = set()

        self.report_path = Path(report_path) if report_path else None
        self._report_file = None
        self._report_writer = None
        self._last_sim_time = 0.0

        if self.report_path:
            self.report_path.parent.mkdir(parents=True, exist_ok=True)
            self._report_file = open(self.report_path, "w", newline="", encoding="utf-8-sig")
            self._report_writer = csv.DictWriter(
                self._report_file,
                fieldnames=[
                    "tiempo_s", "evento_id", "tipo", "accion", "edge",
                    "camion", "destino_segmento", "ruta_anterior", "ruta_nueva",
                    "resultado", "detalle",
                ],
            )
            self._report_writer.writeheader()

    def close(self) -> None:
        # Si E3 alcanzó a insertar un PA_SAFE pero la simulación terminó antes de
        # que el camión llegara y comenzara la reparación, dejarlo explícito.
        for event_id, state in list(self.active_truck_breakdowns.items()):
            if state.get("repair_start_s") is not None:
                continue
            veh_id = str(state.get("truck", ""))
            parking_id = str(state.get("parking_id", ""))
            parking_edge = str(state.get("parking_edge", ""))
            self._write_report(
                self._last_sim_time, event_id, "NO_DETENCION",
                parking_edge, veh_id, parking_id,
                "", "", "PA_SAFE_NO_ALCANZADO",
                "La simulacion termino antes de que el camion alcanzara el PA_SAFE y comenzara la reparacion.",
                event_type="averia_camion",
            )
            print(
                f"[averia_camion] {event_id} NO_DETENCION / PA_SAFE_NO_ALCANZADO "
                f"camion={veh_id} t={self._last_sim_time:.1f}s"
            )

        if self._report_file:
            self._report_file.close()
            self._report_file = None
            self._report_writer = None

    def get_truck_breakdown(self, veh_id: str) -> Optional[Dict[str, Any]]:
        """Retorna una copia del estado de averia activo del camion."""
        veh_id = str(veh_id or "").strip()
        for event_id, state in self.active_truck_breakdowns.items():
            if str(state.get("truck", "")) == veh_id:
                result = dict(state)
                result["event_id"] = str(event_id)
                return result
        return None

    def pop_repaired_trucks(self) -> List[Dict[str, Any]]:
        """Entrega una sola vez las reparaciones terminadas al runner."""
        repaired = list(self._repaired_trucks)
        self._repaired_trucks.clear()
        return repaired

    def update(self, sim_time: float, configured_events: Iterable[Dict[str, Any]]) -> None:
        """Activa, mantiene y finaliza eventos segun el tiempo de SUMO."""
        sim_time = float(sim_time)
        self._last_sim_time = sim_time

        for event in configured_events:
            event_type = str(event.get("tipo", "")).strip().lower()
            event_id = str(event.get("id", "EVENTO")).strip() or "EVENTO"
            start = float(event.get("ini", 0.0) or 0.0)
            end = float(event.get("fin", 0.0) or 0.0)

            if event_type == "derrumbe":
                if event_id not in self.active_events and event_id not in self.finished_events:
                    if start <= sim_time < end:
                        self._activate_collapse(event, sim_time)

                if event_id in self.active_events and sim_time >= end:
                    self._deactivate_collapse(event_id, sim_time)
                continue

            if event_type in ("averia_camion", "falla_camion"):
                if (
                    event_id not in self.active_truck_breakdowns
                    and event_id not in self.finished_events
                    and start <= sim_time < end
                ):
                    self._activate_truck(event, sim_time)

                # Si terminó toda la ventana y jamás se consiguió insertar la
                # detención, dejar evidencia explícita en consola y CSV.
                if (
                    event_id not in self.active_truck_breakdowns
                    and event_id not in self.finished_events
                    and sim_time >= end
                ):
                    self.finished_events.add(event_id)
                    if event_id in self.truck_candidate_seen:
                        resultado = "ERROR_INSERCION_STOP"
                        detalle = (
                            "E3 termino sin detencion: hubo al menos un camion candidato, "
                            "pero no se pudo insertar un PA_SAFE durante la ventana."
                        )
                    else:
                        resultado = "SIN_CANDIDATO"
                        detalle = (
                            "E3 termino sin detencion: no existio ningun camion circulando "
                            "con un PA_SAFE valido por delante durante la ventana."
                        )

                    self._write_report(
                        sim_time, event_id, "NO_ACTIVADO", "", "", "",
                        "", "", resultado, detalle,
                        event_type="averia_camion",
                    )
                    print(
                        f"[averia_camion] {event_id} NO_ACTIVADO / {resultado} "
                        f"t={sim_time:.1f}s"
                    )
                continue

            if event_type in ("averia_pala", "falla_pala"):
                if (
                    event_id not in self.active_shovel_breakdowns
                    and event_id not in self.finished_events
                    and start <= sim_time < end
                ):
                    self._activate_shovel(event, sim_time)

                if event_id in self.active_shovel_breakdowns and sim_time >= end:
                    self._deactivate_shovel(event_id, sim_time)
                continue

        # Los camiones con averia se actualizan aunque el intervalo nominal del CSV
        # haya terminado, porque la hora real de reparación comienza al llegar al PA_SAFE.
        self._update_trucks(sim_time)

        # Mientras una pala se encuentra degradada, cualquier CARGA que realmente
        # comience en esa pala recibe el factor de duración configurado.
        self._update_shovels(sim_time)

        # Mientras exista un derrumbe activo, comprobar que ninguna ruta vuelva
        # a incorporar el edge bloqueado.
        if self.active_events:
            self._enforce_closures(sim_time)

    # ------------------------------------------------------------------
    # Averia de camion
    # ------------------------------------------------------------------

    @staticmethod
    def _event_seed(event: Dict[str, Any], default: int = 2026) -> int:
        try:
            seed = int(float(str(event.get("seed", default)).replace(",", ".")))
        except Exception:
            seed = int(default)
        event_id = str(event.get("id", "EVENTO"))
        # Desplazamiento estable; no usa hash() porque cambia entre procesos.
        return seed + sum((i + 1) * ord(ch) for i, ch in enumerate(event_id))

    def _parking_edge(self, parking_id: str) -> str:
        parking_id = str(parking_id or "").strip()
        if not parking_id:
            return ""
        try:
            lane = str(self.traci.parkingarea.getLaneID(parking_id) or "").strip()
        except Exception:
            return ""
        if not lane:
            return ""
        try:
            # routing_service ya conoce el mapeo lane -> edge.
            return normalizar_id(self.net.lane_to_edge.get(lane, ""))
        except Exception:
            pass
        # Respaldo para lanes con sufijo _0, _1, ...
        if "_" in lane:
            return normalizar_id(lane.rsplit("_", 1)[0])
        return normalizar_id(lane)

    def _stop_edge(self, stop_data: Any) -> str:
        parking_id = str(getattr(stop_data, "stoppingPlaceID", "") or "").strip()
        if parking_id:
            edge = self._parking_edge(parking_id)
            if edge:
                return edge
        lane = str(getattr(stop_data, "lane", "") or "").strip()
        if lane:
            try:
                return normalizar_id(self.net.lane_to_edge.get(lane, ""))
            except Exception:
                pass
            if "_" in lane:
                return normalizar_id(lane.rsplit("_", 1)[0])
        return ""

    def _route_position(
        self,
        route: List[str],
        edge_id: str,
        start_idx: int,
    ) -> Optional[int]:
        edge_id = normalizar_id(edge_id)
        for idx in range(max(0, int(start_idx)), len(route)):
            if normalizar_id(route[idx]) == edge_id:
                return idx
        return None

    def _safe_wait_options(self, veh_id: str) -> List[Dict[str, Any]]:
        """
        Busca PA_SAFE realmente ubicados por delante del camión sobre su ruta pendiente.

        Regla importante: si un PA_SAFE está en el edge actual, no basta con que el
        edge coincida con route_index. También se compara la posición longitudinal
        del camión con el endPos del parking. Si el camión ya sobrepasó el PA_SAFE,
        esa ocurrencia del edge se descarta y se busca la siguiente aparición del
        mismo edge dentro de la ruta. Esto evita que SUMO espere una vuelta completa
        para alcanzar un PA_SAFE que físicamente ya quedó atrás.

        El PA_SAFE elegido debe quedar antes de al menos una parada operacional futura,
        de modo que, tras la reparación, el camión pueda continuar su ciclo original.
        """
        try:
            route = list(self.traci.vehicle.getRoute(veh_id))
            route_index = int(self.traci.vehicle.getRouteIndex(veh_id))
            stops = list(self.traci.vehicle.getStops(veh_id))
            current_edge = normalizar_id(self.traci.vehicle.getRoadID(veh_id))
            current_lane_pos = float(self.traci.vehicle.getLanePosition(veh_id))
        except Exception:
            return []

        if route_index < 0 or not route or not stops:
            return []

        # Posición de cada stop futuro en la ruta, respetando el orden.
        stop_positions: List[Optional[int]] = []
        cursor = route_index
        existing_parking_ids: Set[str] = set()
        for st in stops:
            parking_id = str(getattr(st, "stoppingPlaceID", "") or "").strip()
            if parking_id:
                existing_parking_ids.add(parking_id)
            edge_stop = self._stop_edge(st)
            pos = self._route_position(route, edge_stop, cursor) if edge_stop else None
            stop_positions.append(pos)
            if pos is not None:
                cursor = pos

        options: List[Dict[str, Any]] = []
        try:
            parking_ids = list(self.traci.parkingarea.getIDList())
        except Exception:
            parking_ids = []

        for parking_id in parking_ids:
            parking_id = str(parking_id or "").strip()
            if not parking_id.startswith("PA_SAFE_"):
                continue
            if parking_id in existing_parking_ids:
                # Un PA_SAFE ya insertado no se vuelve a insertar.
                continue

            edge_wait = self._parking_edge(parking_id)
            if not edge_wait:
                continue

            pos = self._route_position(route, edge_wait, route_index)
            if pos is None:
                continue

            parking_start = None
            parking_end = None
            try:
                parking_start = float(self.traci.parkingarea.getStartPos(parking_id))
                parking_end = float(self.traci.parkingarea.getEndPos(parking_id))
            except Exception:
                # En SUMO 1.26 estos getters existen. Si por compatibilidad no
                # estuvieran disponibles, no se arriesga a considerar como válido
                # un PA_SAFE situado en el edge actual cuya posición no puede comprobarse.
                parking_start = None
                parking_end = None

            # Si el primer candidato corresponde al edge que el camión ocupa ahora,
            # comprobar si el PA_SAFE todavía está físicamente por delante.
            if int(pos) == int(route_index) and edge_wait == current_edge:
                if parking_end is None:
                    # No se puede demostrar que siga por delante: buscar la siguiente
                    # ocurrencia del mismo edge en vez de reutilizar la actual.
                    pos = self._route_position(route, edge_wait, route_index + 1)
                    if pos is None:
                        continue
                elif current_lane_pos > parking_end + 0.10:
                    # El camión ya superó completamente el PA_SAFE en este edge.
                    # Si la ruta vuelve a pasar por el mismo edge, usar esa próxima
                    # ocurrencia; si no, este PA_SAFE deja de ser candidato.
                    pos = self._route_position(route, edge_wait, route_index + 1)
                    if pos is None:
                        continue

            # Insertar después de los stops que ocurren en el mismo edge o antes.
            insert_index = sum(
                1 for p in stop_positions
                if p is not None and int(p) <= int(pos)
            )

            # Debe quedar al menos un stop posterior para que el camión vuelva
            # a su operación luego de la reparación.
            if insert_index >= len(stops):
                continue

            options.append({
                "parking_id": parking_id,
                "edge": edge_wait,
                "route_pos": int(pos),
                "insert_index": int(insert_index),
                "parking_start": parking_start,
                "parking_end": parking_end,
                "current_edge": current_edge,
                "current_lane_pos": current_lane_pos,
            })

        # Primero el PA_SAFE de la ocurrencia de ruta más cercana. Si está en el
        # edge actual, se prioriza el que empieza antes pero aún permanece delante.
        options.sort(key=lambda item: (
            item["route_pos"],
            float(item["parking_start"]) if item.get("parking_start") is not None else float("inf"),
            item["parking_id"],
        ))
        return options

    def _truck_candidates(self) -> List[Dict[str, Any]]:
        candidates: List[Dict[str, Any]] = []
        try:
            vehicles = sorted(self.traci.vehicle.getIDList())
        except Exception:
            return candidates

        already_broken = {
            str(state.get("truck", ""))
            for state in self.active_truck_breakdowns.values()
        }

        for veh_id in vehicles:
            if not str(veh_id).startswith("Truck"):
                continue
            if veh_id in already_broken or veh_id not in self.planes:
                continue
            try:
                # No elegir un camión que ya esté detenido en CARGA/DESCARGA/PA_WAIT/PA_SAFE.
                if self.traci.vehicle.isStoppedParking(veh_id):
                    continue
                if bool(self.traci.vehicle.getStopState(veh_id)):
                    continue
            except Exception:
                pass

            options = self._safe_wait_options(veh_id)
            if options:
                candidates.append({
                    "truck": veh_id,
                    "wait_options": options,
                })
        return candidates

    def _activate_truck(self, event: Dict[str, Any], sim_time: float) -> None:
        event_id = str(event.get("id", "AVERIA_CAMION")).strip() or "AVERIA_CAMION"
        candidates = self._truck_candidates()
        if not candidates:
            # No se marca como finalizado todavía: puede existir un camión elegible
            # en un step posterior mientras el intervalo del evento siga abierto.
            if event_id not in self.truck_no_candidate:
                self.truck_no_candidate.add(event_id)
                print(
                    f"[averia_camion] {event_id} t={sim_time:.1f}s "
                    "SIN_CANDIDATO - se seguira intentando"
                )
            return

        self.truck_candidate_seen.add(event_id)
        rng = random.Random(self._event_seed(event))
        selected = rng.choice(candidates)
        veh_id = str(selected["truck"])
        option = selected["wait_options"][0]

        try:
            duration_s = float(event.get("duracion_s", 0.0) or 0.0)
        except Exception:
            duration_s = 0.0
        if duration_s <= 0:
            duration_s = max(
                1.0,
                float(event.get("fin", sim_time) or sim_time)
                - float(event.get("ini", sim_time) or sim_time),
            )

        # Flags TraCI: parking + parkingArea. Se obtienen desde el módulo TraCI
        # recibido por el manager; se dejan valores de respaldo para compatibilidad.
        tc = getattr(self.traci, "constants", None)
        stop_parking = int(getattr(tc, "STOP_PARKING", 1)) if tc is not None else 1
        stop_parking_area = int(getattr(tc, "STOP_PARKING_AREA", 64)) if tc is not None else 64
        flags = stop_parking | stop_parking_area

        try:
            self.traci.vehicle.insertStop(
                veh_id,
                int(option["insert_index"]),
                str(option["parking_id"]),
                duration=float(duration_s),
                flags=flags,
                teleport=0,
            )
        except Exception as exc:
            # Dejar el evento disponible para un nuevo intento durante su ventana.
            print(f"[averia_camion] {event_id} no pudo insertar PA_SAFE para {veh_id}: {exc}")
            return

        self.active_truck_breakdowns[event_id] = {
            "event": deepcopy(event),
            "truck": veh_id,
            "parking_id": str(option["parking_id"]),
            "parking_edge": str(option["edge"]),
            "duration_s": float(duration_s),
            "detected_s": float(sim_time),
            "repair_start_s": None,
            "repair_end_s": None,
            "missing_since_s": None,
        }

        self._write_report(
            sim_time, event_id, "AVERIA_DETECTADA", str(option["edge"]), veh_id,
            str(option["parking_id"]), "", "", "OK",
            f"camion_seleccionado={veh_id};PA_SAFE={option['parking_id']};"
            f"route_pos_safe={option.get('route_pos')};"
            f"pos_camion_m={float(option.get('current_lane_pos', 0.0)):.2f};"
            f"safe_start_m={option.get('parking_start')};safe_end_m={option.get('parking_end')};"
            f"duracion_reparacion_s={duration_s:.1f};seleccion=random_reproducible",
            event_type="averia_camion",
        )
        print(
            f"[averia_camion] {event_id} t={sim_time:.1f}s camion={veh_id} "
            f"-> {option['parking_id']} reparacion={duration_s:.1f}s"
        )

    def _update_trucks(self, sim_time: float) -> None:
        for event_id, state in list(self.active_truck_breakdowns.items()):
            veh_id = str(state.get("truck", ""))
            parking_id = str(state.get("parking_id", ""))
            duration_s = float(state.get("duration_s", 0.0) or 0.0)

            try:
                alive = veh_id in set(self.traci.vehicle.getIDList())
            except Exception:
                alive = False

            if not alive:
                missing_since = state.get("missing_since_s")
                if missing_since is None:
                    state["missing_since_s"] = float(sim_time)
                    print(
                        f"[averia_camion] {event_id} {veh_id} ausente temporalmente "
                        f"en t={sim_time:.1f}s; se esperaran "
                        f"{TRUCK_BREAKDOWN_MISSING_GRACE_S:.1f}s antes de finalizar la averia"
                    )
                    continue

                missing_duration = float(sim_time) - float(missing_since)
                if missing_duration < TRUCK_BREAKDOWN_MISSING_GRACE_S:
                    continue

                self.active_truck_breakdowns.pop(event_id, None)
                self.finished_events.add(event_id)
                self._write_report(
                    sim_time, event_id, "FINALIZAR", "", veh_id, parking_id,
                    "", "", "VEHICULO_NO_DISPONIBLE_CONFIRMADO",
                    f"El camion permanecio ausente de SUMO durante {missing_duration:.1f}s "
                    "antes de completar la reparacion",
                    event_type="averia_camion",
                )
                continue

            # Una ausencia breve durante el cambio de operacion no cancela E3.
            if state.get("missing_since_s") is not None:
                print(
                    f"[averia_camion] {event_id} {veh_id} reaparecio en "
                    f"t={sim_time:.1f}s; la averia continua activa"
                )
                state["missing_since_s"] = None

            at_parking = False
            try:
                stops = list(self.traci.vehicle.getStops(veh_id))
                current_parking = (
                    str(getattr(stops[0], "stoppingPlaceID", "") or "").strip()
                    if stops else ""
                )
                at_parking = bool(
                    self.traci.vehicle.isStoppedParking(veh_id)
                    and current_parking == parking_id
                )
            except Exception:
                current_parking = ""

            if state.get("repair_start_s") is None and at_parking:
                state["repair_start_s"] = float(sim_time)
                state["repair_end_s"] = float(sim_time) + duration_s
                self._write_report(
                    sim_time, event_id, "REPARACION_INICIADA",
                    str(state.get("parking_edge", "")), veh_id, parking_id,
                    "", "", "OK",
                    f"fin_estimado_s={state['repair_end_s']:.1f}",
                    event_type="averia_camion",
                )
                print(
                    f"[averia_camion] {event_id} {veh_id} inicio reparacion "
                    f"t={sim_time:.1f}s fin_estimado={state['repair_end_s']:.1f}s"
                )
                continue

            repair_end = state.get("repair_end_s")
            if repair_end is None:
                continue

            # insertStop libera al vehículo automáticamente al cumplir duration.
            # Se cierra el evento cuando ya salió del PA_SAFE tras el tiempo previsto.
            if float(sim_time) + 1e-9 >= float(repair_end) and not at_parking:
                repaired_info = {
                    "event_id": str(event_id),
                    "truck": veh_id,
                    "parking_id": parking_id,
                    "parking_edge": str(state.get("parking_edge", "")),
                    "repair_start_s": float(state.get("repair_start_s", sim_time)),
                    "repair_end_s": float(repair_end),
                    "repaired_s": float(sim_time),
                }
                self.active_truck_breakdowns.pop(event_id, None)
                self.finished_events.add(event_id)
                self._repaired_trucks.append(repaired_info)
                self._write_report(
                    sim_time, event_id, "REPARADO",
                    str(state.get("parking_edge", "")), veh_id, parking_id,
                    "", "", "OK",
                    "El camion abandono el PA_SAFE y retomo su ruta programada",
                    event_type="averia_camion",
                )
                print(
                    f"[averia_camion] {event_id} {veh_id} REPARADO "
                    f"t={sim_time:.1f}s; continua operacion"
                )

    # ------------------------------------------------------------------
    # Averia parcial de pala: carga mas lenta
    # ------------------------------------------------------------------

    @staticmethod
    def _segment_destination(seg: Dict[str, Any]) -> str:
        ruta = str(seg.get("ruta", "") or "").strip()
        if "->" in ruta:
            return normalizar_id(ruta.split("->", 1)[1])
        return normalizar_id(
            seg.get("destinoRuta", seg.get("ubicacion", seg.get("edge_fin", "")))
        )

    def _candidate_shovels(self) -> List[str]:
        """
        Palas que todavía aparecen como destino de un VIAJE_VACIO en los planes
        activos. De esta forma la selección aleatoria tiene efecto observable.
        """
        shovels: Set[str] = set()
        try:
            live = set(self.traci.vehicle.getIDList())
        except Exception:
            live = set(self.planes.keys())

        for veh_id in sorted(live):
            plan = self.planes.get(veh_id)
            if not plan:
                continue

            start_seg = 0
            info = self._active_segment_info(veh_id)
            if info:
                start_seg = int(info.get("seg_idx", 0))

            for seg in plan.get("segmentos", [])[start_seg:]:
                if str(seg.get("operacion", "") or "").upper() != "VIAJE_VACIO":
                    continue
                dest = self._segment_destination(seg)
                if dest.upper().startswith("PA"):
                    shovels.add(dest)
        return sorted(shovels)

    def _activate_shovel(self, event: Dict[str, Any], sim_time: float) -> None:
        event_id = str(event.get("id", "AVERIA_PALA")).strip() or "AVERIA_PALA"
        shovels = self._candidate_shovels()
        if not shovels:
            return

        objetivo = str(event.get("objetivo", "random") or "random").strip()
        if objetivo and objetivo.lower() not in ("random", "aleatorio", "auto", "automatico"):
            selected = normalizar_id(objetivo)
            if selected not in shovels:
                return
        else:
            rng = random.Random(self._event_seed(event))
            selected = rng.choice(shovels)

        try:
            factor = float(event.get("factor_carga", 1.5) or 1.5)
        except Exception:
            factor = 1.5
        factor = max(1.0, factor)

        self.active_shovel_breakdowns[event_id] = {
            "event": deepcopy(event),
            "shovel": selected,
            "factor_carga": factor,
            "start_s": float(sim_time),
            "end_s": float(event.get("fin", sim_time) or sim_time),
        }

        self._write_report(
            sim_time, event_id, "DEGRADACION_INICIADA", "", "",
            selected, "", "", "OK",
            f"pala_seleccionada={selected};factor_carga={factor:.3f};"
            "seleccion=random_reproducible",
            event_type="averia_pala",
        )
        print(
            f"[averia_pala] {event_id} t={sim_time:.1f}s pala={selected} "
            f"factor_carga={factor:.3f}"
        )

    def _current_load_identity(
        self,
        veh_id: str,
        event_id: str,
    ) -> Tuple[Optional[Tuple[Any, ...]], Optional[Dict[str, Any]]]:
        info = self._active_segment_info(veh_id)
        if not info:
            return None, None
        seg = info.get("seg", {})
        if str(seg.get("operacion", "") or "").upper() != "VIAJE_VACIO":
            return None, None

        identity = (
            event_id,
            veh_id,
            int(seg.get("cycle_id", 0) or 0),
            int(seg.get("secuencia_ciclo", 0) or 0),
            int(seg.get("seg_num", info.get("seg_idx", -1)) or -1),
            int(seg.get("depart_ms", 0) or 0),
        )
        return identity, seg

    def _update_shovels(self, sim_time: float) -> None:
        if not self.active_shovel_breakdowns:
            return

        try:
            vehicles = list(self.traci.vehicle.getIDList())
        except Exception:
            vehicles = []

        for event_id, state in list(self.active_shovel_breakdowns.items()):
            selected = str(state.get("shovel", ""))
            factor = float(state.get("factor_carga", 1.0) or 1.0)
            prefix = f"PA_AUTO_{selected}_"

            for veh_id in vehicles:
                try:
                    if not self.traci.vehicle.isStoppedParking(veh_id):
                        continue
                    stops = list(self.traci.vehicle.getStops(veh_id))
                except Exception:
                    continue
                if not stops:
                    continue

                parking_id = str(
                    getattr(stops[0], "stoppingPlaceID", "") or ""
                ).strip()
                if not parking_id.startswith(prefix):
                    continue

                load_key, seg = self._current_load_identity(veh_id, event_id)
                if load_key is None or seg is None or load_key in self.adjusted_loads:
                    continue

                # Base del scheduling JADE/SUMO. El factor modifica la duración
                # física de la CARGA, no el cronograma original.
                try:
                    base_duration = float(seg.get("stop_after_duration_s", 0.0) or 0.0)
                except Exception:
                    base_duration = 0.0
                if base_duration <= 0:
                    try:
                        base_duration = float(getattr(stops[0], "duration", 0.0) or 0.0)
                    except Exception:
                        base_duration = 0.0
                if base_duration <= 0:
                    continue

                new_duration = base_duration * factor
                try:
                    self.traci.vehicle.setStopParameter(
                        veh_id, 0, "duration", f"{new_duration:.3f}"
                    )
                except Exception as exc:
                    print(
                        f"[averia_pala] no se pudo prolongar CARGA de {veh_id} "
                        f"en {selected}: {exc}"
                    )
                    continue

                self.adjusted_loads.add(load_key)
                self._write_report(
                    sim_time, event_id, "CARGA_RALENTIZADA", "", veh_id,
                    selected, "", "", "OK",
                    f"duracion_base_s={base_duration:.3f};"
                    f"factor={factor:.3f};duracion_averia_s={new_duration:.3f}",
                    event_type="averia_pala",
                )
                print(
                    f"[averia_pala] {event_id} {selected} camion={veh_id} "
                    f"carga {base_duration:.1f}s -> {new_duration:.1f}s"
                )

    def _deactivate_shovel(self, event_id: str, sim_time: float) -> None:
        state = self.active_shovel_breakdowns.pop(event_id, None)
        if not state:
            return
        selected = str(state.get("shovel", ""))
        self.finished_events.add(event_id)
        self._write_report(
            sim_time, event_id, "DEGRADACION_FINALIZADA", "", "",
            selected, "", "", "OK",
            "Las nuevas operaciones de CARGA vuelven a su duracion normal",
            event_type="averia_pala",
        )
        print(
            f"[averia_pala] {event_id} FINALIZADO t={sim_time:.1f}s "
            f"pala={selected}; carga normal"
        )

    # ------------------------------------------------------------------
    # Seleccion y validacion
    # ------------------------------------------------------------------

    def _active_segment_info(self, veh_id: str) -> Optional[Dict[str, Any]]:
        """
        Obtiene el segmento operacional real que el camión está ejecutando.

        Después de setRoute() durante el rescheduling, el routeIndex físico de SUMO
        puede no coincidir con los índices del plan. Por eso, si el runner entregó
        plan_index_resolver, se usa exactamente la misma traducción que utiliza el
        runner para velocidades, estados y cierre de ciclos.
        """
        plan = self.planes.get(veh_id)
        if not plan:
            return None

        try:
            route_index_sumo = int(self.traci.vehicle.getRouteIndex(veh_id))
            route = list(self.traci.vehicle.getRoute(veh_id))
            edge_actual = normalizar_id(self.traci.vehicle.getRoadID(veh_id))
        except Exception:
            return None

        if route_index_sumo < 0 or not route:
            return None

        # Índice lógico del plan activo. En scheduling inicial coincide con SUMO;
        # en rescheduling se resuelve mediante el mapa que mantiene el runner.
        route_index_plan = route_index_sumo
        if self.plan_index_resolver is not None:
            try:
                route_index_plan = self.plan_index_resolver(
                    self.traci,
                    veh_id,
                    plan,
                    route_index_sumo=route_index_sumo,
                    edge_actual=edge_actual,
                )
            except Exception:
                return None

        if route_index_plan is None:
            return None
        try:
            route_index_plan = int(route_index_plan)
        except Exception:
            return None

        seg_idx = plan.get("index_to_segment", {}).get(route_index_plan)
        if seg_idx is None or seg_idx < 0 or seg_idx >= len(plan.get("segmentos", [])):
            return None

        seg = plan["segmentos"][seg_idx]
        destination_edge = normalizar_id(seg.get("edge_fin", ""))
        if not destination_edge:
            return None

        # Determinar el fin FÍSICO del segmento dentro de la ruta actual de SUMO.
        # Primero se aprovecha el mapa creado por el runner para rutas de rescheduling.
        physical_end_idx = None
        applied_edges = list(plan.get("_ruta_aplicada_edges", []) or [])
        applied_map = list(plan.get("_ruta_aplicada_plan_indices", []) or [])
        try:
            base_sumo = int(plan.get("_route_index_base_sumo", route_index_sumo))
        except Exception:
            base_sumo = route_index_sumo

        pos_rel = route_index_sumo - base_sumo
        if (
            applied_edges
            and applied_map
            and len(applied_edges) == len(applied_map)
            and 0 <= pos_rel < len(applied_map)
        ):
            last_rel = None
            for rel in range(pos_rel, len(applied_map)):
                mapped_idx = applied_map[rel]
                mapped_seg = plan.get("index_to_segment", {}).get(mapped_idx)
                if mapped_seg == seg_idx:
                    last_rel = rel
                elif last_rel is not None:
                    break
            if last_rel is not None:
                candidate = base_sumo + last_rel
                if 0 <= candidate < len(route):
                    physical_end_idx = candidate

        # Respaldo: buscar el destino operacional del segmento hacia delante.
        if physical_end_idx is None:
            for idx in range(route_index_sumo, len(route)):
                if normalizar_id(route[idx]) == destination_edge:
                    physical_end_idx = idx
                    break

        if physical_end_idx is None or physical_end_idx < route_index_sumo:
            return None

        return {
            "veh_id": veh_id,
            "plan": plan,
            "route": route,
            "route_index": route_index_sumo,
            "route_index_sumo": route_index_sumo,
            "route_index_plan": route_index_plan,
            "seg_idx": seg_idx,
            "seg": seg,
            "end_idx": physical_end_idx,
            "pending_segment": route[route_index_sumo : physical_end_idx + 1],
            "remaining_route": route[route_index_sumo:],
            "destination_edge": destination_edge,
        }

    @staticmethod
    def _eligible_edge(edge_id: str, info: Dict[str, Any]) -> bool:
        edge_id = normalizar_id(edge_id)
        if not edge_id or edge_id.startswith(":"):
            return False
        pending = info["pending_segment"]
        # No bloquear el edge donde ya esta el camion ni el edge destino del segmento.
        if edge_id == pending[0] or edge_id == info["destination_edge"]:
            return False
        return True

    def _candidate_usage(self) -> Tuple[Counter, Dict[str, Dict[str, Any]]]:
        """
        Cuenta solo los edges del SEGMENTO ACTUAL de cada camión.

        E1 ya no analiza ni reconstruye ciclos futuros. Esto evita que un derrumbe
        altere la semántica del scheduling/rescheduling (pala, botadero, operación
        o índices de segmentos posteriores).
        """
        usage: Counter = Counter()
        infos: Dict[str, Dict[str, Any]] = {}

        broken_trucks = {
            str(state.get("truck", ""))
            for state in self.active_truck_breakdowns.values()
        }

        for veh_id in self.traci.vehicle.getIDList():
            if veh_id in broken_trucks:
                # Un camión que va a PA_SAFE o está reparándose no se modifica por E1.
                continue

            info = self._active_segment_info(veh_id)
            if not info:
                continue

            infos[veh_id] = info
            for edge_id in set(info.get("pending_segment", [])):
                if self._eligible_edge(edge_id, info):
                    usage[edge_id] += 1

        return usage, infos

    def _build_detour(
        self,
        info: Dict[str, Any],
        blocked_edge: str,
    ) -> Tuple[List[str], List[Dict[str, Any]], str]:
        """
        Desvía ÚNICAMENTE el segmento que el camión está ejecutando.

        La ruta física nueva se construye como:
            desvío desde edge actual -> MISMO destino del segmento actual
            + sufijo original posterior a ese segmento.

        Los segmentos futuros no se recalculan, no se cambian sus destinos y no
        se modifica seg["ruta"]. Así E1 no puede transformar VIAJE_VACIO en
        VIAJE_CARGADO ni alterar un ciclo de rescheduling.
        """
        route = list(info.get("route", []))
        plan = info.get("plan", {})
        current_route_index = int(info.get("route_index", -1))
        current_plan_index = int(info.get("route_index_plan", current_route_index))
        physical_end_idx = int(info.get("end_idx", -1))
        current_seg_idx = int(info.get("seg_idx", -1))
        destination = normalizar_id(info.get("destination_edge", ""))

        if not route or current_route_index < 0 or current_route_index >= len(route):
            return [], [], "ROUTE_INDEX_FISICO_FUERA_DE_RANGO"
        if physical_end_idx < current_route_index or physical_end_idx >= len(route):
            return [], [], "FIN_SEGMENTO_FISICO_FUERA_DE_RANGO"
        if not destination:
            return [], [], "DESTINO_SEGMENTO_INVALIDO"

        origin = normalizar_id(route[current_route_index])
        if not origin:
            return [], [], "ORIGEN_ACTUAL_INVALIDO"

        detour, status = self.net.ruta_entre_edges(
            origin,
            destination,
            excluded_edges={normalizar_id(blocked_edge)},
        )
        detour = [normalizar_id(e) for e in (detour or []) if normalizar_id(e)]
        if not detour or normalizar_id(blocked_edge) in detour:
            return [], [], status or "SIN_DESVIO_SEGMENTO_ACTUAL"

        # Debe arrancar desde el edge físico donde está el vehículo.
        if detour[0] != origin:
            return [], [], f"DESVIO_NO_PARTE_EN_EDGE_ACTUAL:{detour[0]}!={origin}"

        # El resto de la ruta se conserva EXACTAMENTE como estaba después del
        # destino del segmento actual. Solo se evita duplicar el edge de empalme.
        suffix = list(route[physical_end_idx + 1:])
        if suffix and detour and suffix[0] == detour[-1]:
            suffix = suffix[1:]

        new_route = list(detour) + list(suffix)
        if normalizar_id(blocked_edge) in detour:
            return [], [], "EDGE_BLOQUEADO_PERMANECE_EN_DESVIO_ACTUAL"

        # Construir el mapa físico -> índice lógico sin tocar los mapas semánticos
        # originales del plan.
        old_applied_edges = list(plan.get("_ruta_aplicada_edges", []) or [])
        old_applied_map = list(plan.get("_ruta_aplicada_plan_indices", []) or [])
        try:
            old_base = int(plan.get("_route_index_base_sumo", 0))
        except Exception:
            old_base = 0

        def logical_old(physical_idx: int) -> int:
            if old_applied_edges and old_applied_map and len(old_applied_edges) == len(old_applied_map):
                rel = int(physical_idx) - int(old_base)
                if 0 <= rel < len(old_applied_map):
                    return int(old_applied_map[rel])
            # Scheduling inicial: índice SUMO == índice lógico.
            return int(physical_idx)

        try:
            seg = plan.get("segmentos", [])[current_seg_idx]
            logical_seg_end = int(seg.get("end_idx", current_plan_index))
        except Exception:
            logical_seg_end = int(current_plan_index)

        new_plan_map: List[int] = []
        # Todo el desvío sigue perteneciendo al mismo segmento actual.
        for pos in range(len(detour)):
            if pos == len(detour) - 1:
                new_plan_map.append(logical_seg_end)
            else:
                new_plan_map.append(current_plan_index)

        # Los edges futuros recuperan su índice lógico original.
        suffix_start_old = physical_end_idx + 1
        # Si se eliminó un duplicado al empalmar, avanzar uno también en el origen.
        if route[physical_end_idx + 1:physical_end_idx + 2] and detour and route[physical_end_idx + 1] == detour[-1]:
            suffix_start_old += 1

        for old_phys in range(suffix_start_old, len(route)):
            new_plan_map.append(logical_old(old_phys))

        if len(new_plan_map) != len(new_route):
            return [], [], "MAPA_RUTA_DESVIO_INCONSISTENTE"

        rebuilt = [{
            "seg_idx": current_seg_idx,
            "route": list(detour),
            "start_idx": 0,
            "end_idx": len(detour) - 1,
            "destination_edge": destination,
            "plan_map": new_plan_map,
        }]

        # Se valida conectividad física, pero NO se exige que el edge bloqueado
        # desaparezca de ciclos futuros: si vuelve a aparecer mientras E1 siga
        # activo, la vigilancia lo desviará cuando ese segmento sea el actual.
        ok, detail = self.net.validar_ruta_excluida(detour, {normalizar_id(blocked_edge)})
        if not ok:
            return [], [], detail
        return new_route, rebuilt, "OK"

    def _find_valid_candidate(
        self,
    ) -> Tuple[Optional[str], Dict[str, Dict[str, Any]], str]:
        """
        Busca un edge compartido por al menos min_affected_vehicles, considerando
        solamente los segmentos que están ejecutándose en ese instante.
        """
        usage, infos = self._candidate_usage()
        if not usage:
            return None, {}, "SIN_EDGES_CANDIDATOS_EN_SEGMENTOS_ACTUALES"

        ordered = sorted(usage.items(), key=lambda item: (-item[1], item[0]))
        diagnostics: List[str] = []

        for edge_id, count in ordered:
            if count < self.min_affected_vehicles:
                diagnostics.append(f"{edge_id}:solo_{count}_camion(es)")
                continue

            affected_infos = {
                veh_id: info
                for veh_id, info in infos.items()
                if edge_id in info.get("pending_segment", [])
            }
            if len(affected_infos) < self.min_affected_vehicles:
                diagnostics.append(
                    f"{edge_id}:solo_{len(affected_infos)}_segmento(s)_actual(es)"
                )
                continue

            occupying = [
                veh_id for veh_id, info in affected_infos.items()
                if normalizar_id(info.get("route", [""])[int(info.get("route_index", 0))]) == edge_id
                or normalizar_id(self.traci.vehicle.getRoadID(veh_id)) == edge_id
            ]
            if occupying:
                diagnostics.append(
                    f"{edge_id}:EDGE_OCUPADO_POR={','.join(sorted(occupying))}"
                )
                continue

            detours: Dict[str, Dict[str, Any]] = {}
            valid = True
            for veh_id, info in sorted(affected_infos.items()):
                new_route, rebuilt_segments, status = self._build_detour(
                    info, edge_id
                )
                if not new_route:
                    valid = False
                    diagnostics.append(f"{edge_id}:{veh_id}:{status}")
                    break

                detours[veh_id] = {
                    **info,
                    "alternative_route": new_route,
                    "rebuilt_segments": rebuilt_segments,
                    "route_status": status,
                }

            if valid and len(detours) >= self.min_affected_vehicles:
                return edge_id, detours, "OK"

        return None, {}, ";".join(diagnostics[-20:]) or "SIN_CANDIDATO_VALIDO"

    def _activate_collapse(self, event: Dict[str, Any], sim_time: float) -> None:
        event_id = str(event.get("id", "DERRUMBE")).strip() or "DERRUMBE"
        selected_edge, detours, reason = self._find_valid_candidate()

        if not selected_edge:
            # No finalizar E1 por un instante desfavorable. Mientras la ventana siga
            # abierta, update() volverá a intentar en el siguiente step.
            if not hasattr(self, "_collapse_no_candidate"):
                self._collapse_no_candidate = set()
            if event_id not in self._collapse_no_candidate:
                self._collapse_no_candidate.add(event_id)
                print(f"[derrumbe] {event_id} esperando candidato valido: {reason}")
                self._write_report(
                    sim_time, event_id, "ESPERANDO_CANDIDATO", "", "", "", "", "",
                    "PENDIENTE", reason,
                )
            return

        applied: Dict[str, Dict[str, Any]] = {}
        failed_detail = ""

        # Guardar también el mapa previo para poder restaurarlo si falla un camión.
        for veh_id, info in detours.items():
            try:
                old_route = list(info["route"])
                new_route = list(info["alternative_route"])

                current_detour = list(info.get("rebuilt_segments", [{}])[0].get("route", []))
                ok, detail = self.net.validar_ruta_excluida(
                    current_detour, {selected_edge}
                )
                if not ok or selected_edge in current_detour:
                    raise ValueError(detail if not ok else "EDGE_BLOQUEADO_EN_DESVIO_ACTUAL")

                self.traci.vehicle.setRoute(veh_id, new_route)
                applied[veh_id] = {
                    "old_route": old_route,
                    "new_route": new_route,
                    "destination_edge": info["destination_edge"],
                    "info": info,
                }

                self._rebuild_plan(
                    veh_id=veh_id,
                    info=info,
                    new_route=new_route,
                    rebuilt_segments=info["rebuilt_segments"],
                )

                self._write_report(
                    sim_time, event_id, "REROUTE", selected_edge, veh_id,
                    info["destination_edge"], " ".join(old_route), " ".join(new_route),
                    "OK", "SEGMENTO_ACTUAL_DESVIADO;FUTURO_CONSERVADO",
                )
            except Exception as exc:
                failed_detail = f"{veh_id}:{exc}"
                self._write_report(
                    sim_time, event_id, "REROUTE", selected_edge, veh_id,
                    info.get("destination_edge", ""), " ".join(info.get("route", [])), "",
                    "ERROR", str(exc),
                )
                print(f"[derrumbe] ERROR redirigiendo {veh_id}: {exc}")
                break

        if failed_detail or len(applied) < self.min_affected_vehicles:
            # Restaurar físicamente y también limpiar el mapa de ruta aplicada de E1.
            for veh_id, state in applied.items():
                try:
                    self.traci.vehicle.setRoute(veh_id, state["old_route"])
                    plan = self.planes.get(veh_id, {})
                    for key in (
                        "_route_index_base_sumo",
                        "_ruta_aplicada_edges",
                        "_ruta_aplicada_plan_indices",
                        "_ruta_pos_ultima",
                        "_route_index_resync_reportado",
                    ):
                        # Si había un mapa de rescheduling anterior, _rebuild_plan
                        # guarda una copia temporal y se restaura aquí.
                        backup_key = f"_e1_backup{key}"
                        if backup_key in plan:
                            plan[key] = plan.pop(backup_key)
                        elif key in plan and not str(plan.get("tipo_plan", "")).upper() == "RESCHEDULING":
                            plan.pop(key, None)
                except Exception as exc:
                    print(f"[derrumbe] ADVERTENCIA restaurando {veh_id}: {exc}")

            detail = failed_detail or (
                f"SOLO_{len(applied)}_DESVIOS_APLICADOS;MINIMO={self.min_affected_vehicles}"
            )
            self._write_report(
                sim_time, event_id, "NO_ACTIVADO", selected_edge, "", "", "", "",
                "DESVIOS_INSUFICIENTES", detail,
            )
            print(f"[derrumbe] {event_id} no activado en este intento: {detail}")
            return

        lanes = self._close_edge(selected_edge)

        for veh_id, state in applied.items():
            print(
                f"[derrumbe] {event_id} camion={veh_id} edge={selected_edge} "
                "segmento_actual_desviado=SI;segmentos_futuros=CONSERVADOS"
            )
            state.pop("info", None)

        self.active_events[event_id] = {
            "event": deepcopy(event),
            "edge": selected_edge,
            "lanes": lanes,
            "vehicles": applied,
        }
        if hasattr(self, "_collapse_no_candidate"):
            self._collapse_no_candidate.discard(event_id)

        if self.event_callback:
            try:
                self.event_callback({
                    "type": "REGISTER_BLOCKED_EDGE",
                    "event_id": event_id,
                    "edge": selected_edge,
                    "time": float(sim_time),
                })
            except Exception as exc:
                print(f"[derrumbe] no se pudo registrar edge bloqueado en server.py: {exc}")

        self._write_report(
            sim_time, event_id, "ACTIVAR", selected_edge, "", "", "", "",
            "OK", f"camiones_afectados={len(applied)};modo=SOLO_SEGMENTO_ACTUAL",
        )
        print(
            f"[derrumbe] ACTIVADO {event_id} t={sim_time:.1f}s edge={selected_edge} "
            f"camiones={len(applied)}"
        )

    def _deactivate_collapse(self, event_id: str, sim_time: float) -> None:
        state = self.active_events.pop(event_id, None)
        if not state:
            return
        edge_id = state["edge"]
        self._open_edge(edge_id, state.get("lanes", []))
        if self.event_callback:
            try:
                self.event_callback({
                    "type": "UNREGISTER_BLOCKED_EDGE",
                    "event_id": event_id,
                    "edge": edge_id,
                    "time": float(sim_time),
                })
            except Exception as exc:
                print(f"[derrumbe] no se pudo liberar edge bloqueado en server.py: {exc}")
        self.finished_events.add(event_id)
        self._write_report(
            sim_time, event_id, "REABRIR", edge_id, "", "", "", "", "OK",
            "Los camiones mantienen el desvio ya asignado",
        )
        print(f"[derrumbe] FINALIZADO {event_id} t={sim_time:.1f}s edge_reabierto={edge_id}")

    def _close_edge(self, edge_id: str) -> List[Tuple[str, List[str]]]:
        """
        Cierre logico seguro.

        No cambia allowed/disallowed de las lanes porque SUMO vuelve a validar
        todas las rutas cargadas al modificar permisos. Con rutas largas,
        paradas futuras y rutas TraCI, esa validacion puede cerrar la simulacion
        aunque los camiones hayan recibido un desvio.

        El cierre se aplica mediante:
        1) rutas alternativas que excluyen edge_id;
        2) registro del edge bloqueado para las consultas futuras de routing;
        3) vigilancia de rutas mientras el evento permanece activo.

        Se conserva la firma y se retorna una lista vacia por compatibilidad.
        """
        print(
            f"[derrumbe] cierre logico aplicado a {edge_id}; "
            "no se modifican permisos de lane para evitar FatalTraCIError"
        )
        return []

    def _open_edge(self, edge_id: str, lane_states: List[Tuple[str, List[str]]]) -> None:
        """
        Finaliza el cierre logico. No hay permisos de lane que restaurar.
        """
        print(f"[derrumbe] cierre logico liberado para {edge_id}")

    def _enforce_closures(self, sim_time: float) -> None:
        """
        Mientras E1 siga activo, solo se interviene a un camión cuando el edge
        bloqueado aparece dentro de SU SEGMENTO ACTUAL.

        Un edge bloqueado puede existir en un ciclo futuro sin que E1 reescriba
        ese ciclo. Cuando dicho ciclo pase a ser actual, se calcula entonces el
        desvío correspondiente.
        """
        broken_trucks = {
            str(state.get("truck", ""))
            for state in self.active_truck_breakdowns.values()
        }

        for event_id, state in list(self.active_events.items()):
            blocked_edge = normalizar_id(state.get("edge", ""))
            if not blocked_edge:
                continue

            try:
                vehicles = list(self.traci.vehicle.getIDList())
            except Exception:
                vehicles = []

            for veh_id in sorted(vehicles):
                if veh_id in broken_trucks:
                    continue

                info = self._active_segment_info(veh_id)
                if not info:
                    continue
                if blocked_edge not in info.get("pending_segment", []):
                    continue

                # No modificar al camión si ya está sobre el edge bloqueado.
                try:
                    if normalizar_id(self.traci.vehicle.getRoadID(veh_id)) == blocked_edge:
                        continue
                except Exception:
                    pass

                new_route, rebuilt_segments, status = self._build_detour(
                    info, blocked_edge
                )
                if not new_route:
                    self._write_report(
                        sim_time, event_id, "VIGILAR", blocked_edge, veh_id,
                        info.get("destination_edge", ""),
                        " ".join(info.get("route", [])), "",
                        "SIN_DESVIO", status,
                    )
                    continue

                try:
                    old_route = list(info.get("route", []))
                    self.traci.vehicle.setRoute(veh_id, new_route)
                    self._rebuild_plan(
                        veh_id=veh_id,
                        info=info,
                        new_route=new_route,
                        rebuilt_segments=rebuilt_segments,
                    )
                    self._write_report(
                        sim_time, event_id, "VIGILAR", blocked_edge, veh_id,
                        info.get("destination_edge", ""),
                        " ".join(old_route), " ".join(new_route),
                        "REROUTE_OK", "SEGMENTO_ACTUAL_DESVIADO;FUTURO_CONSERVADO",
                    )
                    print(
                        f"[derrumbe] vigilancia: {veh_id} desviado en su segmento actual "
                        f"para evitar {blocked_edge}"
                    )
                except Exception as exc:
                    self._write_report(
                        sim_time, event_id, "VIGILAR", blocked_edge, veh_id,
                        info.get("destination_edge", ""),
                        " ".join(info.get("route", [])), "",
                        "ERROR", str(exc),
                    )

    def _rebuild_plan(
        self,
        veh_id: str,
        info: Dict[str, Any],
        new_route: List[str],
        rebuilt_segments: List[Dict[str, Any]],
    ) -> None:
        """
        Actualiza únicamente el mapa físico de la ruta aplicada por E1.

        NO modifica:
        - plan["edges"]
        - index_to_speed / index_to_operacion / index_to_segment
        - segmentos
        - seg["ruta"], edge_fin, cycle_id o secuencia

        Por tanto el scheduling/rescheduling lógico permanece intacto.
        """
        plan = self.planes.get(veh_id)
        if not plan:
            return
        if not rebuilt_segments:
            return

        rebuilt = rebuilt_segments[0]
        new_plan_map = list(rebuilt.get("plan_map", []) or [])
        if len(new_plan_map) != len(new_route):
            raise ValueError(
                f"MAPA_E1_INCONSISTENTE:{len(new_plan_map)}!={len(new_route)}"
            )

        # Respaldar una sola vez el mapa previo (especialmente importante cuando
        # el camión ya está ejecutando un ciclo de rescheduling).
        for key in (
            "_route_index_base_sumo",
            "_ruta_aplicada_edges",
            "_ruta_aplicada_plan_indices",
            "_ruta_pos_ultima",
            "_route_index_resync_reportado",
        ):
            backup_key = f"_e1_backup{key}"
            if backup_key not in plan and key in plan:
                value = plan[key]
                plan[backup_key] = list(value) if isinstance(value, list) else value

        try:
            base_sumo = int(self.traci.vehicle.getRouteIndex(veh_id))
        except Exception:
            base_sumo = 0

        plan["_route_index_base_sumo"] = int(base_sumo)
        plan["_ruta_aplicada_edges"] = list(new_route)
        plan["_ruta_aplicada_plan_indices"] = [int(x) for x in new_plan_map]
        plan["_ruta_pos_ultima"] = 0
        plan["_route_index_resync_reportado"] = False

    def _write_report(
        self,
        sim_time: float,
        event_id: str,
        action: str,
        edge_id: str,
        vehicle_id: str,
        destination: str,
        old_route: str,
        new_route: str,
        result: str,
        detail: str,
        event_type: str = "derrumbe",
    ) -> None:
        if not self._report_writer:
            return
        self._report_writer.writerow({
            "tiempo_s": f"{float(sim_time):.3f}",
            "evento_id": event_id,
            "tipo": str(event_type or "evento"),
            "accion": action,
            "edge": edge_id,
            "camion": vehicle_id,
            "destino_segmento": destination,
            "ruta_anterior": old_route,
            "ruta_nueva": new_route,
            "resultado": result,
            "detalle": detail,
        })
        self._report_file.flush()
