# -*- coding: utf-8 -*-
"""
metricas.py

Centraliza las métricas de simulación y rendimiento del prototipo JADE + Python + SUMO.

Métricas implementadas:
- Error relativo por segmento, promedio y global.
- Tiempo perdido (timeLoss) promedio, total y máximo desde tripinfo.xml.
- Duración real de la ejecución.
- Utilización promedio/máxima de CPU del prototipo.
- Consumo promedio/máximo de memoria RAM del prototipo.
- Throughput de mensajes ACL informado por JADE.

Estructura esperada:
    proyecto/
        metricas.py
        config/
            small/
            mediano/
            grande/

JADE debe enviar un mensaje JSON de tipo JADE_METRICS al servidor Python. Este módulo
calcula el throughput y conserva los conteos por performative. El runner de SUMO puede
usar GestorMetricas para medir recursos y generar el resumen al finalizar.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import statistics
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, MutableMapping, Optional, Sequence, Set, Tuple

try:
    import psutil  # type: ignore
except Exception:  # pragma: no cover - dependencia opcional en importación
    psutil = None


PROJECT_ROOT = Path(__file__).resolve().parent
CONFIG_ROOT = PROJECT_ROOT / "config"
ESCENARIOS_ALIAS = {
    "small": "small",
    "pequeno": "small",
    "pequeño": "small",
    "mediano": "mediano",
    "medium": "mediano",
    "grande": "grande",
    "large": "grande",
}
PERFORMATIVES = (
    "CFP",
    "PROPOSE",
    "REFUSE",
    "ACCEPT_PROPOSAL",
    "REJECT_PROPOSAL",
    "INFORM",
    "OTROS",
)


def normalizar_escenario(escenario: Any) -> str:
    valor = str(escenario or "small").strip().lower()
    canonico = ESCENARIOS_ALIAS.get(valor, valor)
    if canonico not in ("small", "mediano", "grande"):
        raise ValueError(
            f"Escenario no válido: {escenario!r}. Usa small, mediano o grande."
        )
    return canonico


def directorio_metricas(escenario: Any, output_dir: Any = None) -> Path:
    if output_dir not in (None, ""):
        return Path(output_dir).resolve()
    return (CONFIG_ROOT / normalizar_escenario(escenario)).resolve()


def _float(valor: Any, default: float = 0.0) -> float:
    try:
        numero = float(str(valor).strip().replace(",", "."))
        if not math.isfinite(numero):
            return default
        return numero
    except Exception:
        return default


def _int(valor: Any, default: int = 0) -> int:
    try:
        return int(float(str(valor).strip().replace(",", ".")))
    except Exception:
        return default


def _primer_valor(dic: Mapping[str, Any], claves: Sequence[str], default: Any = None) -> Any:
    for clave in claves:
        if clave in dic and dic.get(clave) not in (None, ""):
            return dic.get(clave)
    inferiores = {str(k).lower(): k for k in dic.keys()}
    for clave in claves:
        real = inferiores.get(str(clave).lower())
        if real is not None and dic.get(real) not in (None, ""):
            return dic.get(real)
    return default


def _escribir_json(path: Path, data: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporal = path.with_suffix(path.suffix + ".tmp")
    with temporal.open("w", encoding="utf-8") as archivo:
        json.dump(data, archivo, ensure_ascii=False, indent=2)
    os.replace(temporal, path)


def _leer_json(path: Path, default: Any = None) -> Any:
    if not path.exists():
        return default
    try:
        with path.open("r", encoding="utf-8") as archivo:
            return json.load(archivo)
    except Exception:
        return default


def calcular_error_relativo(tiempo_simulado_s: Any, tiempo_referencia_s: Any) -> Optional[float]:
    """Calcula |T_sim - T_ref| / T_ref * 100. Devuelve None si T_ref <= 0."""
    simulado = _float(tiempo_simulado_s, 0.0)
    referencia = _float(tiempo_referencia_s, 0.0)
    if referencia <= 0:
        return None
    return abs(simulado - referencia) / referencia * 100.0


def _normalizar_performative(nombre: Any) -> str:
    texto = str(nombre or "").strip().upper().replace("-", "_").replace(" ", "_")
    alias = {
        "ACCEPTPROPOSAL": "ACCEPT_PROPOSAL",
        "REJECTPROPOSAL": "REJECT_PROPOSAL",
        "ACCEPT_PROPOSAL": "ACCEPT_PROPOSAL",
        "REJECT_PROPOSAL": "REJECT_PROPOSAL",
    }
    texto = alias.get(texto, texto)
    return texto if texto in PERFORMATIVES else "OTROS"


def _extraer_conteos_jade(payload: Mapping[str, Any]) -> Dict[str, int]:
    conteos: Dict[str, int] = {nombre: 0 for nombre in PERFORMATIVES}
    origen = payload.get("counts") or payload.get("conteos") or payload.get("mensajesPorTipo") or {}

    if isinstance(origen, Mapping):
        for clave, valor in origen.items():
            conteos[_normalizar_performative(clave)] += max(0, _int(valor, 0))

    claves_directas = {
        "CFP": ["cfp", "mensajesCfp", "mensajes_cfp"],
        "PROPOSE": ["propose", "mensajesPropose", "mensajes_propose"],
        "REFUSE": ["refuse", "mensajesRefuse", "mensajes_refuse"],
        "ACCEPT_PROPOSAL": ["acceptProposal", "accept_proposal", "mensajesAcceptProposal"],
        "REJECT_PROPOSAL": ["rejectProposal", "reject_proposal", "mensajesRejectProposal"],
        "INFORM": ["inform", "mensajesInform", "mensajes_inform"],
        "OTROS": ["otros", "other", "mensajesOtros"],
    }
    for performative, claves in claves_directas.items():
        valor = _primer_valor(payload, claves, None)
        if valor not in (None, ""):
            conteos[performative] = max(conteos[performative], max(0, _int(valor, 0)))

    return conteos


def procesar_metricas_jade(payload: Mapping[str, Any], escenario: Any) -> Dict[str, Any]:
    """Normaliza el mensaje JADE_METRICS y calcula el throughput en mensajes/s."""
    conteos = _extraer_conteos_jade(payload)
    total_calculado = sum(conteos.values())
    total_informado = _int(
        _primer_valor(
            payload,
            ["totalMessages", "mensajesTotal", "mensajes_total", "nMsg", "Nmsg"],
            total_calculado,
        ),
        total_calculado,
    )
    total = max(total_calculado, total_informado)

    duracion_s = _float(
        _primer_valor(
            payload,
            [
                "processingTimeSeconds",
                "duracionProcesamientoS",
                "tiempoProcesamientoS",
                "durationSeconds",
                "tProcS",
            ],
            0.0,
        ),
        0.0,
    )

    if duracion_s <= 0:
        inicio_ns = _int(_primer_valor(payload, ["inicioNs", "startNs"], 0), 0)
        fin_ns = _int(_primer_valor(payload, ["finNs", "endNs"], 0), 0)
        if fin_ns > inicio_ns > 0:
            duracion_s = (fin_ns - inicio_ns) / 1_000_000_000.0

    throughput = total / duracion_s if duracion_s > 0 else 0.0

    resultado: Dict[str, Any] = {
        "tipo": "JADE_METRICS",
        "escenario": normalizar_escenario(escenario),
        "pid_jade": _int(_primer_valor(payload, ["pidJade", "pid_jade", "pid"], 0), 0),
        "inicio_ns": _int(_primer_valor(payload, ["inicioNs", "startNs"], 0), 0),
        "fin_ns": _int(_primer_valor(payload, ["finNs", "endNs"], 0), 0),
        "duracion_procesamiento_s": round(duracion_s, 6),
        "mensajes_total": total,
        "throughput_mensajes_s": round(throughput, 6),
        "conteos": conteos,
        "recibido_epoch_s": time.time(),
    }
    return resultado


def registrar_componente(
    escenario: Any,
    componente: str,
    pid: Any,
    output_dir: Any = None,
) -> Dict[str, Any]:
    """Registra el PID de JADE, server, runner u otro componente del prototipo."""
    carpeta = directorio_metricas(escenario, output_dir)
    carpeta.mkdir(parents=True, exist_ok=True)
    path = carpeta / "pids_componentes.json"
    data = _leer_json(path, {})
    if not isinstance(data, MutableMapping):
        data = {}
    componentes = data.setdefault("componentes", {})
    if not isinstance(componentes, MutableMapping):
        componentes = {}
        data["componentes"] = componentes

    pid_entero = _int(pid, 0)
    if pid_entero <= 0:
        raise ValueError(f"PID inválido para {componente}: {pid!r}")

    componentes[str(componente)] = {
        "pid": pid_entero,
        "actualizado_epoch_s": time.time(),
    }
    data["escenario"] = normalizar_escenario(escenario)
    data["actualizado_epoch_s"] = time.time()
    _escribir_json(path, data)
    return {"ok": True, "componente": str(componente), "pid": pid_entero, "path": str(path)}


def guardar_metricas_jade(
    payload: Mapping[str, Any],
    escenario: Any,
    output_dir: Any = None,
) -> Dict[str, Any]:
    """Guarda el mensaje final de JADE y registra su PID para el monitor de recursos."""
    carpeta = directorio_metricas(escenario, output_dir)
    carpeta.mkdir(parents=True, exist_ok=True)
    resultado = procesar_metricas_jade(payload, escenario)

    _escribir_json(carpeta / "metricas_jade.json", resultado)
    csv_path = carpeta / "metricas_jade.csv"
    campos = [
        "escenario",
        "pid_jade",
        "duracion_procesamiento_s",
        "CFP",
        "PROPOSE",
        "REFUSE",
        "ACCEPT_PROPOSAL",
        "REJECT_PROPOSAL",
        "INFORM",
        "OTROS",
        "mensajes_total",
        "throughput_mensajes_s",
    ]
    with csv_path.open("w", newline="", encoding="utf-8-sig") as archivo:
        writer = csv.DictWriter(archivo, fieldnames=campos)
        writer.writeheader()
        fila = {
            "escenario": resultado["escenario"],
            "pid_jade": resultado["pid_jade"],
            "duracion_procesamiento_s": resultado["duracion_procesamiento_s"],
            "mensajes_total": resultado["mensajes_total"],
            "throughput_mensajes_s": resultado["throughput_mensajes_s"],
        }
        fila.update(resultado["conteos"])
        writer.writerow(fila)

    if resultado["pid_jade"] > 0:
        registrar_componente(resultado["escenario"], "jade", resultado["pid_jade"], carpeta)

    return {
        "ok": True,
        "msg": "Métricas de JADE registradas",
        "metricas_jade": resultado,
        "json": str(carpeta / "metricas_jade.json"),
        "csv": str(csv_path),
    }


def analizar_reporte_segmentos(path: Path, output_csv: Optional[Path] = None) -> Dict[str, Any]:
    """Calcula error relativo por segmento y sus agregados desde reporte_segmentos_sumo.csv."""
    if not path.exists():
        return {
            "disponible": False,
            "archivo": str(path),
            "cantidad_segmentos": 0,
            "motivo": "REPORTE_SEGMENTOS_NO_EXISTE",
        }

    filas_salida: List[Dict[str, Any]] = []
    errores: List[float] = []
    tiempos_planificados: List[float] = []
    tiempos_simulados: List[float] = []
    time_loss_segmentos: List[float] = []

    with path.open("r", newline="", encoding="utf-8-sig") as archivo:
        reader = csv.DictReader(archivo)
        for fila in reader:
            plan = _float(
                _primer_valor(fila, ["jade_duracion_s", "tiempo_planificado_s", "duracion_planificada_s"], 0.0),
                0.0,
            )
            sim = _float(
                _primer_valor(fila, ["sumo_duracion_s", "tiempo_simulado_s", "duracion_simulada_s"], 0.0),
                0.0,
            )
            error = calcular_error_relativo(sim, plan)
            time_loss = _float(
                _primer_valor(fila, ["time_loss_segmento_s", "timeLoss_segmento_s", "time_loss_s"], 0.0),
                0.0,
            )

            if plan > 0:
                tiempos_planificados.append(plan)
                tiempos_simulados.append(sim)
                if error is not None:
                    errores.append(error)
            if time_loss >= 0 and any(
                clave in fila for clave in ("time_loss_segmento_s", "timeLoss_segmento_s", "time_loss_s")
            ):
                time_loss_segmentos.append(time_loss)

            nueva = dict(fila)
            nueva["error_relativo_pct"] = "" if error is None else f"{error:.6f}"
            filas_salida.append(nueva)

    if output_csv is not None:
        output_csv.parent.mkdir(parents=True, exist_ok=True)
        fieldnames: List[str] = []
        for fila in filas_salida:
            for clave in fila.keys():
                if clave not in fieldnames:
                    fieldnames.append(clave)
        if "error_relativo_pct" not in fieldnames:
            fieldnames.append("error_relativo_pct")
        with output_csv.open("w", newline="", encoding="utf-8-sig") as archivo:
            writer = csv.DictWriter(archivo, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(filas_salida)

    total_plan = sum(tiempos_planificados)
    total_sim = sum(tiempos_simulados)
    error_global = calcular_error_relativo(total_sim, total_plan)

    return {
        "disponible": True,
        "archivo": str(path),
        "cantidad_segmentos": len(tiempos_planificados),
        "tiempo_planificado_total_s": round(total_plan, 6),
        "tiempo_simulado_total_s": round(total_sim, 6),
        "diferencia_total_s": round(total_sim - total_plan, 6),
        "error_relativo_promedio_pct": round(statistics.fmean(errores), 6) if errores else 0.0,
        "error_relativo_global_pct": round(error_global or 0.0, 6),
        "error_relativo_maximo_pct": round(max(errores), 6) if errores else 0.0,
        "time_loss_segmentos_disponible": bool(time_loss_segmentos),
        "tiempo_perdido_segmento_promedio_s": (
            round(statistics.fmean(time_loss_segmentos), 6) if time_loss_segmentos else 0.0
        ),
        "tiempo_perdido_segmento_total_s": round(sum(time_loss_segmentos), 6),
    }


def analizar_tripinfo(path: Path) -> Dict[str, Any]:
    """Lee timeLoss de cada <tripinfo> generado por SUMO."""
    if not path.exists():
        return {
            "disponible": False,
            "archivo": str(path),
            "cantidad_viajes": 0,
            "motivo": "TRIPINFO_NO_EXISTE",
        }

    valores: List[float] = []
    try:
        root = ET.parse(path).getroot()
        for elemento in root.findall(".//tripinfo"):
            valores.append(max(0.0, _float(elemento.get("timeLoss", "0"), 0.0)))
    except Exception as exc:
        return {
            "disponible": False,
            "archivo": str(path),
            "cantidad_viajes": 0,
            "motivo": f"TRIPINFO_INVALIDO: {exc}",
        }

    return {
        "disponible": True,
        "archivo": str(path),
        "cantidad_viajes": len(valores),
        "tiempo_perdido_promedio_s": round(statistics.fmean(valores), 6) if valores else 0.0,
        "tiempo_perdido_total_s": round(sum(valores), 6),
        "tiempo_perdido_maximo_s": round(max(valores), 6) if valores else 0.0,
    }


def resumir_recursos(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {
            "disponible": False,
            "archivo": str(path),
            "cantidad_muestras": 0,
            "motivo": "METRICAS_RECURSOS_NO_EXISTE",
        }

    cpu: List[float] = []
    ram: List[float] = []
    with path.open("r", newline="", encoding="utf-8-sig") as archivo:
        reader = csv.DictReader(archivo)
        for fila in reader:
            cpu.append(_float(fila.get("cpu_prototipo_pct", 0.0), 0.0))
            ram.append(_float(fila.get("ram_prototipo_mb", 0.0), 0.0))

    return {
        "disponible": True,
        "archivo": str(path),
        "cantidad_muestras": len(cpu),
        "cpu_promedio_pct": round(statistics.fmean(cpu), 6) if cpu else 0.0,
        "cpu_maximo_pct": round(max(cpu), 6) if cpu else 0.0,
        "ram_promedio_mb": round(statistics.fmean(ram), 6) if ram else 0.0,
        "ram_maximo_mb": round(max(ram), 6) if ram else 0.0,
    }


@dataclass
class GestorMetricas:
    escenario: str
    output_dir: Any = None
    intervalo_recursos_s: float = 1.0
    incluir_hijos: bool = True
    _inicio_perf: Optional[float] = field(default=None, init=False, repr=False)
    _fin_perf: Optional[float] = field(default=None, init=False, repr=False)
    _stop_event: threading.Event = field(default_factory=threading.Event, init=False, repr=False)
    _thread: Optional[threading.Thread] = field(default=None, init=False, repr=False)
    _procesos_inicializados: Set[int] = field(default_factory=set, init=False, repr=False)
    _procesos_cache: Dict[int, Any] = field(default_factory=dict, init=False, repr=False)

    def __post_init__(self) -> None:
        self.escenario = normalizar_escenario(self.escenario)
        self.output_dir = directorio_metricas(self.escenario, self.output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.intervalo_recursos_s = max(0.2, float(self.intervalo_recursos_s))
        self.recursos_path = self.output_dir / "metricas_recursos.csv"
        self.resumen_path = self.output_dir / "metricas_resumen.json"
        self.resumen_csv_path = self.output_dir / "metricas_resumen.csv"
        self.segmentos_metricas_path = self.output_dir / "metricas_simulacion.csv"
        self._preparar_csv_recursos()

    def _preparar_csv_recursos(self) -> None:
        campos = [
            "tiempo_real_s",
            "epoch_s",
            "cantidad_procesos",
            "pids",
            "cpu_prototipo_pct",
            "cpu_prototipo_raw_pct",
            "ram_prototipo_mb",
        ]
        with self.recursos_path.open("w", newline="", encoding="utf-8-sig") as archivo:
            writer = csv.DictWriter(archivo, fieldnames=campos)
            writer.writeheader()

    def registrar_componente(self, componente: str, pid: Any) -> Dict[str, Any]:
        return registrar_componente(self.escenario, componente, pid, self.output_dir)

    def iniciar(self) -> None:
        if self._inicio_perf is None:
            self._inicio_perf = time.perf_counter()
        self.registrar_componente("runner", os.getpid())
        self.iniciar_monitoreo_recursos()

    def _leer_pids_raiz(self) -> Set[int]:
        datos = _leer_json(self.output_dir / "pids_componentes.json", {})
        pids: Set[int] = {os.getpid()}
        if isinstance(datos, Mapping):
            componentes = datos.get("componentes", {})
            if isinstance(componentes, Mapping):
                for valor in componentes.values():
                    if isinstance(valor, Mapping):
                        pid = _int(valor.get("pid", 0), 0)
                    else:
                        pid = _int(valor, 0)
                    if pid > 0:
                        pids.add(pid)
        return pids

    def _resolver_procesos(self) -> Dict[int, Any]:
        if psutil is None:
            return {}
        encontrados: Dict[int, Any] = {}
        for pid in self._leer_pids_raiz():
            try:
                proceso = self._procesos_cache.get(pid)
                if proceso is None:
                    proceso = psutil.Process(pid)
                    self._procesos_cache[pid] = proceso
                if not proceso.is_running():
                    continue
                encontrados[pid] = proceso
                if self.incluir_hijos:
                    for hijo in proceso.children(recursive=True):
                        try:
                            proceso_hijo = self._procesos_cache.get(hijo.pid)
                            if proceso_hijo is None:
                                proceso_hijo = hijo
                                self._procesos_cache[hijo.pid] = proceso_hijo
                            if proceso_hijo.is_running():
                                encontrados[proceso_hijo.pid] = proceso_hijo
                        except Exception:
                            continue
            except Exception:
                continue
        return encontrados

    def _muestrear_recursos(self) -> None:
        cpu_count = max(1, int(psutil.cpu_count(logical=True) or 1)) if psutil is not None else 1
        while not self._stop_event.wait(self.intervalo_recursos_s):
            procesos = self._resolver_procesos()
            cpu_raw = 0.0
            ram_mb = 0.0
            pids_validos: List[int] = []

            for pid, proceso in procesos.items():
                try:
                    if pid not in self._procesos_inicializados:
                        proceso.cpu_percent(interval=None)
                        self._procesos_inicializados.add(pid)
                        cpu_actual = 0.0
                    else:
                        cpu_actual = max(0.0, float(proceso.cpu_percent(interval=None)))
                    memoria = max(0, int(proceso.memory_info().rss)) / (1024.0 * 1024.0)
                    cpu_raw += cpu_actual
                    ram_mb += memoria
                    pids_validos.append(pid)
                except Exception:
                    continue

            cpu_normalizada = cpu_raw / cpu_count
            tiempo_real = (
                time.perf_counter() - self._inicio_perf if self._inicio_perf is not None else 0.0
            )
            fila = {
                "tiempo_real_s": f"{tiempo_real:.6f}",
                "epoch_s": f"{time.time():.6f}",
                "cantidad_procesos": len(pids_validos),
                "pids": ";".join(str(pid) for pid in sorted(pids_validos)),
                "cpu_prototipo_pct": f"{cpu_normalizada:.6f}",
                "cpu_prototipo_raw_pct": f"{cpu_raw:.6f}",
                "ram_prototipo_mb": f"{ram_mb:.6f}",
            }
            try:
                with self.recursos_path.open("a", newline="", encoding="utf-8-sig") as archivo:
                    writer = csv.DictWriter(archivo, fieldnames=list(fila.keys()))
                    writer.writerow(fila)
            except Exception:
                continue

    def iniciar_monitoreo_recursos(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        if psutil is None:
            _escribir_json(
                self.output_dir / "metricas_recursos_error.json",
                {
                    "ok": False,
                    "error": "PSUTIL_NO_INSTALADO",
                    "instruccion": "Instala psutil con: pip install psutil",
                },
            )
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._muestrear_recursos,
            name="monitor-metricas-recursos",
            daemon=True,
        )
        self._thread.start()

    def detener_monitoreo_recursos(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=max(2.0, self.intervalo_recursos_s * 3.0))
        self._thread = None

    def finalizar(
        self,
        tiempo_sumo_final_s: Any = None,
        reporte_segmentos_path: Any = None,
        tripinfo_path: Any = None,
    ) -> Dict[str, Any]:
        self._fin_perf = time.perf_counter()
        self.detener_monitoreo_recursos()

        reporte_segmentos = Path(reporte_segmentos_path) if reporte_segmentos_path else self.output_dir / "reporte_segmentos_sumo.csv"
        tripinfo = Path(tripinfo_path) if tripinfo_path else self.output_dir / "tripinfo.xml"

        simulacion = analizar_reporte_segmentos(
            reporte_segmentos,
            output_csv=self.segmentos_metricas_path,
        )
        time_loss = analizar_tripinfo(tripinfo)
        recursos = resumir_recursos(self.recursos_path)
        jade = _leer_json(self.output_dir / "metricas_jade.json", {})
        if not isinstance(jade, Mapping):
            jade = {}

        duracion_real = 0.0
        if self._inicio_perf is not None and self._fin_perf is not None:
            duracion_real = max(0.0, self._fin_perf - self._inicio_perf)

        resumen: Dict[str, Any] = {
            "escenario": self.escenario,
            "generado_epoch_s": time.time(),
            "duracion_real_ejecucion_s": round(duracion_real, 6),
            "tiempo_sumo_final_s": round(_float(tiempo_sumo_final_s, 0.0), 6),
            "simulacion": simulacion,
            "tiempo_perdido": time_loss,
            "recursos": recursos,
            "jade": dict(jade),
        }

        # Campos planos para facilitar tablas comparativas entre escenarios.
        resumen["indicadores"] = {
            "cantidad_segmentos": simulacion.get("cantidad_segmentos", 0),
            "error_relativo_promedio_pct": simulacion.get("error_relativo_promedio_pct", 0.0),
            "error_relativo_global_pct": simulacion.get("error_relativo_global_pct", 0.0),
            "tiempo_perdido_promedio_s": time_loss.get("tiempo_perdido_promedio_s", 0.0),
            "cpu_promedio_pct": recursos.get("cpu_promedio_pct", 0.0),
            "cpu_maximo_pct": recursos.get("cpu_maximo_pct", 0.0),
            "ram_promedio_mb": recursos.get("ram_promedio_mb", 0.0),
            "ram_maximo_mb": recursos.get("ram_maximo_mb", 0.0),
            "mensajes_acl_total": jade.get("mensajes_total", 0),
            "throughput_jade_mensajes_s": jade.get("throughput_mensajes_s", 0.0),
            "duracion_real_ejecucion_s": round(duracion_real, 6),
            "tiempo_sumo_final_s": round(_float(tiempo_sumo_final_s, 0.0), 6),
        }

        _escribir_json(self.resumen_path, resumen)
        self._escribir_resumen_csv(resumen)
        return resumen

    def _escribir_resumen_csv(self, resumen: Mapping[str, Any]) -> None:
        indicadores = resumen.get("indicadores", {})
        if not isinstance(indicadores, Mapping):
            indicadores = {}
        fila = {"escenario": self.escenario, **dict(indicadores)}
        with self.resumen_csv_path.open("w", newline="", encoding="utf-8-sig") as archivo:
            writer = csv.DictWriter(archivo, fieldnames=list(fila.keys()))
            writer.writeheader()
            writer.writerow(fila)


def generar_resumen_postproceso(escenario: Any, output_dir: Any = None) -> Dict[str, Any]:
    """Genera el resumen sin iniciar un nuevo monitoreo de recursos."""
    carpeta = directorio_metricas(escenario, output_dir)
    simulacion = analizar_reporte_segmentos(
        carpeta / "reporte_segmentos_sumo.csv",
        output_csv=carpeta / "metricas_simulacion.csv",
    )
    time_loss = analizar_tripinfo(carpeta / "tripinfo.xml")
    recursos = resumir_recursos(carpeta / "metricas_recursos.csv")
    jade = _leer_json(carpeta / "metricas_jade.json", {})
    if not isinstance(jade, Mapping):
        jade = {}

    resumen = {
        "escenario": normalizar_escenario(escenario),
        "generado_epoch_s": time.time(),
        "simulacion": simulacion,
        "tiempo_perdido": time_loss,
        "recursos": recursos,
        "jade": dict(jade),
    }
    resumen["indicadores"] = {
        "cantidad_segmentos": simulacion.get("cantidad_segmentos", 0),
        "error_relativo_promedio_pct": simulacion.get("error_relativo_promedio_pct", 0.0),
        "error_relativo_global_pct": simulacion.get("error_relativo_global_pct", 0.0),
        "tiempo_perdido_promedio_s": time_loss.get("tiempo_perdido_promedio_s", 0.0),
        "cpu_promedio_pct": recursos.get("cpu_promedio_pct", 0.0),
        "ram_promedio_mb": recursos.get("ram_promedio_mb", 0.0),
        "mensajes_acl_total": jade.get("mensajes_total", 0),
        "throughput_jade_mensajes_s": jade.get("throughput_mensajes_s", 0.0),
    }
    _escribir_json(carpeta / "metricas_resumen.json", resumen)

    fila = {"escenario": resumen["escenario"], **resumen["indicadores"]}
    with (carpeta / "metricas_resumen.csv").open("w", newline="", encoding="utf-8-sig") as archivo:
        writer = csv.DictWriter(archivo, fieldnames=list(fila.keys()))
        writer.writeheader()
        writer.writerow(fila)
    return resumen


def main() -> None:
    parser = argparse.ArgumentParser(description="Cálculo de métricas JADE + SUMO")
    parser.add_argument("--escenario", "--scenario", default="small")
    parser.add_argument("--output-dir", default="")
    parser.add_argument(
        "--procesar",
        action="store_true",
        help="Procesa reportes existentes y genera metricas_resumen.json/csv",
    )
    parser.add_argument(
        "--jade-json",
        default="",
        help="Registra un payload JADE_METRICS almacenado en un archivo JSON",
    )
    args = parser.parse_args()

    carpeta = directorio_metricas(args.escenario, args.output_dir or None)
    if args.jade_json:
        payload = _leer_json(Path(args.jade_json), None)
        if not isinstance(payload, Mapping):
            raise ValueError(f"El archivo no contiene un objeto JSON válido: {args.jade_json}")
        resultado = guardar_metricas_jade(payload, args.escenario, carpeta)
        print(json.dumps(resultado, ensure_ascii=False, indent=2))

    if args.procesar:
        resultado = generar_resumen_postproceso(args.escenario, carpeta)
        print(json.dumps(resultado, ensure_ascii=False, indent=2))

    if not args.procesar and not args.jade_json:
        parser.print_help()


if __name__ == "__main__":
    main()
