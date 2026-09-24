# -*- coding: utf-8 -*-
"""
limpiar_generados.py

Borra archivos generados por el flujo JADE -> Python -> SUMO.
No borra archivos base como:
- net.net.xml
- objects.xml
- trucks.xml
- server.py
- routing_service.py
- sumo_writer.py
"""

from pathlib import Path

# Carpeta raíz donde estás ejecutando el script
ROOT = Path(__file__).resolve().parent

# Archivos generados conocidos
ARCHIVOS_GENERADOS = [
    
    "cronograma_sumo.csv",
    "cronograma_sumo.json",
    "cronograma_sumo_memoria.json",
    "mntrucks_generado.rou.xml",
    "mntrucks_generado.trips.xml",
    "mnobject_generado.add.xml",
    "small_generado.sumocfg",
    "mediano_generado.sumocfg",
    "grande_generado.sumocfg",
    "tripinfo.xml",
    "summary.xml",
    "vehroute.xml",
    "emissions.xml",
    "fcd.xml",
    "schedules.csv",
    "rescheduling.csv",
    "eventos_dinamicos_sumo.csv",
    "snapshot_flota_monitoreo.csv",
    "snapshot_flota_monitoreo_ultimo.json",
    "snapshot_flota_rescheduling.csv",
    "snapshot_flota_rescheduling.json",
    "reporte_segmentos_sumo.csv",
    "reporte_derrumbe_sumo.csv",
    "trace_truck_free.csv",
    "metricas_simulacion.csv",
    "metricas_recursos.csv",
    "metricas_resumen.json",
    "sumoFleetSinRLreschedules.csv",
    "sumoFleetSinRlreschedules.csv",
]

# Revisar la raíz y las carpetas de salida de todos los escenarios.
# Los XML base de mediano/ y grande/ NO se tocan; únicamente config/<escenario>.
CONFIG_DIR = ROOT / "config"
CARPETAS_A_REVISAR = [
    ROOT,
    CONFIG_DIR / "small",
    CONFIG_DIR / "mediano",
    CONFIG_DIR / "grande",
]

# Patrones generados opcionales
PATRONES_GENERADOS = [
    "*_generado.rou.xml",
    "*_generado.add.xml",
    "*_generado.sumocfg",
    "reporte_segmentos_sumo.csv",
    "run_sumo_velocidad_jade.py",
    "cronograma_sumo*.csv",
    "cronograma_sumo*.json",
    "tripinfo*.xml",
    "summary*.xml",
    "vehroute*.xml",
    "emissions*.xml",
    "fcd*.xml",
    "rescheduling.csv",
    "snapshot_flota_rescheduling.csv",
    "snapshot_flota_rescheduling.json",
    "reporte_derrumbe_sumo*.csv",
    "trace_truck_free*.csv",
    "metricas_simulacion*.csv",
    "metricas_recursos*.csv",
    "metricas_resumen*.json",
]

def borrar_archivo(path: Path):
    if path.exists() and path.is_file():
        try:
            path.unlink()
            print(f"[OK] Borrado: {path}")
        except Exception as e:
            print(f"[ERROR] No se pudo borrar {path}: {e}")

def main():
    print("========================================")
    print(" LIMPIEZA DE ARCHIVOS GENERADOS")
    print("========================================")

    borrados = set()

    for carpeta in CARPETAS_A_REVISAR:
        if not carpeta.exists():
            continue

        print(f"\nRevisando carpeta: {carpeta}")

        for nombre in ARCHIVOS_GENERADOS:
            path = carpeta / nombre
            if path.exists() and path not in borrados:
                borrar_archivo(path)
                borrados.add(path)

        for patron in PATRONES_GENERADOS:
            for path in carpeta.glob(patron):
                if path.exists() and path.is_file() and path not in borrados:
                    borrar_archivo(path)
                    borrados.add(path)

    print("\n========================================")
    print(" Limpieza terminada")
    print("========================================")
    print(f"Total archivos borrados: {len(borrados)}")

if __name__ == "__main__":
    main()