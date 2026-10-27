#!/usr/bin/env python3
"""
CapaSense - Prueba del programa receptor sin el ESP32.

Publica en el broker una secuencia de mensajes iguales a los del nodo,
incluidos tres errores a propósito, para comprobar que el receptor:
  - muestra y guarda los mensajes correctos,
  - detecta un estado que no coincide con la ocupación,
  - detecta mensajes perdidos (salto en seq),
  - marca un mensaje diferido (hora del nodo 95 s atrás),
  - rechaza un mensaje que no es JSON.

Uso (con el broker y el receptor ya corriendo):
  python3 prueba_receptor.py --host localhost --user capasense --password CLAVE
"""

import argparse
import json
import time
from datetime import datetime, timedelta, timezone

import paho.mqtt.client as mqtt

ZONA_GT = timezone(timedelta(hours=-6))
BASE = "capasense/nodo99"          # nodo de prueba, para no mezclarlo con nodo01

p = argparse.ArgumentParser()
p.add_argument("--host", default="localhost")
p.add_argument("--port", type=int, default=1883)
p.add_argument("--user")
p.add_argument("--password")
args = p.parse_args()

cliente = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="capasense-prueba")
if args.user:
    cliente.username_pw_set(args.user, args.password)
cliente.connect(args.host, args.port)
cliente.loop_start()
time.sleep(0.5)

seq = 0


def publicar(tipo, datos, atraso_s=0):
    """Agrega nodo, seq y hora como lo hace el ESP32 y publica."""
    global seq
    seq += 1
    hora = datetime.now(ZONA_GT) - timedelta(seconds=atraso_s)
    datos.update(nodo="nodo99", seq=seq, ts=hora.strftime("%Y-%m-%dT%H:%M:%S-06:00"))
    cliente.publish(f"{BASE}/{tipo}", json.dumps(datos))
    time.sleep(0.2)


def nivel(ocupacion):
    return "lleno" if ocupacion >= 20 else "por_llenarse" if ocupacion >= 16 else "disponible"


# 1. Mensajes correctos: inicio y entradas hasta llenar la sala.
publicar("conteo", dict(evento="inicio", origen="sistema", ocupacion=17, capacidad=20,
                        estado=nivel(17), acceso="abierto", anulacion=False))
for o in (18, 19, 20):
    publicar("conteo", dict(evento="entrada", origen="sensor", ocupacion=o, capacidad=20,
                            estado=nivel(o), acceso="cerrado" if o >= 20 else "abierto", anulacion=False))
publicar("sala", dict(evento="puerta", puerta="abierta", actividad=True))
publicar("ambiente", dict(fuera_rango=["temp_alta"], temp_c=25.3, hum_pct=52.1, lecturas_dht=6,
                          errores_dht=0, luz_adc=1830, luz_baja=False, ruido_pp=420, ruido_base=180,
                          ruido_alto=False, voltaje_v=5.02))
publicar("alerta", dict(tipo="energia_baja", voltaje_v=4.62))

# 2. Error: se saltan 2 números de secuencia y el estado no coincide con 19 personas.
seq += 2
publicar("conteo", dict(evento="salida", origen="sensor", ocupacion=19, capacidad=20,
                        estado="lleno", acceso="abierto", anulacion=False))

# 3. Mensaje diferido: su hora es 95 s anterior a la de envío.
publicar("conteo", dict(evento="entrada", origen="sensor", ocupacion=20, capacidad=20,
                        estado="lleno", acceso="cerrado", anulacion=False), atraso_s=95)

# 4. Error: mensaje que no es JSON.
cliente.publish(f"{BASE}/ambiente", "esto no es json")
time.sleep(0.5)

cliente.loop_stop()
cliente.disconnect()
print(f"Prueba enviada: {seq - 2} mensajes con seq y 1 mensaje inválido en {BASE}/#")
