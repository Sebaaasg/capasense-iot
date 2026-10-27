#!/usr/bin/env python3
"""
CapaSense - Programa receptor (cliente suscriptor) en la Raspberry Pi
CC3108 Internet of Things - UVG - Fase 03

Qué hace:
  1. Se conecta al broker Mosquitto y se suscribe a capasense/+/# (todos los nodos).
  2. Valida cada mensaje: que sea JSON, que traiga los campos esperados y
     que los valores estén en rango.
  3. Detecta mensajes perdidos con el número de secuencia (seq) de cada nodo.
  4. Calcula el retraso entre la hora del nodo (ts) y la hora de llegada.
  5. Revisa que el estado (verde/amarillo/rojo) coincida con la ocupación.
  6. Guarda todo en una base de datos SQLite (capasense.db).
  7. Al cerrar con Ctrl+C muestra un resumen: mensajes por tema, intervalo
     promedio entre mensajes y mensajes perdidos.

Uso:
  python3 receptor_capasense.py --host localhost --user capasense --password CLAVE
"""

import argparse
import json
import signal
import sqlite3
import sys
import time
from datetime import datetime

import paho.mqtt.client as mqtt

TEMA_SUSCRIPCION = "capasense/+/#"
CAPACIDAD_POR_DEFECTO = 20
UMBRAL_AMARILLO = 0.8      # 80 % de la capacidad
RETRASO_DIFERIDO_S = 10    # más de 10 s de retraso = mensaje enviado desde la cola

# Campos obligatorios de cada tipo de mensaje (último nivel del tema).
CAMPOS = {
    "conteo":   ["nodo", "seq", "ts", "evento", "origen", "ocupacion", "capacidad", "estado", "acceso", "anulacion"],
    "sala":     ["nodo", "seq", "ts", "evento", "puerta", "actividad"],
    "ambiente": ["nodo", "seq", "ts", "temp_c", "hum_pct", "luz_adc", "ruido_pp", "voltaje_v", "fuera_rango"],
    "alerta":   ["nodo", "seq", "ts", "tipo", "voltaje_v"],
    "estado":   ["nodo", "seq", "ts", "ocupacion", "estado", "acceso", "rssi_dbm", "uptime_s"],
    "conexion": ["nodo", "conectado"],
}


class Receptor:
    def __init__(self, ruta_db):
        self.db = sqlite3.connect(ruta_db, check_same_thread=False)
        self.db.execute("""
            CREATE TABLE IF NOT EXISTS mensajes (
                id            INTEGER PRIMARY KEY AUTOINCREMENT,
                recibido      TEXT,     -- hora de llegada a la Raspberry Pi
                nodo          TEXT,
                tipo          TEXT,     -- conteo, sala, ambiente, alerta, estado, conexion
                tema          TEXT,
                seq           INTEGER,
                ts_nodo       TEXT,     -- hora del RTC del nodo
                retraso_s     REAL,
                valido        INTEGER,  -- 1 = pasó la validación
                observaciones TEXT,
                payload       TEXT      -- mensaje original completo
            )""")
        self.db.commit()
        self.ultima_seq = {}         # nodo -> último seq recibido
        self.perdidos = {}           # nodo -> mensajes perdidos detectados
        self.llegadas = {}           # tema -> lista de horas de llegada
        self.total = 0

    # ------------------------------------------------------------------ validación
    def validar(self, tipo, datos):
        """Devuelve dos listas: errores (el mensaje no es válido) y avisos
        (el mensaje es válido pero hay algo que el personal debe saber)."""
        errores, avisos = [], []
        if tipo not in CAMPOS:
            return [f"tipo de mensaje desconocido: {tipo}"], avisos

        faltan = [c for c in CAMPOS[tipo] if c not in datos]
        if faltan:
            return ["faltan campos: " + ", ".join(faltan)], avisos

        if tipo in ("conteo", "estado"):
            ocupacion = datos["ocupacion"]
            capacidad = datos.get("capacidad", CAPACIDAD_POR_DEFECTO)
            if not isinstance(ocupacion, int) or not 0 <= ocupacion <= 99:
                errores.append(f"ocupación fuera de rango: {ocupacion}")
            else:
                esperado = estado_esperado(ocupacion, capacidad)
                if datos["estado"] != esperado:
                    errores.append(f"estado {datos['estado']} no coincide con ocupación {ocupacion} (esperado {esperado})")
                if ocupacion > capacidad:
                    avisos.append(f"sobrecupo: {ocupacion} de {capacidad}")

        if tipo == "ambiente":
            t, h, v = datos["temp_c"], datos["hum_pct"], datos["voltaje_v"]
            if t is not None and not -40 <= t <= 80:
                errores.append(f"temperatura fuera del rango del sensor: {t}")
            if h is not None and not 0 <= h <= 100:
                errores.append(f"humedad fuera del rango del sensor: {h}")
            if not 0 <= v <= 6.5:
                errores.append(f"voltaje fuera del rango del FZ0430: {v}")
            if not 0 <= datos["luz_adc"] <= 4095:
                errores.append(f"luz fuera del rango del ADC: {datos['luz_adc']}")
            if datos["fuera_rango"]:
                avisos.append("ambiente fuera de rango: " + ", ".join(datos["fuera_rango"]))
        return errores, avisos

    def revisar_secuencia(self, nodo, seq):
        """Si el seq salta, se perdieron mensajes. Si baja, el nodo se reinició."""
        anterior = self.ultima_seq.get(nodo)
        self.ultima_seq[nodo] = seq
        if anterior is None:
            return None
        if seq == anterior + 1:
            return None
        if seq <= anterior:
            return f"el nodo se reinició (seq {anterior} -> {seq})"
        faltantes = seq - anterior - 1
        self.perdidos[nodo] = self.perdidos.get(nodo, 0) + faltantes
        return f"se perdieron {faltantes} mensaje(s) entre seq {anterior} y {seq}"

    # ------------------------------------------------------------------ recepción
    def procesar(self, tema, texto):
        ahora = datetime.now().astimezone()
        partes = tema.split("/")
        nodo = partes[1] if len(partes) > 1 else "?"
        tipo = partes[2] if len(partes) > 2 else "?"
        self.total += 1
        self.llegadas.setdefault(tema, []).append(time.monotonic())

        try:
            datos = json.loads(texto)
        except json.JSONDecodeError:
            self.guardar(ahora, nodo, tipo, tema, None, None, None, False, "JSON inválido", texto)
            print(f"{hora(ahora)} [ERROR] {tema}: JSON inválido -> {texto}")
            return

        if tipo == "cmd":      # comandos enviados por el personal, solo se registran
            self.guardar(ahora, nodo, tipo, tema, None, None, None, True, "comando", texto)
            print(f"{hora(ahora)} [CMD] {texto}")
            return

        errores, avisos = self.validar(tipo, datos)
        seq = datos.get("seq")
        if isinstance(seq, int):
            aviso = self.revisar_secuencia(nodo, seq)
            if aviso:
                avisos.append(aviso)

        retraso = calcular_retraso(datos.get("ts"), ahora)
        if retraso is not None and retraso > RETRASO_DIFERIDO_S:
            avisos.append(f"diferido {retraso:.0f} s (enviado desde la cola del nodo)")

        valido = not errores
        observaciones = "; ".join(errores + avisos)
        self.guardar(ahora, nodo, tipo, tema, seq, datos.get("ts"), retraso, valido, observaciones, texto)
        marca = "  ERROR: " if errores else "  !! "
        print(f"{hora(ahora)} {resumen(tipo, datos)}" + (marca + observaciones if observaciones else ""))

    def guardar(self, ahora, nodo, tipo, tema, seq, ts, retraso, valido, obs, payload):
        self.db.execute(
            "INSERT INTO mensajes (recibido, nodo, tipo, tema, seq, ts_nodo, retraso_s, valido, observaciones, payload)"
            " VALUES (?,?,?,?,?,?,?,?,?,?)",
            (ahora.isoformat(timespec="seconds"), nodo, tipo, tema, seq, ts,
             None if retraso is None else round(retraso, 1), int(valido), obs, payload))
        self.db.commit()

    def imprimir_resumen(self):
        print("\n================ Resumen de la sesión ================")
        print(f"Mensajes recibidos: {self.total}")
        for tema, horas in sorted(self.llegadas.items()):
            if len(horas) > 1:
                intervalos = [b - a for a, b in zip(horas, horas[1:])]
                promedio = sum(intervalos) / len(intervalos)
                print(f"  {tema:32} {len(horas):4} mensajes, intervalo promedio {promedio:6.1f} s")
            else:
                print(f"  {tema:32} {len(horas):4} mensaje")
        for nodo, n in self.perdidos.items():
            print(f"Mensajes perdidos de {nodo}: {n}")
        print("Detalle guardado en la base de datos SQLite.")


# ---------------------------------------------------------------------- utilidades
def estado_esperado(ocupacion, capacidad):
    if ocupacion >= capacidad:
        return "lleno"
    if ocupacion >= int(capacidad * UMBRAL_AMARILLO):
        return "por_llenarse"
    return "disponible"


def calcular_retraso(ts, ahora):
    try:
        return (ahora - datetime.fromisoformat(ts)).total_seconds()
    except (TypeError, ValueError):
        return None      # el nodo envió "sin_rtc" o no envió hora


def hora(fecha):
    return fecha.strftime("%H:%M:%S")


def resumen(tipo, d):
    """Una línea legible por tipo de mensaje para la consola."""
    if tipo == "conteo":
        return (f"[CONTEO]   {d.get('evento'):22} ocupación {d.get('ocupacion')}/{d.get('capacidad')}"
                f"  {d.get('estado'):12} acceso {d.get('acceso')}  anulación {d.get('anulacion')}")
    if tipo == "sala":
        return f"[SALA]     {d.get('evento'):22} puerta {d.get('puerta')}  actividad {d.get('actividad')}"
    if tipo == "ambiente":
        return (f"[AMBIENTE] {d.get('temp_c')} °C  {d.get('hum_pct')} %HR  luz {d.get('luz_adc')}"
                f"  ruido {d.get('ruido_pp')}  fuente {d.get('voltaje_v')} V  fuera de rango {d.get('fuera_rango')}")
    if tipo == "alerta":
        return f"[ALERTA]   {d.get('tipo')}  fuente {d.get('voltaje_v')} V"
    if tipo == "estado":
        return (f"[ESTADO]   ocupación {d.get('ocupacion')}  {d.get('estado')}  acceso {d.get('acceso')}"
                f"  señal {d.get('rssi_dbm')} dBm  activo {d.get('uptime_s')} s")
    if tipo == "conexion":
        return f"[CONEXIÓN] nodo {d.get('nodo')} {'en línea' if d.get('conectado') else 'DESCONECTADO'}"
    return f"[{tipo}] {d}"


# ---------------------------------------------------------------------- MQTT
def main():
    p = argparse.ArgumentParser(description="Receptor MQTT de CapaSense")
    p.add_argument("--host", default="localhost", help="dirección del broker")
    p.add_argument("--port", type=int, default=1883)
    p.add_argument("--user", default=None)
    p.add_argument("--password", default=None)
    p.add_argument("--db", default="capasense.db", help="archivo SQLite")
    args = p.parse_args()

    receptor = Receptor(args.db)

    def al_conectar(cliente, userdata, flags, codigo, propiedades):
        if codigo.is_failure:
            print(f"[MQTT] Conexión rechazada: {codigo}")
            return
        # Al reconectarse también se vuelve a suscribir.
        cliente.subscribe(TEMA_SUSCRIPCION, qos=1)
        print(f"[MQTT] Conectado a {args.host}:{args.port}, suscrito a {TEMA_SUSCRIPCION}")

    def al_desconectar(cliente, userdata, flags, codigo, propiedades):
        if codigo.is_failure:   # caída inesperada: paho reintenta solo
            print(f"[MQTT] Se perdió la conexión con el broker ({codigo}). Reintentando...")

    def al_recibir(cliente, userdata, msg):
        receptor.procesar(msg.topic, msg.payload.decode("utf-8", errors="replace"))

    cliente = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="capasense-receptor")
    if args.user:
        cliente.username_pw_set(args.user, args.password)
    cliente.on_connect = al_conectar
    cliente.on_disconnect = al_desconectar
    cliente.on_message = al_recibir
    cliente.reconnect_delay_set(min_delay=1, max_delay=10)   # reconexión automática

    def salir(*_):
        receptor.imprimir_resumen()
        cliente.disconnect()
        sys.exit(0)

    signal.signal(signal.SIGINT, salir)
    signal.signal(signal.SIGTERM, salir)

    # Si el broker aún no está listo, se reintenta en lugar de terminar.
    while True:
        try:
            cliente.connect(args.host, args.port, keepalive=30)
            break
        except OSError as e:
            print(f"[MQTT] No se pudo conectar ({e}). Reintento en 5 s...")
            time.sleep(5)
    cliente.loop_forever(retry_first_connection=True)


if __name__ == "__main__":
    main()
