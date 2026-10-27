/*
 * CapaSense - Nodo IoT (ESP32-WROOM-32, tarjeta de 30 pines)
 * CC3108 Internet of Things - UVG - Fase 03
 *
 * Qué hace este programa:
 *   - Cuenta entradas (HC-SR04) y salidas (barrera IR) y calcula la ocupación.
 *   - Controla el semáforo (3 LEDs), la alarma (buzzer) y la barrera (servo)
 *     DIRECTAMENTE en el ESP32, sin esperar a la red.
 *   - Lee puerta (reed switch), actividad (PIR), temperatura y humedad (DHT22),
 *     luz (LDR), ruido (módulo de sonido), voltaje de la fuente del servo (FZ0430)
 *     y la hora (RTC).
 *   - Publica todo por MQTT hacia el broker (Raspberry Pi o laptop).
 *
 * Organización en dos tareas (el ESP32 tiene dos núcleos):
 *   - Núcleo 1 (loop): sensores y salidas. Nunca espera a la red.
 *   - Núcleo 0 (tareaRed): WiFi y MQTT. Si el broker se cae, los mensajes
 *     quedan en una cola y se envían al reconectarse.
 *
 * Librerías (Gestor de librerías del Arduino IDE):
 *   PubSubClient (Nick O'Leary), ArduinoJson (Benoit Blanchon, v7),
 *   DHT sensor library + Adafruit Unified Sensor, RTClib (Adafruit).
 * Núcleo de placa: "esp32" de Espressif (compilado con la versión 3.1.1).
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <RTClib.h>
#include <DHT.h>

// ============================================================================
// 1. CONFIGURACIÓN (lo único que normalmente hay que cambiar)
// ============================================================================

// 1 = simulación en Wokwi, 0 = circuito real
#define MODO_WOKWI 1

#if MODO_WOKWI
  // Wokwi solo tiene salida a Internet, por eso se usa un broker público.
  // El broker de la laptop/Raspberry recibe estos mensajes mediante un puente.
  const char* WIFI_SSID  = "Wokwi-GUEST";
  const char* WIFI_PASS  = "";
  const char* MQTT_HOST  = "broker.hivemq.com";
  const char* MQTT_USER  = nullptr;   // el broker público no usa usuario
  const char* MQTT_PASS  = nullptr;
  RTC_DS1307 rtc;                      // Wokwi no tiene DS3231
  const bool BUZZER_ACTIVO_EN_BAJO = false;     // buzzer de 2 pines de Wokwi
  const uint32_t PIR_CALENTAMIENTO_MS = 5000;   // en Wokwi no hace falta esperar
#else
  const char* WIFI_SSID  = "NOMBRE_DE_LA_RED";  // red local del aula o hotspot
  const char* WIFI_PASS  = "CLAVE_DE_LA_RED";
  const char* MQTT_HOST  = "192.168.1.50";      // IP de la Raspberry Pi o laptop
  const char* MQTT_USER  = "capasense";         // usuario creado en Mosquitto
  const char* MQTT_PASS  = "CLAVE_MQTT";
  RTC_DS3231 rtc;
  const bool BUZZER_ACTIVO_EN_BAJO = true;      // la mayoría de módulos se activa con LOW
  const uint32_t PIR_CALENTAMIENTO_MS = 120000; // el PIR necesita ~2 min al encender
#endif

const uint16_t MQTT_PORT = 1883;
const char* ID_NODO      = "nodo01";
const char* ID_CLIENTE   = "capasense-nodo01";   // identificador del cliente MQTT

// Temas MQTT
const char* TEMA_CONTEO   = "capasense/nodo01/conteo";
const char* TEMA_SALA     = "capasense/nodo01/sala";
const char* TEMA_AMBIENTE = "capasense/nodo01/ambiente";
const char* TEMA_ALERTA   = "capasense/nodo01/alerta";
const char* TEMA_ESTADO   = "capasense/nodo01/estado";
const char* TEMA_CONEXION = "capasense/nodo01/conexion";
const char* TEMA_CMD      = "capasense/nodo01/cmd";

// Capacidad y umbrales
const int   CAPACIDAD_MAXIMA     = 20;
const int   UMBRAL_AMARILLO      = 16;     // 80 % de 20
const float DISTANCIA_CRUCE_CM   = 15.0;   // menos de 15 cm = alguien cruza
const float DISTANCIA_LIBRE_CM   = 18.0;   // más de 18 cm = el paso quedó libre
const float TEMP_MIN = 18.0, TEMP_MAX = 24.0;   // °C
const float HUM_MIN  = 40.0, HUM_MAX  = 60.0;   // %HR
const float VOLTAJE_MINIMO       = 4.8;    // V, mínimo del servo SG90
const float VOLTAJE_RESTABLECIDO = 4.9;    // V, para no alternar alertas
const float FACTOR_DIVISOR       = 5.0;    // FZ0430: 30k / 7.5k
const float CORRECCION_VOLTAJE   = 0.0;    // ajuste con multímetro (V)
const int   UMBRAL_LUZ_BAJA      = 2500;   // calibrar: en estos módulos más luz = valor menor
const int   MARGEN_RUIDO         = 300;    // cuentas sobre la línea base
const uint8_t BRILLO_ALTO = 255, BRILLO_BAJO = 70;   // PWM de 8 bits
const int   SERVO_ABIERTO = 0, SERVO_CERRADO = 90;  // grados

// Intervalos (ms)
const uint32_t T_ULTRASONICO = 100;
const uint32_t T_PIR         = 1000;
const uint32_t T_RUIDO       = 2000;
const uint32_t T_AMBIENTE    = 5000;     // captura de DHT22, LDR y voltaje
const uint32_t T_PUB_AMBIENTE = 30000;   // envío del resumen ambiental
const uint32_t T_PUB_ESTADO   = 60000;   // envío del estado general
const uint32_t T_BUZZER       = 2000;    // duración de la alarma al llenarse
const uint32_t T_PULSACION_LARGA = 2000; // S3: más de 2 s suma 1

// ============================================================================
// 2. PINES (Tabla 4 de la Fase 02)
// ============================================================================
const uint8_t PIN_LDR = 34, PIN_SONIDO = 35, PIN_VOLTAJE = 36;
const uint8_t PIN_REED = 25, PIN_BARRERA_IR = 33, PIN_PIR = 32;
const uint8_t PIN_S2 = 13, PIN_S3 = 23;
const uint8_t PIN_DHT = 14, PIN_ECHO = 27, PIN_TRIG = 26;
const uint8_t PIN_SDA = 21, PIN_SCL = 22;
const uint8_t PIN_LED_VERDE = 16, PIN_LED_AMARILLO = 17, PIN_LED_ROJO = 4;
const uint8_t PIN_BUZZER = 18, PIN_SERVO = 19;

// Canales PWM (LEDC). El canal N usa el temporizador (N/2).
// Servo en canal 0 (temporizador 0, 50 Hz); LEDs en canales 2-4
// (temporizadores 1 y 2, 5 kHz). Así no comparten frecuencia.
const uint8_t CANAL_SERVO = 0, CANAL_VERDE = 2, CANAL_AMARILLO = 3, CANAL_ROJO = 4;

// ============================================================================
// 3. ESTADO DEL SISTEMA
// ============================================================================
enum Nivel { DISPONIBLE, POR_LLENARSE, LLENO };
const char* NOMBRE_NIVEL[] = {"disponible", "por_llenarse", "lleno"};

int ocupacion = 0;
bool anulacionActiva = false;
bool barreraCerrada  = false;
Nivel nivelActual    = DISPONIBLE;
bool buzzerEncendido = false;
uint32_t tInicioBuzzer = 0;

bool luzBaja = false;
bool puertaAbierta = false;
bool actividad = false;
bool alertaEnergia = false;
bool rtcDisponible = false;

// Acumuladores del resumen ambiental (se reinician cada 30 s)
float sumaTemp = 0, sumaHum = 0;
int lecturasDht = 0, erroresDht = 0;
int ultimaLuz = 0, ruidoMaxPeriodo = 0;
float ultimoVoltaje = 0;
int lecturasVoltajeBajo = 0;

// Ruido: línea base y detección sostenida
long sumaBaseRuido = 0;
int ventanasBase = 0, lineaBaseRuido = 0, ventanasRuidoAlto = 0;
bool ruidoAlto = false;
const int VENTANAS_CALIBRACION = 10;   // 10 ventanas x 2 s = 20 s con la sala vacía

uint32_t secuencia = 0;          // número de mensaje, permite detectar pérdidas
uint32_t mensajesDescartados = 0;
volatile bool mqttConectado = false;

DHT dht(PIN_DHT, DHT22);
WiFiClient clienteWifi;
PubSubClient mqtt(clienteWifi);

// Cola de mensajes salientes: el loop escribe, la tarea de red envía.
struct MensajeMqtt { char tema[40]; char payload[384]; };
const int TAMANO_COLA = 40;
QueueHandle_t colaSalida;

// Cola de comandos recibidos por MQTT: la tarea de red escribe, el loop aplica.
enum TipoComando { CMD_ANULAR, CMD_RESTAURAR, CMD_AJUSTAR };
struct Comando { TipoComando tipo; int valor; };
QueueHandle_t colaComandos;

// ============================================================================
// 4. ENTRADAS DIGITALES CON ANTIRREBOTE
//    Una lectura solo se acepta si se mantiene igual durante "msEstable".
// ============================================================================
struct EntradaDigital {
  uint8_t pin;
  uint16_t msEstable;
  bool estable;        // estado aceptado (true = nivel bajo / activo)
  bool ultimaLectura;
  uint32_t tCambio;
};

EntradaDigital botonS2    = {PIN_S2, 30, false, false, 0};
EntradaDigital botonS3    = {PIN_S3, 30, false, false, 0};
EntradaDigital barreraIR  = {PIN_BARRERA_IR, 20, false, false, 0};
EntradaDigital sensorReed = {PIN_REED, 50, false, false, 0};

// Devuelve true solo en el momento en que cambia el estado estable.
bool actualizarEntrada(EntradaDigital& e) {
  bool lectura = (digitalRead(e.pin) == LOW);   // con pull-up, LOW = activo
  if (lectura != e.ultimaLectura) {
    e.ultimaLectura = lectura;
    e.tCambio = millis();
  }
  if (lectura != e.estable && millis() - e.tCambio >= e.msEstable) {
    e.estable = lectura;
    return true;
  }
  return false;
}

// ============================================================================
// 5. UTILIDADES: hora, PWM y envío de mensajes
// ============================================================================

// Hora del RTC en formato ISO 8601 con zona de Guatemala (UTC-6).
void obtenerHora(char* destino, size_t tam) {
  if (!rtcDisponible) { snprintf(destino, tam, "sin_rtc"); return; }
  DateTime t = rtc.now();
  snprintf(destino, tam, "%04d-%02d-%02dT%02d:%02d:%02d-06:00",
           t.year(), t.month(), t.day(), t.hour(), t.minute(), t.second());
}

double redondear(float valor, int decimales) {
  double f = pow(10, decimales);
  return round(valor * f) / f;
}

void iniciarPwm(uint8_t pin, uint32_t frecuencia, uint8_t resolucion, uint8_t canal) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(pin, frecuencia, resolucion, canal);
#else
  ledcSetup(canal, frecuencia, resolucion);
  ledcAttachPin(pin, canal);
#endif
}

void escribirPwm(uint8_t pin, uint8_t canal, uint32_t ciclo) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, ciclo);
#else
  ledcWrite(canal, ciclo);
#endif
}

// Servo: periodo de 20 ms (50 Hz), pulso de 500 a 2400 µs, PWM de 14 bits.
void moverServo(int grados) {
  uint32_t pulsoUs = map(grados, 0, 180, 500, 2400);
  uint32_t ciclo = pulsoUs * 16383UL / 20000UL;
  escribirPwm(PIN_SERVO, CANAL_SERVO, ciclo);
}

void controlarBuzzer(bool encender) {
  buzzerEncendido = encender;
  digitalWrite(PIN_BUZZER, BUZZER_ACTIVO_EN_BAJO ? !encender : encender);
}

// Agrega nodo, número de secuencia y hora, y pone el mensaje en la cola.
// Si la cola está llena (broker caído mucho tiempo), se descarta el más viejo.
void encolar(const char* tema, JsonDocument& doc) {
  char hora[32];
  obtenerHora(hora, sizeof(hora));
  doc["nodo"] = ID_NODO;
  doc["seq"]  = ++secuencia;
  doc["ts"]   = hora;

  MensajeMqtt m;
  strlcpy(m.tema, tema, sizeof(m.tema));
  serializeJson(doc, m.payload, sizeof(m.payload));

  if (xQueueSend(colaSalida, &m, 0) != pdTRUE) {
    MensajeMqtt descartado;
    xQueueReceive(colaSalida, &descartado, 0);
    xQueueSend(colaSalida, &m, 0);
    mensajesDescartados++;
  }
  Serial.printf("[COLA] %s %s\n", m.tema, m.payload);
}

// ============================================================================
// 6. LÓGICA DE OCUPACIÓN Y SALIDAS
// ============================================================================
Nivel calcularNivel(int personas) {
  if (personas >= CAPACIDAD_MAXIMA) return LLENO;
  if (personas >= UMBRAL_AMARILLO)  return POR_LLENARSE;
  return DISPONIBLE;
}

// Aplica el estado actual a LEDs, barrera y alarma. Se llama cada vez que
// cambia la ocupación, la anulación o el nivel de luz.
void actualizarSalidas() {
  Nivel nuevo = calcularNivel(ocupacion);
  uint8_t brillo = luzBaja ? BRILLO_BAJO : BRILLO_ALTO;

  escribirPwm(PIN_LED_VERDE,    CANAL_VERDE,    nuevo == DISPONIBLE   ? brillo : 0);
  escribirPwm(PIN_LED_AMARILLO, CANAL_AMARILLO, nuevo == POR_LLENARSE ? brillo : 0);
  escribirPwm(PIN_LED_ROJO,     CANAL_ROJO,     nuevo == LLENO        ? brillo : 0);

  // La barrera se cierra solo si la sala está llena y no hay anulación.
  bool cerrar = (nuevo == LLENO) && !anulacionActiva;
  if (cerrar != barreraCerrada) {
    barreraCerrada = cerrar;
    moverServo(cerrar ? SERVO_CERRADO : SERVO_ABIERTO);
  }

  // La alarma suena 2 s al momento de llenarse.
  if (nuevo == LLENO && nivelActual != LLENO) {
    controlarBuzzer(true);
    tInicioBuzzer = millis();
  }
  if (nuevo != LLENO && buzzerEncendido) controlarBuzzer(false);

  nivelActual = nuevo;
}

void publicarConteo(const char* evento, const char* origen) {
  JsonDocument doc;
  doc["evento"]     = evento;
  doc["origen"]     = origen;
  doc["ocupacion"]  = ocupacion;
  doc["capacidad"]  = CAPACIDAD_MAXIMA;
  doc["estado"]     = NOMBRE_NIVEL[nivelActual];
  doc["acceso"]     = barreraCerrada ? "cerrado" : "abierto";
  doc["anulacion"]  = anulacionActiva;
  encolar(TEMA_CONTEO, doc);
}

// Primero se actúa sobre las salidas y después se publica.
void cambiarOcupacion(int cambio, const char* evento, const char* origen) {
  ocupacion = max(0, ocupacion + cambio);
  actualizarSalidas();
  publicarConteo(evento, origen);
}

void cambiarAnulacion(bool activar, const char* origen) {
  if (anulacionActiva == activar) return;
  anulacionActiva = activar;
  actualizarSalidas();
  publicarConteo(activar ? "anulacion_activada" : "anulacion_desactivada", origen);
}

// ============================================================================
// 7. LECTURA DE SENSORES
// ============================================================================

// HC-SR04: devuelve la distancia en cm, o -1 si no hubo eco.
float medirDistanciaCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long duracion = pulseIn(PIN_ECHO, HIGH, 25000UL);  // máx. ~4 m
  if (duracion == 0) return -1;
  return duracion / 58.0;
}

// Una entrada se cuenta cuando hay 2 lecturas seguidas por debajo de 15 cm.
// No se vuelve a contar hasta que haya 2 lecturas seguidas con el paso libre.
bool personaEnEntrada = false;
int lecturasCerca = 0, lecturasLibre = 0;

void revisarEntrada() {
  float d = medirDistanciaCm();
  bool cerca = (d > 2 && d < DISTANCIA_CRUCE_CM);
  bool libre = (d < 0 || d > DISTANCIA_LIBRE_CM);

  lecturasCerca = cerca ? lecturasCerca + 1 : 0;
  lecturasLibre = libre ? lecturasLibre + 1 : 0;

  if (!personaEnEntrada && lecturasCerca >= 2) {
    personaEnEntrada = true;
    cambiarOcupacion(+1, "entrada", "sensor");
  } else if (personaEnEntrada && lecturasLibre >= 2) {
    personaEnEntrada = false;
  }
}

// Barrera IR: cuenta una salida cuando el haz se interrumpe.
// Se ignoran nuevos cortes durante 500 ms (por ejemplo, las dos piernas).
uint32_t tUltimaSalida = 0;

void revisarSalida() {
  if (actualizarEntrada(barreraIR) && barreraIR.estable) {
    if (millis() - tUltimaSalida > 500) {
      tUltimaSalida = millis();
      cambiarOcupacion(-1, "salida", "sensor");
    }
  }
}

// S2: cada pulsación activa o desactiva la anulación.
// S3: pulsación corta resta 1, pulsación de 2 s o más suma 1.
uint32_t tInicioS3 = 0;

void revisarBotones() {
  if (actualizarEntrada(botonS2) && botonS2.estable) {
    cambiarAnulacion(!anulacionActiva, "boton_s2");
  }
  if (actualizarEntrada(botonS3)) {
    if (botonS3.estable) {
      tInicioS3 = millis();                       // se presionó
    } else {
      uint32_t duracion = millis() - tInicioS3;   // se soltó
      if (duracion >= T_PULSACION_LARGA) cambiarOcupacion(+1, "correccion_suma", "boton_s3");
      else                               cambiarOcupacion(-1, "correccion_resta", "boton_s3");
    }
  }
}

void publicarSala(const char* evento) {
  JsonDocument doc;
  doc["evento"]    = evento;
  doc["puerta"]    = puertaAbierta ? "abierta" : "cerrada";
  doc["actividad"] = actividad;
  encolar(TEMA_SALA, doc);
}

// Reed switch: con el imán cerca el contacto cierra (LOW) = puerta cerrada.
void revisarPuerta() {
  if (actualizarEntrada(sensorReed)) {
    puertaAbierta = !sensorReed.estable;
    publicarSala("puerta");
  }
}

void revisarPir() {
  if (millis() < PIR_CALENTAMIENTO_MS) return;
  bool lectura = digitalRead(PIN_PIR) == HIGH;
  if (lectura != actividad) {
    actividad = lectura;
    publicarSala("actividad");
  }
}

// Ruido: valor pico a pico en una ventana de 50 ms.
// Las primeras 10 ventanas forman la línea base de la sala vacía.
void revisarRuido() {
  int minimo = 4095, maximo = 0;
  uint32_t inicio = millis();
  while (millis() - inicio < 50) {
    int v = analogRead(PIN_SONIDO);
    minimo = min(minimo, v);
    maximo = max(maximo, v);
  }
  int picoAPico = maximo - minimo;

  if (ventanasBase < VENTANAS_CALIBRACION) {
    sumaBaseRuido += picoAPico;
    ventanasBase++;
    lineaBaseRuido = sumaBaseRuido / ventanasBase;
    return;
  }
  ruidoMaxPeriodo = max(ruidoMaxPeriodo, picoAPico);
  ventanasRuidoAlto = (picoAPico > lineaBaseRuido + MARGEN_RUIDO) ? ventanasRuidoAlto + 1 : 0;
  ruidoAlto = (ventanasRuidoAlto >= 3);   // 3 ventanas seguidas = ruido sostenido
}

int promedioAnalogico(uint8_t pin, int muestras) {
  long suma = 0;
  for (int i = 0; i < muestras; i++) suma += analogRead(pin);
  return suma / muestras;
}

void publicarAlerta(const char* tipo) {
  JsonDocument doc;
  doc["tipo"]      = tipo;
  doc["voltaje_v"] = redondear(ultimoVoltaje, 2);
  encolar(TEMA_ALERTA, doc);
}

// Captura cada 5 s: DHT22, LDR y voltaje de la fuente del servo.
void revisarAmbiente() {
  // DHT22: se descartan lecturas fallidas o fuera del rango del sensor.
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t) && !isnan(h) && t > -40 && t < 80 && h >= 0 && h <= 100) {
    sumaTemp += t;
    sumaHum += h;
    lecturasDht++;
  } else {
    erroresDht++;
  }

  // LDR: con margen de 100 cuentas para que el brillo no oscile.
  ultimaLuz = promedioAnalogico(PIN_LDR, 8);
  bool nuevaLuzBaja = luzBaja ? (ultimaLuz > UMBRAL_LUZ_BAJA - 100)
                              : (ultimaLuz > UMBRAL_LUZ_BAJA + 100);
  if (nuevaLuzBaja != luzBaja) {
    luzBaja = nuevaLuzBaja;
    actualizarSalidas();
  }

  // Voltaje: promedio de 16 lecturas calibradas en mV, multiplicado por 5.
  long sumaMv = 0;
#if MODO_WOKWI
  // Wokwi no simula la atenuación de 2.5 dB: se lee en el rango normal de 0 a 3.3 V.
  for (int i = 0; i < 16; i++) sumaMv += analogRead(PIN_VOLTAJE) * 3300L / 4095;
#else
  for (int i = 0; i < 16; i++) sumaMv += analogReadMilliVolts(PIN_VOLTAJE);
#endif
  ultimoVoltaje = (sumaMv / 16.0) / 1000.0 * FACTOR_DIVISOR + CORRECCION_VOLTAJE;

  // Alerta con 2 lecturas seguidas bajas; se libera al superar 4.9 V.
  lecturasVoltajeBajo = (ultimoVoltaje < VOLTAJE_MINIMO) ? lecturasVoltajeBajo + 1 : 0;
  if (!alertaEnergia && lecturasVoltajeBajo >= 2) {
    alertaEnergia = true;
    publicarAlerta("energia_baja");
  } else if (alertaEnergia && ultimoVoltaje >= VOLTAJE_RESTABLECIDO) {
    alertaEnergia = false;
    publicarAlerta("energia_restablecida");
  }
}

// Resumen ambiental cada 30 s: promedio de temperatura y humedad,
// última luz, ruido máximo del periodo y voltaje.
void publicarAmbiente() {
  JsonDocument doc;
  JsonArray fueraDeRango = doc["fuera_rango"].to<JsonArray>();

  if (lecturasDht > 0) {
    float temp = sumaTemp / lecturasDht;
    float hum  = sumaHum / lecturasDht;
    doc["temp_c"]  = redondear(temp, 1);
    doc["hum_pct"] = redondear(hum, 1);
    if (temp < TEMP_MIN) fueraDeRango.add("temp_baja");
    if (temp > TEMP_MAX) fueraDeRango.add("temp_alta");
    if (hum < HUM_MIN)   fueraDeRango.add("hum_baja");
    if (hum > HUM_MAX)   fueraDeRango.add("hum_alta");
  } else {
    doc["temp_c"]  = nullptr;   // ninguna lectura válida en el periodo
    doc["hum_pct"] = nullptr;
  }
  doc["lecturas_dht"] = lecturasDht;
  doc["errores_dht"]  = erroresDht;
  doc["luz_adc"]      = ultimaLuz;
  doc["luz_baja"]     = luzBaja;
  doc["ruido_pp"]     = ruidoMaxPeriodo;
  doc["ruido_base"]   = lineaBaseRuido;
  doc["ruido_alto"]   = ruidoAlto;
  doc["voltaje_v"]    = redondear(ultimoVoltaje, 2);
  if (ruidoAlto)     fueraDeRango.add("ruido_alto");
  if (alertaEnergia) fueraDeRango.add("energia_baja");

  encolar(TEMA_AMBIENTE, doc);

  sumaTemp = sumaHum = 0;
  lecturasDht = erroresDht = 0;
  ruidoMaxPeriodo = 0;
}

// Estado general cada 60 s. Solo se envía con conexión, porque guardarlo
// en la cola no aporta: al reconectarse se envía uno nuevo.
void publicarEstado() {
  if (!mqttConectado) return;
  JsonDocument doc;
  doc["ocupacion"]   = ocupacion;
  doc["estado"]      = NOMBRE_NIVEL[nivelActual];
  doc["acceso"]      = barreraCerrada ? "cerrado" : "abierto";
  doc["anulacion"]   = anulacionActiva;
  doc["rssi_dbm"]    = WiFi.RSSI();
  doc["uptime_s"]    = millis() / 1000;
  doc["descartados"] = mensajesDescartados;
  encolar(TEMA_ESTADO, doc);
}

// Aplica en el loop los comandos que llegaron por MQTT.
void procesarComandos() {
  Comando c;
  while (xQueueReceive(colaComandos, &c, 0) == pdTRUE) {
    if (c.tipo == CMD_ANULAR)    cambiarAnulacion(true, "mqtt");
    if (c.tipo == CMD_RESTAURAR) cambiarAnulacion(false, "mqtt");
    if (c.tipo == CMD_AJUSTAR && c.valor >= 0 && c.valor <= 99) {
      cambiarOcupacion(c.valor - ocupacion, "ajuste_remoto", "mqtt");
    }
  }
}

// ============================================================================
// 8. TAREA DE RED (núcleo 0): WiFi, MQTT y vaciado de la cola
// ============================================================================

// Comandos aceptados en capasense/nodo01/cmd:
//   {"accion":"anular"}  {"accion":"restaurar"}  {"accion":"ajustar","ocupacion":12}
void alRecibirMensaje(char* tema, byte* payload, unsigned int largo) {
  JsonDocument doc;
  if (deserializeJson(doc, payload, largo)) {
    Serial.println("[CMD] JSON inválido, se ignora");
    return;
  }
  const char* accion = doc["accion"] | "";
  Comando c = {CMD_ANULAR, 0};
  if      (strcmp(accion, "anular") == 0)    c.tipo = CMD_ANULAR;
  else if (strcmp(accion, "restaurar") == 0) c.tipo = CMD_RESTAURAR;
  else if (strcmp(accion, "ajustar") == 0)  { c.tipo = CMD_AJUSTAR; c.valor = doc["ocupacion"] | -1; }
  else { Serial.printf("[CMD] Acción desconocida: %s\n", accion); return; }
  xQueueSend(colaComandos, &c, 0);
  Serial.printf("[CMD] Recibido: %s\n", accion);
}

bool conectarMqtt() {
  char ultimaVoluntad[64];
  snprintf(ultimaVoluntad, sizeof(ultimaVoluntad), "{\"nodo\":\"%s\",\"conectado\":false}", ID_NODO);

  // Si el nodo se desconecta sin avisar, el broker publica la "última voluntad".
  if (!mqtt.connect(ID_CLIENTE, MQTT_USER, MQTT_PASS, TEMA_CONEXION, 1, true, ultimaVoluntad)) {
    // -2 = no se pudo abrir la conexión con el broker (dirección, puerto o red)
    Serial.printf("[MQTT] Falló la conexión con %s, código %d\n", MQTT_HOST, mqtt.state());
    return false;
  }
  char aviso[96];
  snprintf(aviso, sizeof(aviso), "{\"nodo\":\"%s\",\"conectado\":true,\"ip\":\"%s\"}",
           ID_NODO, WiFi.localIP().toString().c_str());
  mqtt.publish(TEMA_CONEXION, aviso, true);   // retenido
  mqtt.subscribe(TEMA_CMD, 1);
  Serial.println("[MQTT] Conectado al broker");
  return true;
}

// Envía hasta 10 mensajes por vuelta. Un mensaje solo sale de la cola
// cuando el envío fue aceptado; si falla, se reintenta después.
void enviarPendientes() {
  MensajeMqtt m;
  for (int i = 0; i < 10 && xQueuePeek(colaSalida, &m, 0) == pdTRUE; i++) {
    if (!mqtt.publish(m.tema, m.payload)) break;
    xQueueReceive(colaSalida, &m, 0);
  }
}

void tareaRed(void* parametro) {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(alRecibirMensaje);
  mqtt.setBufferSize(512);
  mqtt.setKeepAlive(15);   // el broker detecta la caída en ~22 s

  uint32_t tUltimoIntento = 0;
  bool wifiAvisado = false;
  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      mqttConectado = false;
      wifiAvisado = false;
    } else if (!mqtt.connected()) {
      mqttConectado = false;
      if (!wifiAvisado) {
        Serial.printf("[WiFi] Conectado, IP %s\n", WiFi.localIP().toString().c_str());
        wifiAvisado = true;
      }
      if (millis() - tUltimoIntento > 5000) {   // reintento cada 5 s
        tUltimoIntento = millis();
        mqttConectado = conectarMqtt();
      }
    } else {
      mqttConectado = true;
      mqtt.loop();
      enviarPendientes();
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ============================================================================
// 9. SETUP Y LOOP (núcleo 1)
// ============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\nCapaSense - nodo01 iniciando");

  // Salidas: el buzzer se deja apagado antes de configurarlo.
  digitalWrite(PIN_BUZZER, BUZZER_ACTIVO_EN_BAJO ? HIGH : LOW);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  iniciarPwm(PIN_SERVO, 50, 14, CANAL_SERVO);
  iniciarPwm(PIN_LED_VERDE, 5000, 8, CANAL_VERDE);
  iniciarPwm(PIN_LED_AMARILLO, 5000, 8, CANAL_AMARILLO);
  iniciarPwm(PIN_LED_ROJO, 5000, 8, CANAL_ROJO);
  moverServo(SERVO_ABIERTO);

  // Entradas digitales con pull-up interno; el PIR y ECHO no lo necesitan.
  pinMode(PIN_REED, INPUT_PULLUP);
  pinMode(PIN_BARRERA_IR, INPUT_PULLUP);
  pinMode(PIN_S2, INPUT_PULLUP);
  pinMode(PIN_S3, INPUT_PULLUP);
  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_ECHO, INPUT);

  // Entradas analógicas (ADC1). El voltaje usa atenuación de 2.5 dB.
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_VOLTAJE, ADC_2_5db);

  dht.begin();

  // RTC: si perdió la hora, se ajusta con la hora de compilación.
  Wire.begin(PIN_SDA, PIN_SCL);
  rtcDisponible = rtc.begin();
  if (!rtcDisponible) {
    Serial.println("[RTC] No responde, los mensajes llevarán ts=\"sin_rtc\"");
  }
#if MODO_WOKWI
  else if (!rtc.isrunning()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
#else
  else if (rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
#endif

  // Estado inicial de la puerta.
  sensorReed.estable = sensorReed.ultimaLectura = (digitalRead(PIN_REED) == LOW);
  puertaAbierta = !sensorReed.estable;

  colaSalida   = xQueueCreate(TAMANO_COLA, sizeof(MensajeMqtt));
  colaComandos = xQueueCreate(10, sizeof(Comando));
  xTaskCreatePinnedToCore(tareaRed, "tareaRed", 8192, nullptr, 1, nullptr, 0);

  actualizarSalidas();
  publicarConteo("inicio", "sistema");
}

void loop() {
  static uint32_t tUltra = 0, tPir = 0, tRuido = 0, tAmb = 0, tPubAmb = 0, tPubEst = 0;
  uint32_t ahora = millis();

  // En cada vuelta: botones, barrera IR, puerta y comandos.
  revisarBotones();
  revisarSalida();
  revisarPuerta();
  procesarComandos();

  if (buzzerEncendido && ahora - tInicioBuzzer >= T_BUZZER) controlarBuzzer(false);

  if (ahora - tUltra  >= T_ULTRASONICO)  { tUltra = ahora;  revisarEntrada(); }
  if (ahora - tPir    >= T_PIR)          { tPir = ahora;    revisarPir(); }
  if (ahora - tRuido  >= T_RUIDO)        { tRuido = ahora;  revisarRuido(); }
  if (ahora - tAmb    >= T_AMBIENTE)     { tAmb = ahora;    revisarAmbiente(); }
  if (ahora - tPubAmb >= T_PUB_AMBIENTE) { tPubAmb = ahora; publicarAmbiente(); }
  if (ahora - tPubEst >= T_PUB_ESTADO)   { tPubEst = ahora; publicarEstado(); }
}
