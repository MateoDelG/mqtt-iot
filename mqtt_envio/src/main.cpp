#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Configuración de los detalles de la red WiFi
const char *ssid = "ESP32-IU";     // Nombre de la red WiFi
const char *password = "12345678"; // Contraseña de la red WiFi

// Configuración del servidor MQTT
// broker MQTT https://testclient-cloud.mqtt.cool/
const char *mqtt_server = "broker.mqtt.cool"; // Dirección del servidor
#define MQTT_PORT 1883                        // Puerto del servidor MQTT, el puerto estándar no seguro es 1883

WiFiClient espClient;
PubSubClient client(espClient);


void MQTTStateVerification();
float simulationSensorTemperature();
int simulationSensorHumidity();
String serializeJson(float temperature, int humidity);
void publishManager(float temperature, int humidity);

// Función para conectar el ESP32 a la red WiFi
void setup_wifi()
{
  delay(1000); // Pequeño retardo para no saturar el proceso
  Serial.println("Esperando la conexión wifi...");
  // Intenta conectarse a la red WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print("."); // Muestra un punto en el serial para indicar el proceso de conexión
    delay(500);        // Espera 500ms antes de volver a verificar el estado de la conexión
  }
  Serial.println("WiFi conectado"); // Informa cuando la conexión es exitosa
}

void setup()
{
  Serial.begin(115200);                     // Inicia la comunicación serial a 115200 baudios
  Serial.println("Start");                  // Indica el inicio del programa en el monitor serial
  setup_wifi();                             // Llama a la función de configuración de WiFi
  client.setServer(mqtt_server, MQTT_PORT); // Configura el servidor MQTT y el puerto
}

void loop()
{
  MQTTStateVerification(); // Verifica la conexión MQTT y reconecta si es necesario
  float temperature = simulationSensorTemperature();
  int humidity = simulationSensorHumidity();
  publishManager(temperature, humidity);

  client.loop(); // Permite al cliente MQTT procesar cualquier mensaje entrante y mantener la conexión
}

void setup_wifi()
{
  delay(1000); // Pequeño retardo para no saturar el proceso
  Serial.println("Esperando la conexión wifi...");
  // Intenta conectarse a la red WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print("."); // Muestra un punto en el serial para indicar el proceso de conexión
    delay(500);        // Espera 500ms antes de volver a verificar el estado de la conexión
  }
  Serial.println("WiFi conectado"); // Informa cuando la conexión es exitosa
}

void reconnect()
{
  // Mientras el cliente no esté conectado, intenta reconectar
  while (!client.connected())
  {
    Serial.println("Esperando la conexión MQTT...");
    // Intenta conectarse con un ID de cliente MQTT
    if (client.connect("ESP32Client"))
    {
      Serial.println("Conexión MQTT reestablecida"); // Si la conexión es exitosa, lo indica
    }
    else
    {
      // Si la conexión falla, muestra el código de error y espera 5 segundos antes de reintentar
      Serial.print("fallo, rc=");
      Serial.print(client.state());
      Serial.println(" intentar de nuevo en 5 segundos");
      delay(5000);
    }
  }
}

void MQTTStateVerification()
{
  // Verifica la conexión al servidor MQTT y reconecta si es necesario
  if (!client.connected())
  {
    reconnect();
  }
}

void publishManager(float temperature, int humidity)
{
  static unsigned long current_time = 0; // Variable para almacenar el tiempo desde el último mensaje
  const int publish_interval = 20000;     // Intervalo de publicación en milisegundos
  // Publica un mensaje cada 5 segundos
  if (millis() > current_time + publish_interval)
  {

    String msg = serializeJson(temperature, humidity);

    client.publish("ae3/topic", msg.c_str());
    Serial.println("mensaje enviado: " + String(msg)); // Imprime el mensaje enviado al monitor serial
    current_time = millis();                           // Actualiza el tiempo del último mensaje enviado
  }
}

String serializeJson(float temperature, int humidity)
{
JsonDocument doc;

doc["id"] = "12345678";

JsonArray properties = doc["properties"].to<JsonArray>();
properties.add("temperatura");
properties.add("humedad");

JsonArray values = doc["values"].to<JsonArray>();
values.add(temperature);
values.add(humidity);

String output;

doc.shrinkToFit();  // optional

serializeJson(doc, output);
return output;
}

float simulationSensorTemperature()
{
  return random(10, 100) + random(0, 10) / 10.0;
}

int simulationSensorHumidity()
{
  return random(5, 60);
}

void stayAlive()
{
  static unsigned long current_time_alive = 0; // Variable para mensajes de keep-alive
  // Imprime un mensaje cada segundo para mostrar que el programa está ejecutándose
  if (millis() > current_time_alive + 1000)
  {
    Serial.println("ejecutando programa");
    current_time_alive = millis();
  }
}