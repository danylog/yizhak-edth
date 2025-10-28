#include "RF24Network.h"
#include "RF24.h"
#include "RF24Mesh.h"
#include <SPI.h>

#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiManager.h>

#include "env_b.h"

WiFiManager wm;

#define CEPin 4
#define CSPin 5

RF24 radio(CEPin, CSPin);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

uint32_t displayTimer = 0;

const uint32_t POST_INTERVAL_MS = 5000UL; // send every 5s
const size_t BUFFER_SIZE = 128;            // stored messages capacity
const size_t MAX_BATCH_SEND = 50;          // max messages per POST

uint32_t lastPostMs = 0;

// Sensor message layout
struct __attribute__((packed)) SensorMsg
{
  uint16_t deviceId;        // numeric device id
  uint32_t timestamp_ms;    // millis()
  int16_t sensor_value;     // temperature in centi-degrees (e.g. 2345 = 23.45°C)
  float battery_voltage; // battery voltage in millivolts (e.g. 3600 = 3.6V)
  float latitude;           // latitude in degrees
  float longitude;          // longitude in degrees
};

// store a received message along with the time master received it
struct StoredMsg
{
  SensorMsg msg;
  uint32_t received_ms;
};

// circular buffer
StoredMsg msgBuffer[BUFFER_SIZE];
size_t bufStart = 0;
size_t bufCount = 0;

// helper to push one message into buffer (drops oldest when full)
void bufferPush(const SensorMsg &m)
{
  size_t idx;
  if (bufCount < BUFFER_SIZE)
  {
    idx = (bufStart + bufCount) % BUFFER_SIZE;
    bufCount++;
  }
  else
  {
    // overwrite oldest
    idx = bufStart;
    bufStart = (bufStart + 1) % BUFFER_SIZE;
  }
  msgBuffer[idx].msg = m;
  msgBuffer[idx].received_ms = millis();
}

// helper to pop N messages (advance start)
void bufferPopN(size_t n)
{
  if (n >= bufCount)
  {
    bufStart = 0;
    bufCount = 0;
  }
  else
  {
    bufStart = (bufStart + n) % BUFFER_SIZE;
    bufCount -= n;
  }
}

// Build JSON payload for up to `n` messages from buffer start
String buildJsonPayload(size_t n)
{
  String s = "[";
  for (size_t i = 0; i < n; ++i)
  {
    size_t idx = (bufStart + i) % BUFFER_SIZE;
    SensorMsg &m = msgBuffer[idx].msg;
    uint32_t rcv = msgBuffer[idx].received_ms;
    if (i)
      s += ",";
    s += "{";
    s += "\"deviceId\":" + String(m.deviceId) + ",";
    s += "\"timestamp_ms\":" + String(m.timestamp_ms) + ",";
    s += "\"sensor_value\":" + String(m.sensor_value) + ",";
    s += "\"battery_voltage\":" + String(m.battery_voltage) + ",";
    s += "\"latitude\":" + String(m.latitude, 7) + ",";
    s += "\"longitude\":" + String(m.longitude, 7) + ",";
    s += "\"received_ms\":" + String(rcv);
    s += "}";
  }
  s += "]";
  return s;
}

// Attempt to POST up to MAX_BATCH_SEND messages; on success remove them from buffer
void postBatchIfNeeded()
{
  if (bufCount == 0)
    return;
  if (millis() - lastPostMs < POST_INTERVAL_MS)
    return;
  lastPostMs = millis();

  size_t toSend = bufCount;
  if (toSend > MAX_BATCH_SEND)
    toSend = MAX_BATCH_SEND;

  // ensure WiFi connected
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi not connected, attempting reconnect...");
    WiFi.reconnect();
    // give short time to reconnect; if fails will try again next interval
    delay(200);
    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println("WiFi reconnect failed");
      return;
    }
  }

  String payload = buildJsonPayload(toSend);

  HTTPClient http;
  http.begin(SERVER_URL);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(payload);
  if (code > 0)
  {
    Serial.print("POST code: ");
    Serial.println(code);
    String resp = http.getString();
    Serial.print("Resp: ");
    Serial.println(resp);

    if (code >= 200 && code < 300)
    {
      bufferPopN(toSend);
      Serial.print("Sent and removed ");
      Serial.print(toSend);
      Serial.println(" msgs from buffer");
    }
    else
    {
      Serial.println("Server error, will retry later");
    }
  }
  else
  {
    Serial.print("HTTP POST failed, err: ");
    Serial.println(code);
  }
  http.end();
}

void setup()
{
  Serial.begin(115200);
  bool res;
  res = wm.autoConnect("AutoConnectAP", "password"); // password protected ap
  while (!Serial)
  {
    // some boards need this because of native USB capability
  }

  // Set the nodeID to 0 for the master node
  mesh.setNodeID(0);
  Serial.println(mesh.getNodeID());

  // Set the PA Level to MIN and disable LNA for testing & power supply related issues
  radio.begin();
  radio.setPALevel(RF24_PA_MIN, 0);

  // Connect to the mesh
  if (!mesh.begin())
  {
    // if mesh.begin() returns false for a master node, then radio.begin() returned false.
    Serial.println(F("Radio hardware not responding."));
    while (1)
    {
      // hold in an infinite loop
    }
  }
}

void loop()
{

  // Call mesh.update to keep the network updated
  mesh.update();

  // In addition, keep the 'DHCP service' running on the master node so addresses will
  // be assigned to the sensor nodes
  mesh.DHCP();

  // Check for incoming data from the sensors
  if (network.available())
  {
    RF24NetworkHeader header;
    SensorMsg msg;

    network.peek(header);

    uint32_t dat = 0;
    switch (header.type)
    {
    // Display the incoming millis() values from the sensor nodes
    case 'M':
      network.read(header, &msg, sizeof(msg));
      // Serial.print("battery voltage: ");
      // Serial.println(msg.battery_voltage,3);
      // Serial.print("sensor value: ");
      // Serial.println(msg.sensor_value);
      // Serial.print("latitude:");
      // Serial.println(msg.latitude, 7);
      // Serial.print("longitude:");
      // Serial.println(msg.longitude, 7);

      // buffer the incoming message for batch upload
      bufferPush(msg);
      // echo minimal info to serial
      Serial.print("Buffered from ");
      Serial.print(msg.deviceId);
      Serial.print(" ts:");
      Serial.print(msg.timestamp_ms);
      Serial.print(" bufCount=");
      Serial.println(bufCount);
      break;
    default:
      network.read(header, 0, 0);
      Serial.println(header.type);
      break;
    }
  }

  // periodically try to POST buffered messages
  postBatchIfNeeded();
}
