#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#include <SPI.h>
#include <printf.h>
#include <stdint.h>

#include "env_a.h"

#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#define RXPin 4
#define TXPin 3

#define CEPin 9
#define CSPin 8

#define GPSBaud 9600

TinyGPSPlus gps;

SoftwareSerial ss(RXPin, TXPin);

RF24 radio(CEPin, CSPin);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

uint32_t displayTimer = 0;
// Backoff timestamp to avoid repeated renewAddress attempts
uint32_t lastRenewAttempt = 0;

#define MSG_VERSION 1
#define DEVICE_ID 12 // add/update in env.h as a numeric id

//Message to send to ESP32
struct __attribute__((packed)) SensorMsg
{
  uint16_t deviceId;        // numeric device id
  uint32_t timestamp_ms;    // millis()
  int16_t sensor_value;     // temperature in centi-degrees (e.g. 2345 = 23.45°C)
  float battery_voltage;    // battery voltage in millivolts (e.g. 3600 = 3.6V)
  float latitude;           // latitude in degrees
  float longitude;          // longitude in degrees
};

SensorMsg msg;


void setup()
{

  Serial.begin(115200);
  ss.begin(GPSBaud);
  initRadio();
}

void loop()
{
  float batVoltage=analogRead(A0)*5.0/1023;
  while (!ss.available())
  {
  }
  if (gps.encode(ss.read()))
  {
    if (gps.location.isValid())
    {
      msg.latitude = gps.location.lat();
      msg.longitude = gps.location.lng();
    }
  }

  mesh.update();

  // Send to the master node every second
  if (millis() - displayTimer >= 2000)
  {
    displayTimer = millis();
    // Build a string message (include the millis value)

    msg.deviceId = NODE_ID;
    msg.timestamp_ms = millis();
    msg.sensor_value = analogRead(A1);
    msg.battery_voltage = batVoltage;
    Serial.println(analogRead(A1));

    // Ensure the payload fits in the nRF24L01+ 32-byte limit (including null terminator)
    if (sizeof(msg) > 32)
    {
      Serial.println("Message too long to send over RF24 (max 32 bytes)");
    }
    else
    {
      // Send an 'M' type message containing the C-string (include null terminator)
      if (!mesh.write(&msg, 'M', sizeof(msg)))
      {
        // If a write fails, check connectivity to the mesh network
        if (!mesh.checkConnection())
        {
          // avoid spamming renewAddress — only attempt every 1s
          if (millis() - lastRenewAttempt > 1000UL)
          {
            lastRenewAttempt = millis();
            Serial.println("Renewing Address");
            if (mesh.renewAddress() == MESH_DEFAULT_ADDRESS)
            {
              // If address renewal fails, reconfigure the radio and restart the mesh
              // This allows recovery from most if not all radio errors
              mesh.begin();
            }
          }
          else
          {
            // backoff in effect
            Serial.println("Renew attempt deferred (backoff)");
          }
        }
        else
        {
          Serial.println("Send fail, Test OK");
        }
      }
      else
      {
        Serial.print("Send OK: ");
        Serial.println(msg.timestamp_ms);
      }
    }
  }

  while (network.available())
  {
    RF24NetworkHeader header;
    SensorMsg payload;
    network.read(header, &payload, sizeof(payload));
    Serial.print("Received packet from");
    Serial.print(payload.deviceId);
    Serial.print(" at ");
    Serial.println(payload.timestamp_ms);
  }
}
