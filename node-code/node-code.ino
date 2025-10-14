#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#include <SPI.h>
#include <printf.h>
#include <stdint.h>

#include "env.h"

#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
/*
   This sample sketch demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

RF24 radio(9, 8);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

uint32_t displayTimer = 0;
// Backoff timestamp to avoid repeated renewAddress attempts
uint32_t lastRenewAttempt = 0;

#define MSG_VERSION 1
#define DEVICE_ID 12 // add/update in env.h as a numeric id

struct __attribute__((packed)) SensorMsg
{
  uint16_t deviceId;        // numeric device id
  uint32_t timestamp_ms;    // millis()
  int16_t sensor_value;     // temperature in centi-degrees (e.g. 2345 = 23.45°C)
  float battery_voltage; // battery voltage in millivolts (e.g. 3600 = 3.6V)
  float latitude;           // latitude in degrees
  float longitude;          // longitude in degrees
};

SensorMsg msg;
// Generic payload used when receiving packets from the network
// The incoming packet in this sketch expects a small payload with a millisecond
// timestamp and a counter field. Keep it packed to match network layout.
struct __attribute__((packed)) payload_t
{
  uint32_t ms;
  uint32_t counter;
};

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
          // avoid spamming renewAddress — only attempt every 10s
          if (millis() - lastRenewAttempt > 10000UL)
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
    payload_t payload;
    network.read(header, &payload, sizeof(payload));
    Serial.print("Received packet #");
    Serial.print(payload.counter);
    Serial.print(" at ");
    Serial.println(payload.ms);
  }
}
