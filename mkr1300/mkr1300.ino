/*
  Lora Send And Receive
  This sketch demonstrates how to send and receive data with the MKR WAN 1300 LoRa module.
  This example code is in the public domain.
*/

#include <MKRWAN.h>

LoRaModem modem;

// Uncomment if using the Murata chip as a module
// LoRaModem modem(Serial1);

#include "arduino_secrets.h"
// Please enter your sensitive data in the Secret tab or arduino_secrets.h
String appEui = SECRET_APP_EUI;
String appKey = SECRET_APP_KEY;

byte packet[17] =
{
  // GPS VALID = 0 GPS invalid = 1
  0x00,
  // lat float value in 4 bytes
  0x00,
  0x00,
  0x00,
  0x00,
  // lon float value in 4 bytes
  0x00,
  0x00,
  0x00,
  0x00,
  // decibel float value in 4 bytes
  0x00,
  0x00,
  0x00,
  0x00,
  // voltage float in 4 bytes
  0x00,
  0x00,
  0x00,
  0x00
};

// decibel
float db = 60.0f;

// used for float2bytes
byte* floatBuff = new byte[4];

// fallback gps coordinates
const float fixedLon = 4.9054446;
const float fixedLat = 52.396237;

// forward declarations
void float2Bytes(float val, byte* bytes_array);
void setupLora();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial && millis() < 10000);

  setupLora();
}

void setupLora()
{
    // change this to your regional band (eg. US915, AS923, ...)
  if (!modem.begin(EU868)) {
    Serial.println("Failed to start module");
    while (1) {}
  };
  Serial.print("Your module version is: ");
  Serial.println(modem.version());
  Serial.print("Your device EUI is: ");
  Serial.println(modem.deviceEUI());

  int connected = modem.joinOTAA(appEui, appKey);
  if (!connected) {
    Serial.println("Something went wrong; are you indoor? Move near a window and retry");
    while (!connected) {
      modem.joinOTAA(appEui, appKey);
    }
  }

  // Set poll interval to 60 secs.
  modem.minPollInterval(60);
}

void loop() {

  // construct gps part of the packet
  packet[0] = 0; // gps valid

  float2Bytes(fixedLat, floatBuff);
  packet[1] = floatBuff[0];
  packet[2] = floatBuff[1];
  packet[3] = floatBuff[2];
  packet[4] = floatBuff[3];

  float2Bytes(fixedLon, floatBuff);
  packet[5] = floatBuff[0];
  packet[6] = floatBuff[1];
  packet[7] = floatBuff[2];
  packet[8] = floatBuff[3];

  // measure db
  int counter = 0;
  long prevTime = millis();
  double dbAvg = 0.0;
  float dbPeak = 0.0;
  int time = 0;
  while ( time < 60000 )
  {
    int deltaTime = millis() - prevTime;
    prevTime += deltaTime;
    time += deltaTime;
    counter++;

    db = (analogRead(0) + 83.2073) / 11.003;
    dbAvg += db;

    SerialUSB.print("Decibel : ");
    SerialUSB.print(db);
    SerialUSB.println();

    delay(125);
  }

  dbAvg /= counter;

  // put db into the packet
  float2Bytes((float)dbAvg, floatBuff);
  packet[9] = floatBuff[0];
  packet[10] = floatBuff[1];
  packet[11] = floatBuff[2];
  packet[12] = floatBuff[3];

  // no voltage at this time...

  float v = analogRead(ADC_BATTERY);
  float2Bytes(v, floatBuff);
  packet[13] = floatBuff[0];
  packet[14] = floatBuff[1];
  packet[15] = floatBuff[2];
  packet[16] = floatBuff[3];

  // send the packet
  int err;
  modem.beginPacket();
  for(int i = 0 ; i < 17; i++)
  {
    modem.print((char)packet[i]);
  }
  err = modem.endPacket(true);
  if (err > 0) {
    Serial.println("Message sent correctly!");
  } else {
    Serial.println("Error sending message :(");
    Serial.println("Restarting lorawan module...");
    modem.restart();
    setupLora();
    return;
  }
  delay(1000);

  // receive message
  if (!modem.available()) {
    Serial.println("No downlink message received at this time.");
    return;
  }

  // read the package
  char rcv[64];
  int i = 0;
  while (modem.available()) {
    rcv[i++] = (char)modem.read();
  }
  Serial.print("Received: ");
  for (unsigned int j = 0; j < i; j++) {
    Serial.print(rcv[j] >> 4, HEX);
    Serial.print(rcv[j] & 0xF, HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void float2Bytes(float val, byte* bytes_array) {
  // Create union of shared memory space
  union {
    float float_variable;
    byte temp_array[4];
  } u;
  // Overite bytes of union with float variable
  u.float_variable = val;
  // Assign bytes to input array
  memcpy(bytes_array, u.temp_array, 4);
}
