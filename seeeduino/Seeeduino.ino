//#define USE_GPS

#include <LoRaWan.h>

#ifdef USE_GPS
#include "TinyGPS++.h"
TinyGPSPlus gps;
#endif

char buffer[256];

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

// battery of Seeeduino LoRaWAN
const int pin_battery_status  = A5;
const int pin_battery_voltage = A4;

#ifdef USE_GPS
// used for waking gps every 10 minutes
bool wakeGPS = true;
int gpsCounter = 0;
const int gpsWakeInterval = 10; // 10 minutes
bool gpsIsSleeping = false;

// gps coordinates
float lon = 0;
float lat = 0;
#endif

// fallback gps coordinates
const float fixedLon = 4.9054446;
const float fixedLat = 52.396237;

// forward declarations
#ifdef USE_GPS
void setupGPS();
#endif

void setupLora();
void float2Bytes(float val, byte* bytes_array);

void setup(void)
{
  // start up serial
  SerialUSB.begin(115200);
  while (!SerialUSB && millis() < 10000);

  SerialUSB.println("--------BEGIN_SETUP--------");

  // battery
  pinMode(pin_battery_status, INPUT);

#ifdef USE_GPS
  setupGPS();
#endif

  setupLora();

  SerialUSB.println("--------END_SETUP--------");
}

void loop(void)
{
  SerialUSB.println("--------BEGIN_LOOP--------");

  bool result = false;

#ifdef USE_GPS
  if ( wakeGPS )
  {
    setupGPS();
  }

  SerialUSB.print(F("Location : "));
  if (gps.location.isValid())
  {
    float lat = gps.location.lat();
    float lon = gps.location.lng();

    SerialUSB.print(lat, 6);
    SerialUSB.print(F(","));
    SerialUSB.print(lon, 6);

    // construct gps part of the packet
    packet[0] = 1; // gps valid

    float2Bytes(lat, floatBuff);
    packet[1] = floatBuff[0];
    packet[2] = floatBuff[1];
    packet[3] = floatBuff[2];
    packet[4] = floatBuff[3];

    float2Bytes(lon, floatBuff);
    packet[5] = floatBuff[0];
    packet[6] = floatBuff[1];
    packet[7] = floatBuff[2];
    packet[8] = floatBuff[3];

    SerialUSB.println();

    if ( !gpsIsSleeping )
    {
      SerialUSB.println("Putting gps to sleep...");
      Serial.write("$PMTK161,0*28\r\n");
      gpsIsSleeping = true;
    }

  } else
  {
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
    
    SerialUSB.print("No gps...");
    SerialUSB.println();
  }
#else
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
#endif

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

  float2Bytes((float)dbAvg, floatBuff);
  packet[9] = floatBuff[0];
  packet[10] = floatBuff[1];
  packet[11] = floatBuff[2];
  packet[12] = floatBuff[3];

  // battery
  int a = analogRead(pin_battery_voltage);
  float v = a / 1023.0 * 3.3 * 11.0;  // there's an 1M and 100k resistor divider
  float2Bytes(v, floatBuff);
  SerialUSB.print("Battery Status : ");
  SerialUSB.print(v, 2);
  SerialUSB.print('\t');
  SerialUSB.println(digitalRead(pin_battery_status));

  packet[13] = floatBuff[0];
  packet[14] = floatBuff[1];
  packet[15] = floatBuff[2];
  packet[16] = floatBuff[3];

  // send packet over lora
  lora.setPower(14);
  result = lora.transferPacket(packet, 17);
  if (result)
  {
    short length;
    short rssi;

    memset(buffer, 0, 256);
    length = lora.receivePacket(buffer, 256, &rssi);

    if (length)
    {
      SerialUSB.println("Received packet ->");
      SerialUSB.print("Length is: ");
      SerialUSB.println(length);
      SerialUSB.print("RSSI is: ");
      SerialUSB.println(rssi);
      SerialUSB.print("Data is: ");
      for (unsigned char i = 0; i < length; i ++)
      {
        SerialUSB.print("0x");
        SerialUSB.print(buffer[i], HEX);
        SerialUSB.print(" ");
      }
      SerialUSB.println();

#ifdef USE_GPS
      if ( buffer[0] == 1 )
      {
        // wake gps signal received
        SerialUSB.println("Wake GPS signal received!");
        wakeGPS = true;
        gpsCounter = 0;
      }
#endif
    }
  } else
  {
    SerialUSB.println("Some LoRa error occured... resetting");

    lora.setDeviceReset();
    setupLora();
  }

  lora.setDeviceLowPower();

#ifdef USE_GPS
  if ( gpsIsSleeping )
  {

    gpsCounter++;
    if ( gpsCounter > gpsWakeInterval )
    {
      gpsIsSleeping = false;
      SerialUSB.println("Wake GPS next loop...");
      wakeGPS = true;
      gpsCounter = 0;
    }
  }
#endif

SerialUSB.println("--------END_LOOP--------");
}

#ifdef USE_GPS
void setupGPS()
{
  bool locked;
  char c;

  Serial.begin(9600);     // open the GPS
  locked = false;

  // wake gps
  Serial.write('A');

  // give the gps some time to wake up
  delay(5000);

  long currentTime = millis();

  while (!gps.location.isValid())
  {
    while (Serial.available() > 0)
    {
      if (gps.encode(c = Serial.read()))
      {
        //displayInfo();
        if (gps.location.isValid())
        {
          locked = true;
          break;
        }
      }
      //        SerialUSB.print(c);
    }

    if (locked)
      break;

    if (millis() > currentTime + 15000 && gps.charsProcessed() < 10)
    {
      SerialUSB.println(F("No GPS detected: check wiring."));
      SerialUSB.println(gps.charsProcessed());
    }
    else if (millis() > currentTime + 20000)
    {
      SerialUSB.println(F("Not able to get a fix in alloted time."));
      break;
    }
  }

  wakeGPS = false;
}
#endif

void setupLora()
{
  // lora

  lora.init();

  memset(buffer, 0, 256);
  lora.getVersion(buffer, 256, 1);
  SerialUSB.print(buffer);

  memset(buffer, 0, 256);
  lora.getId(buffer, 256, 1);
  SerialUSB.print(buffer);

  // void setId(char *DevAddr, char *DevEUI, char *AppEUI);
  lora.setId(NULL, "00232D05310A64C2", "70B3D57ED001A7BB");
  // setKey(char *NwkSKey, char *AppSKey, char *AppKey);
  lora.setKey(NULL, NULL, "90B4A8BA4189501550E53E89D4DE84A1");

  lora.setDeciveMode(LWOTAA);
  lora.setDataRate(DR0, EU868);

  lora.setAdaptiveDataRate(true);

  lora.setChannel(0, 868.1);
  lora.setChannel(1, 868.3);
  lora.setChannel(2, 868.5);
  lora.setChannel(3, 867.1);
  lora.setChannel(4, 867.3);
  lora.setChannel(5, 867.5);
  lora.setChannel(6, 867.7);

  lora.setReceiceWindowFirst(1);
  lora.setReceiceWindowSecond(869.5, DR3);

  lora.setPower(14);
  while (!lora.setOTAAJoin(JOIN, 10))
  {
    SerialUSB.println("Joining LoRa...");
  }
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
