/*

  GPS module

  Copyright (C) 2018 by Xose Pérez <xose dot perez at gmail dot com>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <TinyGPS++.h>
#if defined(T_BEAM_V10)
#include <Wire.h>
#include <axp20x.h>
#define I2C_SDA         21
#define I2C_SCL         22
#endif

#include <SparkFun_Ublox_Arduino_Library.h> //http://librarymanager/All#SparkFun_Ublox_GPS

uint32_t LatitudeBinary;
uint32_t LongitudeBinary;
uint16_t altitudeGps;
uint8_t hdopGps;
uint8_t sats;


char t[32]; // used to sprintf for Serial output

TinyGPSPlus* _gps;
HardwareSerial _serial_gps(GPS_SERIAL_NUM);

void gps_time(char * buffer, uint8_t size) {
    snprintf(buffer, size, "%02d:%02d:%02d", _gps->time.hour(), _gps->time.minute(), _gps->time.second());
}

float gps_latitude() {
    // just return fake
    //return 38.946911;
    return _gps->location.lat();
}

float gps_longitude() {
    //return -77.093239;
    return _gps->location.lng();
}

float gps_altitude() {
    //return 101;
    return _gps->altitude.meters();
}

float gps_hdop() {
    //return 1.0;
    return _gps->hdop.hdop();
}

uint8_t gps_sats() {
    //return 6;
    return _gps->satellites.value();
}

void gps_setup() {
    resetgps();
    _gps = new TinyGPSPlus();
}

static void gps_loop() {

    int avail = _serial_gps.available();
    //sprintf(t, "%s", "GPS BEGIN");
    //Serial.println(t);
    while (avail) {
        int r = _serial_gps.read();
        //sprintf(t, "%c", r);
        //Serial.print(t);
        _gps->encode(r);
        avail = _serial_gps.available();
    }
   // sprintf(t, "%s", "GPS END");
    //Serial.println(t);
    //screen_print("[GPS] not available\n");
}

#if defined(PAYLOAD_USE_FULL)

    // More data than PAYLOAD_USE_CAYENNE
    void buildPacket(uint8_t txBuffer[PAYLOAD_LENGTH], bool isfailure)
    {
        LatitudeBinary = ((_gps->location.lat() + 90) / 180.0) * 16777215;
        LongitudeBinary = ((_gps->location.lng() + 180) / 360.0) * 16777215;
        altitudeGps = _gps->altitude.meters();
        hdopGps = _gps->hdop.value() / 10;
        sats = _gps->satellites.value();

        sprintf(t, "Lat: %f", _gps->location.lat());
        Serial.println(t);
        sprintf(t, "Lng: %f", _gps->location.lng());
        Serial.println(t);
        sprintf(t, "Alt: %d", altitudeGps);
        Serial.println(t);
        sprintf(t, "Hdop: %d", hdopGps);
        Serial.println(t);
        sprintf(t, "Sats: %d", sats);
        Serial.println(t);

        txBuffer[0] = ( LatitudeBinary >> 16 ) & 0xFF;
        txBuffer[1] = ( LatitudeBinary >> 8 ) & 0xFF;
        txBuffer[2] = LatitudeBinary & 0xFF;
        txBuffer[3] = ( LongitudeBinary >> 16 ) & 0xFF;
        txBuffer[4] = ( LongitudeBinary >> 8 ) & 0xFF;
        txBuffer[5] = LongitudeBinary & 0xFF;
        txBuffer[6] = ( altitudeGps >> 8 ) & 0xFF;
        txBuffer[7] = altitudeGps & 0xFF;
        txBuffer[8] = hdopGps & 0xFF;
        txBuffer[9] = sats & 0xFF;

        if (isfailure == true) {
            txBuffer[10] = 0x01;          
        }
    }

#elif defined(PAYLOAD_USE_CAYENNE)

    // CAYENNE DF
    void buildPacket(uint8_t txBuffer[PAYLOAD_LENGTH], bool isfailure)
    {
        sprintf(t, "Lat: %f", _gps->location.lat());
        Serial.println(t);
        sprintf(t, "Lng: %f", _gps->location.lng());
        Serial.println(t);        
        sprintf(t, "Alt: %f", _gps->altitude.meters());
        Serial.println(t);        
        int32_t lat = _gps->location.lat() * 10000;
        int32_t lon = _gps->location.lng() * 10000;
        int32_t alt = _gps->altitude.meters() * 100;
        
        txBuffer[2] = lat >> 16;
        txBuffer[3] = lat >> 8;
        txBuffer[4] = lat;
        txBuffer[5] = lon >> 16;
        txBuffer[6] = lon >> 8;
        txBuffer[7] = lon;
        txBuffer[8] = alt >> 16;
        txBuffer[9] = alt >> 8;
        txBuffer[10] = alt;

        if (isfailure == true) {
            txBuffer[11] = 0x01;          
        }
    }

#endif


void resetgps()
{
  SFE_UBLOX_GPS myGPS;
  #if defined(T_BEAM_V10)  
  Wire.begin(I2C_SDA, I2C_SCL);
    if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
        Serial.println("AXP192 Begin PASS");
    } else {
        Serial.println("AXP192 Begin FAIL");
    }
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON); // GPS main power
    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON); // provides power to GPS backup battery
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON); // enables power to ESP32 on T-beam
    axp.setPowerOutPut(AXP192_DCDC3, AXP202_ON); // I foresee similar benefit for restting T-watch 
                                                 // where ESP32 is on DCDC3 but remember to change I2C pins and GPS pins!
  #endif 
   _serial_gps.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("All comms started");
  delay(100);

  do {
    if (myGPS.begin(_serial_gps)) {
      Serial.println("Connected to GPS");
      myGPS.setUART1Output(COM_TYPE_NMEA); //Set the UART port to output NMEA only
      myGPS.saveConfiguration(); //Save the current settings to flash and BBR
      Serial.println("GPS serial connected, output set to NMEA");
      myGPS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1);
      myGPS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
      myGPS.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
      myGPS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);
      myGPS.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);
      myGPS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);
      myGPS.saveConfiguration(); //Save the current settings to flash and BBR
      Serial.println("Enabled/disabled NMEA sentences");
      break;
    }
    delay(1000);
    Serial.println("Waiting for GPS");
    if (_serial_gps.available()) {
        Serial.write(_serial_gps.read());  // print anything comes in from the GPS
    }
  } while(1);

} 