## TTGO T-Beam Tracker for LoRaWAN Systems

Current version: 0.0.8

Uploads GPS data from the TTGO T-Beam to any LoRaWAN compatible system for tracking and determining signal strength of LoRaWAN gateways and nodes.

#### Based on the code from [kizniche/ttgo-tbeam-ttn-tracker](https://github.com/kizniche/ttgo-tbeam-ttn-tracker)

### Setup

1. Install Visual Studio Code and Platform IO.

2. Install the Arduino IDE libraries:

   * [mcci-catena/arduino-lmic](https://github.com/mcci-catena/arduino-lmic) (for Rev0 and Rev1)
   * [mikalhart/TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus) (for Rev0 and Rev1)
   * [ThingPulse/esp8266-oled-ssd1306](https://github.com/ThingPulse/esp8266-oled-ssd1306) (for Rev0 and Rev1)
   * [lewisxhe/AXP202X_Library](https://github.com/lewisxhe/AXP202X_Library) (for Rev1 only)

3. Edit ```arduino-lmic/project_config/lmic_project_config.h``` and uncomment the proper frequency for your region.

4. Edit this project file ```main/configuration.h``` and select your correct board revision, either T_BEAM_V07 or T_BEAM_V10 (see [T-BEAM Board Versions](#t-beam-board-versions) to determine which board revision you have).

5. Edit this project file ```main/credentials.h``` to use either ```USE_ABP``` or ```USE_OTAA``` and add the Keys/EUIs for your Application's Device from The Things Network.

6. Add the Decoder function to your LoRaWAN system:

```C
function Decoder(bytes, port) {
 var decoded = {};
 decoded.type = "position";
 decoded.inTrip = true;
 decoded.fixFailed = false;
 decoded.accuracy = 10;
 decoded.heading = 0.0;
 decoded.speedKmph = 0.0;
 decoded.batV = 0.0;
 decoded.manDown = null;
 decoded.latitude = ((bytes[0]<<16)>>>0) + ((bytes[1]<<8)>>>0) + bytes[2];
 decoded.latitude = (decoded.latitude / 16777215.0 * 180) - 90;
 decoded.longitude = ((bytes[3]<<16)>>>0) + ((bytes[4]<<8)>>>0) + bytes[5];
 decoded.longitude = (decoded.longitude / 16777215.0 * 360) - 180;

 var altValue = ((bytes[6]<<8)>>>0) + bytes[7];
 var sign = bytes[6] & (1 << 7);
 if(sign) decoded.altitude = 0xFFFF0000 | altValue;
 else decoded.altitude = altValue;

 decoded.hdop = bytes[8] / 10.0;
 decoded.sats = bytes[9];
 decoded.ext = bytes[10];
 return decoded;
}
```

7. Build and upload the project to your TTGO T-Beam.

8. Turn on the device and once a GPS lock is acquired, the device will start sending data to the LoraWAN system.
