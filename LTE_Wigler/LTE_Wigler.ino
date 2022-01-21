
#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#define SerialAT Serial1


#define USE_GSM  //! Uncomment will use SIM7000 for GSM communication

#ifdef USE_GSM
#include <CayenneMQTTGSM.h>
#else
#include <CayenneMQTTESP32.h>
#endif
#include <Arduino.h>
#include <Wire.h>
#include <TinyGsmClient.h>



//being wifi stuff for wigle
#include "WiFi.h"
int totalNetworks = 0;

#define NETWORKS_VIRTUAL_CHANNEL            6
#define LAT_VIRTUAL_CHANNEL                 7
#define LON_VIRTUAL_CHANNEL                 5
#define GPS_VIRTUAL_CHANNEL                 11


#define PIN_TX      27
#define PIN_RX      26
#define UART_BAUD   115200
#define PWR_PIN     4
#define BAT_ADC     35
#define SOLAR_ADC   36


HardwareSerial  gsmSerial(1);

TinyGsm modem(gsmSerial);
#ifdef USE_GSM
// GSM connection info.
char apn[] = "h2g2"; // Access point name. Leave empty if it is not needed.
char gprsLogin[] = ""; // GPRS username. Leave empty if it is not needed.
char gprsPassword[] = ""; // GPRS password. Leave empty if it is not needed.
char pin[] = ""; // SIM pin number. Leave empty if it is not needed.
#else
// WiFi network info.
char ssid[] = "your wifi ssid";
char wifiPassword[] = "your wifi password";
#endif

// Cayenne authentication info. This should be obtained from the Cayenne Dashboard.
char username[] = "";
char password[] = "";
char clientID[] = "";

bool bmpSensorDetected = true;



void setup()
{
    Serial.begin(UART_BAUD);
    gsmSerial.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

    pinMode(PWR_PIN, OUTPUT);

    //Launch SIM7000
    digitalWrite(PWR_PIN, HIGH);
    delay(300);
    digitalWrite(PWR_PIN, LOW);


    

    //Wait for the SIM7000 communication to be normal, and will quit when receiving any byte
    int i = 6;
    delay(200);
    while (i) {
        Serial.println("Send AT");
        gsmSerial.println("AT");
        if (gsmSerial.available()) {
            String r = gsmSerial.readString();
            Serial.println(r);
            break;
        }
        delay(1000);
        i--;
    }

#ifdef USE_GSM
    Cayenne.begin(username, password, clientID, gsmSerial, apn, gprsLogin, gprsPassword, pin);
#else
    Cayenne.begin(username, password, clientID, ssid, wifiPassword);
#endif


//more wifi wigle setup 
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
}

//adding in gps stuff
void enableGPS(void)
{
    // Set SIM7000G GPIO4 LOW ,turn on GPS power
    // CMD:AT+SGPIO=0,4,1,1
    // Only in version 20200415 is there a function to control GPS power
    modem.sendAT("+SGPIO=0,4,1,1");
    if (modem.waitResponse(10000L) != 1) {
        DBG(" SGPIO=0,4,1,1 false ");
    }
    modem.enableGPS();


}

void loop()
{
    Cayenne.loop();
    enableGPS();
    Serial.println("GPS Enabled");
}




// Default function for processing actuator commands from the Cayenne Dashboard.
// You can also use functions for specific channels, e.g CAYENNE_IN(1) for channel 1 commands.
CAYENNE_IN_DEFAULT()
{
    CAYENNE_LOG("Channel %u, value %s", request.channel, getValue.asString());
    //Process message here. If there is an error set an error message using getValue.setError(), e.g getValue.setError("Error message");
}

CAYENNE_IN(1)
{
    CAYENNE_LOG("Channel %u, value %s", request.channel, getValue.asString());
}


// This function is called at intervals to send temperature sensor data to Cayenne.

CAYENNE_OUT(NETWORKS_VIRTUAL_CHANNEL)
{
    int n = WiFi.scanNetworks();
        Serial.print("Networks = ");
        Serial.print(n);
        Serial.println(" networks currently seen");

        Cayenne.virtualWrite(NETWORKS_VIRTUAL_CHANNEL, n, "meters", UNIT_METER);

    }


float readBattery(uint8_t pin)
{
    int vref = 1100;
    uint16_t volt = analogRead(pin);
    float battery_voltage = ((float)volt / 4095.0) * 2.0 * 3.3 * (vref);
    return battery_voltage;
}


CAYENNE_OUT(LAT_VIRTUAL_CHANNEL)
{

  enableGPS();//why not?
  float lat2      = 0;
  float lon2      = 0;
  float speed2    = 0;
  float alt2      = 0;
  int   vsat2     = 0;
  int   usat2     = 0;
  float accuracy2 = 0;
  int   year2     = 0;
  int   month2    = 0;
  int   day2      = 0;
  int   hour2     = 0;
  int   min2      = 0;
  int   sec2      = 0;
  for (int8_t i = 15; i; i--) {
    Serial.printf("Requesting current GPS/GNSS/GLONASS location\n");
    if (modem.getGPS(&lat2, &lon2, &speed2, &alt2, &vsat2, &usat2, &accuracy2,
                     &year2, &month2, &day2, &hour2, &min2, &sec2)) {
      /*Serial.printf("Latitude:", String(lat2, 8), "\tLongitude:", String(lon2, 8));
      Serial.printf("Speed:", speed2, "\tAltitude:", alt2);
      Serial.printf("Visible Satellites:", vsat2, "\tUsed Satellites:", usat2);
      Serial.printf("Accuracy:", accuracy2);
      Serial.printf("Year:", year2, "\tMonth:", month2, "\tDay:", day2);
      Serial.printf("Hour:", hour2, "\tMinute:", min2, "\tSecond:", sec2);
      */break;
    } else {
      Serial.printf("Couldn't get GPS/GNSS/GLONASS location, retrying in 15s.\n");
      delay(15000L);
    }
  }
 
    Serial.printf("Lattitude : %f\n", lat2);
    Cayenne.virtualWrite(LAT_VIRTUAL_CHANNEL, lat2, TYPE_VOLTAGE, UNIT_MILLIVOLTS);

}


CAYENNE_OUT(LON_VIRTUAL_CHANNEL)
{

  enableGPS();//why not?
  float lat2      = 0;
  float lon2      = 0;
  float speed2    = 0;
  float alt2      = 0;
  int   vsat2     = 0;
  int   usat2     = 0;
  float accuracy2 = 0;
  int   year2     = 0;
  int   month2    = 0;
  int   day2      = 0;
  int   hour2     = 0;
  int   min2      = 0;
  int   sec2      = 0;
  for (int8_t i = 15; i; i--) {
    Serial.printf("Requesting current GPS/GNSS/GLONASS location\n");
    if (modem.getGPS(&lat2, &lon2, &speed2, &alt2, &vsat2, &usat2, &accuracy2,
                     &year2, &month2, &day2, &hour2, &min2, &sec2)) {
      /*Serial.printf("Latitude:", String(lat2, 8), "\tLongitude:", String(lon2, 8));
      Serial.printf("Speed:", speed2, "\tAltitude:", alt2);
      Serial.printf("Visible Satellites:", vsat2, "\tUsed Satellites:", usat2);
      Serial.printf("Accuracy:", accuracy2);
      Serial.printf("Year:", year2, "\tMonth:", month2, "\tDay:", day2);
      Serial.printf("Hour:", hour2, "\tMinute:", min2, "\tSecond:", sec2);
      */break;
    } else {
      Serial.printf("Couldn't get GPS/GNSS/GLONASS location, retrying in 15s.\n");
      delay(15000L);
    }
  }
  
    Serial.printf("Longitude : %f\n", lon2);
    Cayenne.virtualWrite(LON_VIRTUAL_CHANNEL, lon2, TYPE_VOLTAGE, UNIT_MILLIVOLTS);

}


CAYENNE_OUT(GPS_VIRTUAL_CHANNEL)
{

  enableGPS();//why not?
  float lat2      = 0;
  float lon2      = 0;
  float speed2    = 0;
  float alt2      = 0;
  int   vsat2     = 0;
  int   usat2     = 0;
  float accuracy2 = 0;
  int   year2     = 0;
  int   month2    = 0;
  int   day2      = 0;
  int   hour2     = 0;
  int   min2      = 0;
  int   sec2      = 0;
  for (int8_t i = 15; i; i--) {
    Serial.printf("Requesting current GPS/GNSS/GLONASS location\n");
    if (modem.getGPS(&lat2, &lon2, &speed2, &alt2, &vsat2, &usat2, &accuracy2,
                     &year2, &month2, &day2, &hour2, &min2, &sec2)) {
      /*Serial.printf("Latitude:", String(lat2, 8), "\tLongitude:", String(lon2, 8));
      Serial.printf("Speed:", speed2, "\tAltitude:", alt2);
      Serial.printf("Visible Satellites:", vsat2, "\tUsed Satellites:", usat2);
      Serial.printf("Accuracy:", accuracy2);
      Serial.printf("Year:", year2, "\tMonth:", month2, "\tDay:", day2);
      Serial.printf("Hour:", hour2, "\tMinute:", min2, "\tSecond:", sec2);
      */break;
    } else {
      Serial.printf("Couldn't get GPS/GNSS/GLONASS location, retrying in 15s.\n");
      delay(15000L);
    }
  }
  
   


    
  float x = lat2;
  float y = lon2;
  float z = alt2;
  char buffer[50];
  buffer[0] = '[';
  size_t offset = 1;
  dtostrf(x, 1, 3, &buffer[offset]);
  offset += strlen(&buffer[offset]);
  buffer[offset++] = ',';
  dtostrf(y, 1, 3, &buffer[offset]);
  offset += strlen(&buffer[offset]);
  buffer[offset++] = ',';
  dtostrf(z, 1, 3, &buffer[offset]);
  offset += strlen(&buffer[offset]);
  buffer[offset++] = ']';
  buffer[offset] = 0;
Serial.println(buffer);


Cayenne.virtualWrite(GPS_VIRTUAL_CHANNEL, buffer, "gps", "m");
Serial.println("GPS Chunk sent");  
}
