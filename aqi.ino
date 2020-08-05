//Memory budget
//ssd1306 driver     258648 bytes
//+ wifi lib         262504 bytes
//+ sample program   271756 bytes
//+ bme680 lib       289408 bytes
//+ tsl2516 lib      289688 bytes
//+ HTTP code        296220 bytes
//+ BSEC lib         332525 bytes


//TODO
//Blink value over threshold

#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include "ssd1306.h"
#include "aqi.h"
#include "nano_gfx.h"
#include "bsec.h"


// Delay constants
const int delay1 = 20;
const int delay2 = 100;
const int delay3 = 300;
const int delay4 = 2000;


// Graphical dimensions
const int DoneOffset = 98;
const int maxX=128,maxY=63;
const int height=10;
const int margin=1;
const int startY=53; 

// Functional status
bool status = false;
bool blinkCO2 = true, blinkHumid = false, blinkVOC;
long StartEpochTime, CurrentEpochTime;
float LuxFloat;
int j=0,k;

// Type conversion variables
String TimeString,IPString,AqiString,PlaintextString,output;
char TimeChars[22],IPChars[22],AqiChars[22];
char TemperatureChars[22], PressureChars[22], HumidityChars[22];
char LuxChars[22];

// Initialize objects
WiFiUDP ntpUDP;
WiFiClient connection;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org",0, 60000);
IPAddress ip;
Adafruit_TSL2561_Unified SensorTSL = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
sensors_event_t event;
Bsec iaqSensor;

// Convert IPAddress object to string
String IpAddress2String(const IPAddress& ipAddress)
{
  return String(ipAddress[0]) + String(".") +\
  String(ipAddress[1]) + String(".") +\
  String(ipAddress[2]) + String(".") +\
  String(ipAddress[3])  ;
}

void checkIaqSensorStatus(void)
{
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      Serial.println("BSEC error code : " + String(iaqSensor.status));
      while(1);
    } else {
      Serial.println("BSEC warning code : " + String(iaqSensor.status));
    }
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      Serial.println("BME680 error code : " + String(iaqSensor.bme680Status));
      while(1);
    } else {
      Serial.println("BME680 warning code : " + String(iaqSensor.bme680Status));
    }
  }
}

// Draw graphics on display
static void drawText()
{
  int GraphRange = maxX-2*margin;
  int drawPercent;

  if(LuxFloat > 4){
      //blinky
      status = status ? false : true;

      snprintf(TemperatureChars, sizeof TemperatureChars, "Temperature: %.2f C", iaqSensor.temperature);
      snprintf(PressureChars, sizeof PressureChars, "Pressure: %.1f hPa", iaqSensor.pressure / 100.0);
      snprintf(HumidityChars, sizeof HumidityChars, "Humidity: %.1f %%", iaqSensor.humidity);
      snprintf(AqiChars, sizeof AqiChars, "AQI: %.1f    ", iaqSensor.iaq);
      snprintf(LuxChars, sizeof LuxChars, "Light: %4.0f lux ", LuxFloat);
      
      ssd1306_positiveMode();
      ssd1306_printFixed(0,  0, TimeChars, STYLE_NORMAL);
      ssd1306_printFixed(0,  8, TemperatureChars, STYLE_NORMAL);
      ssd1306_printFixed(0,  16, PressureChars, STYLE_NORMAL);
      ssd1306_printFixed(0,  24, HumidityChars, STYLE_NORMAL);
      ssd1306_printFixed(0,  32, LuxChars, STYLE_NORMAL);

      /*    
      if(blinkCO2&&status)   ssd1306_negativeMode();
      ssd1306_printFixed(0,  24, "CO2: 2200 ppm", STYLE_NORMAL);
      ssd1306_positiveMode();
        
      
      if(blinkVOC&&status)   ssd1306_negativeMode();
      ssd1306_printFixed(0,  40, "VOC's: 120 ppb", STYLE_NORMAL);
      ssd1306_positiveMode();
      */

      //Draw bar outline
      ssd1306_printFixed(0,40, AqiChars, STYLE_NORMAL);
      ssd1306_drawLine(margin,startY,maxX-margin,startY);
      ssd1306_drawLine(margin,startY+height,maxX-margin,startY+height);
      ssd1306_drawLine(margin,startY,margin,startY+height);
      ssd1306_drawLine(maxX-margin,startY,maxX-margin,startY+height);
    
      //Draw ticks
      ssd1306_drawVLine(26,startY,startY+height);
      ssd1306_drawVLine(51,startY,startY+height);
      ssd1306_drawVLine(76,startY,startY+height);
      ssd1306_drawVLine(101,startY,startY+height);
      
    
      drawPercent = iaqSensor.iaq/500*GraphRange;
      ssd1306_fillRect(margin,startY,margin+drawPercent,startY+height);
  } else {
      ssd1306_clearScreen();  
  }
}


void setup()
{
  int progress=0;
  ssd1306_setFixedFont(ssd1306xled_font6x8);
  ssd1306_128x64_i2c_init();
  ssd1306_clearScreen();
  ssd1306_printFixed(0,  0, "Starting...", STYLE_NORMAL);
  Serial.begin(115200);
  delay(100);

  //Initialize WiFi and include animation
  Serial.print("\r\n\r\nInitializing application..");
  WiFi.begin(ssid, password);
  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( delay1 );
    if(progress < 98) {
     progress = progress + 1;
     ssd1306_drawProgressBar(progress);
     Serial.print(".");
    }
  }
  ssd1306_drawProgressBar(98);
  delay ( delay2 );
  ssd1306_drawProgressBar(99);
  delay ( delay2 );
  ssd1306_drawProgressBar(100);
  delay ( delay2 );

  ssd1306_clearScreen();

  //Convert IP to string
  ssd1306_printFixed(0,  0, "DHCP client...", STYLE_NORMAL);
  ssd1306_printFixed(DoneOffset,  0, "Done!", STYLE_NORMAL);
  IPString = "IP: " + IpAddress2String(WiFi.localIP());
  strcpy(IPChars, IPString.c_str());
  ssd1306_printFixed(0,  8, IPChars, STYLE_NORMAL);

  //Initialize NTP client
  ssd1306_printFixed(0,  16, "Start ntpd...", STYLE_NORMAL);
  delay ( delay2 );
  ssd1306_printFixed(DoneOffset,  16, "Done!", STYLE_NORMAL);
  Serial.print(".");
  
  timeClient.begin();
  delay ( delay3 );
  timeClient.update();
  ssd1306_printFixed(0,  24, "Synced:", STYLE_NORMAL);
  TimeString = timeClient.getFormattedTime();
  strcpy(TimeChars, TimeString.c_str());
  ssd1306_printFixed(48,  24, TimeChars, STYLE_NORMAL);


  //Initialize BME680 sensor
  ssd1306_printFixed(0,  32, "Init BME680...", STYLE_NORMAL);
  
  iaqSensor.begin(BME680_I2C_ADDR_PRIMARY, Wire);
  checkIaqSensorStatus();

  // BME sensor values
  bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };  
  
  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);

  ssd1306_printFixed(DoneOffset,  32, "Done!", STYLE_NORMAL);
  
  //Initialize TSL2561 sensor
  ssd1306_printFixed(0,  40, "Init TSL2561...", STYLE_NORMAL);
  if(!SensorTSL.begin())
  {
    Serial.print("No TSL2561 detected. Rebooting.");
    while(1){
      ssd1306_printFixed(DoneOffset,  40, "FAIL!", STYLE_NORMAL);
    }
  }
  SensorTSL.enableAutoRange(true);           
  SensorTSL.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  
  ssd1306_printFixed(DoneOffset,  40, "Done!", STYLE_NORMAL);

  //Initialization done - start application
  ssd1306_printFixed(0,  56, "Starting AQS...", STYLE_NORMAL);
  delay(delay4);
  StartEpochTime = timeClient.getEpochTime();
  Serial.println("\r\nApplication started.");
  Serial.print("Started at: ");
  Serial.println(StartEpochTime);
  
  ssd1306_clearScreen();
}

void loop()
{
  // Update time
  timeClient.update();
  TimeString = timeClient.getFormattedTime();
  TimeString = TimeString.substring(0,5);
  TimeString = "Time: " + TimeString + "UTC";
  strcpy(TimeChars, TimeString.c_str());
    
	// Share results every 60 seconds
  if(connection.connect(HOST_NAME, HTTP_PORT)) {
    connection.println(HTTP_METHOD + " " + PATH_NAME + "/store/" + PlaintextString + " HTTP/1.1");
    Serial.println(HTTP_METHOD + " " + PATH_NAME + "/store/" + PlaintextString + " HTTP/1.1");
    connection.println("Host: " + String(HOST_NAME));
    connection.println("Connection: close");
    connection.println(); 
  } else {
    Serial.println("Connection failed");
  }

  /* // Print debug info: answer from server
  delay(100);
  while(connection.available())
  {
    // read an incoming byte from the server and print them to serial monitor:
    char c = connection.read();
    Serial.print(c);
  }
  */    

  if(!connection.connected()) connection.stop();

  for(k=0;k<60;k++){
    // Read sensors every second
    // Read TSL
    SensorTSL.getEvent(&event);
    if (event.light) LuxFloat = event.light;
  
    if (iaqSensor.run()) {
      // New data is available 
      // Convert measurements to string
      PlaintextString = "S-";
      PlaintextString = PlaintextString + String(timeClient.getEpochTime())+"-";
      PlaintextString = PlaintextString + String(iaqSensor.temperature)+"-";
      PlaintextString = PlaintextString + String(iaqSensor.pressure / 100.0)+"-";
      PlaintextString = PlaintextString + String(iaqSensor.humidity)+"-";
      PlaintextString = PlaintextString + String(iaqSensor.iaq)+"-";
      PlaintextString = PlaintextString + String(iaqSensor.staticIaq)+"-";
      PlaintextString = PlaintextString + String(iaqSensor.co2Equivalent)+"-";
      PlaintextString = PlaintextString + String(iaqSensor.breathVocEquivalent)+"-";
      PlaintextString = PlaintextString + String(LuxFloat)+"-";
      PlaintextString = PlaintextString + "E";
    } else {
      // Wait for more data
      checkIaqSensorStatus();
    }
    drawText();
    delay(1000);
  }
}
