//Memory budget
//ssd1306 driver     258648 bytes
//+ wifi lib         262504 bytes
//+ sample program   271756 bytes
//+ bme680 lib       289408 bytes
//+ tsl2516 lib      289688 bytes
//+ HTTP code        296220 bytes


//TODO
//Blink value over threshold
//Implement BSEC library

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
#include "Adafruit_BME680.h"


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
const int startY=53; //63-10-2=51

// Functional status
bool status = false;
bool blinkCO2 = true, blinkHumid = false, blinkVOC;
long StartEpochTime, CurrentEpochTime;
float LuxFloat;
int j=0,k;



// Type conversion variables
String TimeString,IPString,AqiString,PlaintextString;
char TimeChars[22],IPChars[22],AqiChars[22];
char TemperatureChars[22], PressureChars[22], HumidityChars[22];
char LuxChars[22];

// Initialize objects
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org",0, 60000);
WiFiClient connection;
Adafruit_BME680 SensorBME; // I2C
IPAddress ip;
Adafruit_TSL2561_Unified SensorTSL = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
sensors_event_t event;

// Convert IPAddress object to string
String IpAddress2String(const IPAddress& ipAddress)
{
  return String(ipAddress[0]) + String(".") +\
  String(ipAddress[1]) + String(".") +\
  String(ipAddress[2]) + String(".") +\
  String(ipAddress[3])  ;
}

// Draw graphics on display
static void drawText()
{
  int GraphRange = maxX-2*margin;
  float aqi = 12.3;
  int drawPercent;

  if(LuxFloat > 4){
      //blinky
      status = status ? false : true;

      snprintf(TemperatureChars, sizeof TemperatureChars, "Temperature: %.2f C", SensorBME.temperature);
      snprintf(PressureChars, sizeof PressureChars, "Pressure: %.1f hPa", SensorBME.pressure / 100.0);
      snprintf(HumidityChars, sizeof HumidityChars, "Humidity: %.1f %%", SensorBME.humidity);
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
    
      //Air quality part:
      AqiString = "AQI: ";
      AqiString = AqiString + aqi;
      strcpy(AqiChars, AqiString.c_str());
    
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
      
    
      drawPercent = aqi/500*GraphRange;
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
  if (!SensorBME.begin()) {
    Serial.println("No BME680 detected. Rebooting.");
    ssd1306_printFixed(DoneOffset,  32, "FAIL!", STYLE_NORMAL);
  }

  SensorBME.setTemperatureOversampling(BME680_OS_8X);
  SensorBME.setHumidityOversampling(BME680_OS_2X);
  SensorBME.setPressureOversampling(BME680_OS_4X);
  SensorBME.setIIRFilterSize(BME680_FILTER_SIZE_3);
  SensorBME.setGasHeater(320, 150); // 320*C for 150 ms
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
  TimeString = "Time: " + TimeString + "     UTC+2";
  strcpy(TimeChars, TimeString.c_str());

  // Read BME
  if (! SensorBME.performReading()) {
    Serial.println("Reading BME680 failed.");
    return;
  }

  // Read TSL
  SensorTSL.getEvent(&event);
  if (event.light) LuxFloat = event.light;


  //Convert measurements to string
  PlaintextString = "S-";
  PlaintextString = PlaintextString + String(timeClient.getEpochTime())+"-";
  PlaintextString = PlaintextString + String(SensorBME.temperature)+"-";
  PlaintextString = PlaintextString + String(SensorBME.pressure / 100.0)+"-";
  PlaintextString = PlaintextString + String(SensorBME.humidity)+"-";
  PlaintextString = PlaintextString + String(LuxFloat)+"-";
  PlaintextString = PlaintextString + "E";

    
	// Share results
  if(connection.connect(HOST_NAME, HTTP_PORT)) {
    connection.println(HTTP_METHOD + " " + PATH_NAME + "/store/" + PlaintextString + " HTTP/1.1");
    Serial.println(HTTP_METHOD + " " + PATH_NAME + "/store/" + PlaintextString + " HTTP/1.1");
    connection.println("Host: " + String(HOST_NAME));
    connection.println("Connection: close");
    connection.println(); 
  } else {
    Serial.println("Connection failed");
  }
    
      /*
      delay(100);
      while(connection.available())
      {
        // read an incoming byte from the server and print them to serial monitor:
        char c = connection.read();
        Serial.print(c);
      }
      */    
  if(!connection.connected()) connection.stop();


  drawText();
  
  for(k=0;k<60;k++){
    delay(1000);
  }
}
