//.ino file type

#include <MKRNB.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_HTU31D.h>
#include <Adafruit_MCP9808.h>
#include "Adafruit_SI1145.h"
#include <AS5600.h>
#include <ArduinoHttpClient.h>
#include <Arduino_PMIC.h>
#include <WDTZero.h>
#include "secrets.h"

/*
    Variable Declaration
*/

WDTZero MyWatchDoggy; //! Define WDT

unsigned long baud = 115200;

// HTTP and GPRS connection clients
const char NB_PINNUMBER[] = "";
const char HTTP_HOST_NAME[] = "3d.chordsrt.com";
int HTTP_PORT = 80;
int HTTP_RESPONSE_TIMEOUT;
int modem_auto = 0;
int modem_reset = 0;

// Constants for Average Sea Level Pressure
//!! -------------------------- !! //
// Change Altitude to current location //
const int Fargo_altitude = 276; // in meters
const double N = -5.257;

// Rain bucket and Anemometer Pinouts and counters
volatile float RAIN_COUNTER = 0.0;
volatile float WIND_COUNTER = 0.0;
const float RAIN_CALIBRATION = 0.19;
const int RAIN_PIN = 4;
const int ANEMOMETER_PIN = 5;
float SEA_LEVEL_PRESSURE;

// Timers to control loop duration
unsigned long timerMillis = 0;
unsigned long lastMillis = 0;
int READ_COUNTER = 0;

//! values assigned to strings to be sent in GET request (ID)
String URL_REQUEST;
String URL_REQUEST_ADDRESS = "/measurements/url_create?";
String INSTRUMENT_ID = "";
String BMP390_TEMP;
String BMP390_PRESSURE;
String BMP390_ALTITUDE;
String HTU31D_TEMP;
String HTU31D_HUMIDITY;
String MCP9808_TEMP;
String WIND_DIRECTION;
String WIND_SPEED;
String RAIN_AMOUNT;
String HTTP_RESPONSE;
String HTTP_STATUS_CODE;
String *MODEM_RESPONSE_STORAGE;

/*
    Constructors
*/

sensors_event_t htu_humidity, htu_temp;
Adafruit_BMP3XX bmp;                    // pressure
Adafruit_HTU31D htu;                    // humidity
Adafruit_MCP9808 mcp;                   // temperature
Adafruit_SI1145 uv = Adafruit_SI1145(); // ir
AS5600 as5600;                          // windvane
NBClient nbclient;                      // narrowband client
NB nbAccess(true);                      // see AT command serial printout
NBScanner scannerNetworks;
NBModem modem;
GPRS gprs; // general packet radio service

/*
    Function Creation
*/

// Increments the volatile WIND_COUNTER variable
// Anemometer (pin 5)

void WindInterrupt()
{
  WIND_COUNTER += 1.0;
}

// Increments the volatile RAIN_COUNTER variable
// rain bucket (pin 6)
void RainInterrupt()
{
  RAIN_COUNTER += 1.0;
}

void NB_Reconnect()
{

  while (nbAccess.isAccessAlive() == 0)
  {

    if ((nbAccess.begin(NB_PINNUMBER) == NB_READY) &&
        (gprs.attachGPRS() == GPRS_READY))
    {
      Serial.println("Reconnected to NBClient");
    }
    else
    {
      Serial.println("Failed to restablish connection to NBClient.");
      MODEM.sendf("AT+CEREG=1");
      MODEM.waitForResponse(2000); 
      MODEM.sendf("AT+CGACT=1");
      MODEM.waitForResponse(2000);
      modem_reset++;
      Serial.println(modem_reset);
      if (modem_reset > 10)
      {
        modem_reset = 0;
        MODEM.sendf("AT+CFUN=1,1");
        MODEM.waitForResponse(2000);
      }
    }
  }
}

extern "C" char *sbrk(int incr);

int freeRam()
{
  char top;
  return &top - reinterpret_cast<char *>(sbrk(0));
}

void display_freeram()
{
  Serial.print(F("- SRAM left: "));
  Serial.println(freeRam());
}

void I2CSensorInitialize()
{

  Wire.begin();

  if (!bmp.begin_I2C())
  {
    Serial.println("***BMP390 Not Detected***");
  }

  if (!htu.begin())
  {
    Serial.println("***HTU31D Not Detected***");
  }

  if (!mcp.begin(0x18))
  {
    Serial.println("***MCP9808 Not Detected***");
  }

  if (!uv.begin())
  {
    Serial.println("***SI1145 Not Detected***");
  }
}

void Calibrate_Wind_Direction()
{
  for (int x = 1; x < 600; x++)
  {
    Serial.println(float(analogRead(A0) * 0.08789));
    delay(1000);
  }
}

//! Analog
void GetWindDirectionData_A()
{
  WIND_DIRECTION = String(float(analogRead(A0) * 0.08789));
}

//! Digital
void GetWindDirectionData_D()
{
  WIND_DIRECTION = String(float(as5600.readAngle() * AS5600_RAW_TO_DEGREES));
}

//! ------------- !//
// IR SENSOR //
void GetIRData()
{
  /*"&si1145_vis" + String(uv.readVisible()) + "&si1145_ir" + String(uv.readIR()) + "&si1145_uv" + String((uv.readUV() / 100)) +*/
}

void GetWindSpeedData()
{
  WIND_SPEED = String(float(0.019 * (WIND_COUNTER / 2) + 0.36));
}

void GetRainData()
{
  RAIN_AMOUNT = String(float(RAIN_COUNTER * RAIN_CALIBRATION));
}

void CreateUrlRequest()
{
  URL_REQUEST = String(URL_REQUEST_ADDRESS + "instrument_id=" + INSTRUMENT_ID + "&bmp_temp=" + BMP390_TEMP + "&bmp_pressure=" + BMP390_PRESSURE + /*station level pressure---->*/ "&bmp_slp=" + String(SEA_LEVEL_PRESSURE) + "&bmp_altitude=" + BMP390_ALTITUDE + "&htu21d_temp=" + HTU31D_TEMP + "&htu21d_humidity=" + HTU31D_HUMIDITY + "&mcp9808=" + MCP9808_TEMP + "&wind_direction=" + WIND_DIRECTION + "&wind_speed=" + WIND_SPEED + "&rain=" + RAIN_AMOUNT + "&key=" + SECRET_KEY + "&test");
}

void GetI2CSensorData()
{
  BMP390_TEMP = String(float(bmp.readTemperature()));                                                                                                       // in C
  BMP390_PRESSURE = String(float(bmp.readPressure() / 100));                                                                                                // in hPa's
  SEA_LEVEL_PRESSURE = (bmp.readPressure() / 100) * pow(1 - ((0.0065 * Fargo_altitude) / ((bmp.readTemperature()) + 0.0065 * Fargo_altitude + 273.15)), N); // used https://keisan.casio.com/exec/system/1224575267 as reference
  BMP390_ALTITUDE = String(float(bmp.readAltitude(SEA_LEVEL_PRESSURE)));

  htu.getEvent(&htu_humidity, &htu_temp);
  HTU31D_HUMIDITY = String(float(htu_humidity.relative_humidity));
  HTU31D_TEMP = String(float(htu_temp.temperature));

  MCP9808_TEMP = String(float(mcp.readTempC()));
}

void myshutdown() //! 01.10
{

  Serial.print("\nMKRNB1500 restarting! ...");
}

/*
  ==============================================
  BEGIN CODE BODY
  ==============================================
*/

void setup()
{
  Serial.begin(baud); // baud rate affect connectivity?
  // The firmware that Arduino.cc is selling on the modems on this device is 3 years old and this is apparently a known bug with that firmware. Upgrading it requires that you solder a USB cable to pads on the board and getting the FW from UBlox.
  delay(20000); // Modem has wakeup time

  pinMode(ANEMOMETER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), WindInterrupt, RISING);

  pinMode(RAIN_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), RainInterrupt, RISING);

  analogReadResolution(12); // 12 bit ADC value

  Serial.print("\nWDTZero : Setup Soft Watchdog at 2 minute interval");
  MyWatchDoggy.attachShutdown(myshutdown);
  MyWatchDoggy.setup(WDT_SOFTCYCLE2M); // initialize WDT-softcounter refesh cycle on 2 minute interval

  boolean connected = false;

  while (!connected)
  {
    if ((nbAccess.begin(NB_PINNUMBER) == NB_READY) &&
        (gprs.attachGPRS() == GPRS_READY))
    {
      connected = true;
      Serial.println("Connected to NBClient!");
    }
    else
    {
      Serial.println("Failed to connect to NBClient, trying again.");
      delay(3000);
    }
  }

  I2CSensorInitialize();
}

void loop()
{

  // Calibrate_Wind_Direction(); // Uncomment when calibrating wind vane

  lastMillis = millis();
  MyWatchDoggy.clear(); //! 01.10
  Serial.println(F("Beginning of Loop"));

  NB_Reconnect();

  GetI2CSensorData();
  GetWindSpeedData();
  GetRainData();
  GetWindDirectionData_A(); //! specify analog or digital before operation !//
  CreateUrlRequest();

  // timerMillis = millis();
  HttpClient httpClient = HttpClient(nbclient, HTTP_HOST_NAME, HTTP_PORT);
  Serial.println(URL_REQUEST);
  httpClient.get(String(URL_REQUEST));

  RAIN_COUNTER = 0;
  WIND_COUNTER = 0;
  httpClient.flush(); // graceful socket disconnect
  while ((millis() - lastMillis) <= 50000)
    ;

  httpClient.stop();
  delay(10000); //
  // AT serial socket still transitions intermittently to #1 but returns to #0 shortly afterwards
}
