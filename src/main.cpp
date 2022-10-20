//.ino file type

// last edit: added AT reset command and automatic mode, MQTT has some intermittent connectivity issues
// last edit: adding PMIC MQTT checks
// It seems either the MQTT or PMIC is causing hanging issues after client2.get(), that is
// if it sends a null value or invalid request.
//
// 08/11: seeemed to have fixed indefinite hang issue, it now hangs approxiamately for 1 minute if bad get request is sent
// mqtt and pmic now work in tandem with minimal connectivity problems so far
//

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
#include "secrets.h"
//#include <MQTT.h>
//#include <BQ24195.h> //Charge Timer Control Register, BQ24195 chip Power Managment IC

/*
    Variable Declaration
*/
//! int modem_reset = 0;
const char NB_PINNUMBER[] = "";
const char HTTP_HOST_NAME[] = "3d.chordsrt.com";

String URL_REQUEST;
String URL_REQUEST_ADDRESS = "/measurements/url_create?";
String INSTRUMENT_ID = "92";
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

const int Fargo_altitude = 276; // in meters
const double N = -5.257;

volatile float RAIN_COUNTER = 0.0;
volatile float WIND_COUNTER = 0.0;
const float RAIN_CALIBRATION = 0.19;
float SEA_LEVEL_PRESSURE;

const int RAIN_PIN = 4;
const int ANEMOMETER_PIN = 5;
int HTTP_PORT = 80;
unsigned long timerMillis = 0;
unsigned long lastMillis = 0;
int READ_COUNTER = 0;
int HTTP_RESPONSE_TIMEOUT;

/*
    Constructors
*/
sensors_event_t htu_humidity, htu_temp;
Adafruit_BMP3XX bmp;
Adafruit_HTU31D htu;
Adafruit_MCP9808 mcp;
Adafruit_SI1145 uv = Adafruit_SI1145();
AS5600 as5600;
NBClient nbclient;
GPRS gprs;
NB nbAccess(true);
NBScanner scannerNetworks;
NBModem modem;
// MQTTClient mqttclient(200);

/*
    Function Creation
*/

// Increments the volatile WIND_COUNTER variable
void WindInterrupt()
{
  WIND_COUNTER += 1.0;
}

// Increments the volatile RAIN_COUNTER variable
void RainInterrupt()
{
  RAIN_COUNTER += 1.0;
}

void NB_Reconnect()
{

  while (nbAccess.isAccessAlive() == 0)
  {
    MODEM.begin(true);       // force restarts the modem.
    MODEM.send("AT+COPS=0"); // forces NBClient into automatic mode
    MODEM.send("AT+CMEE=2");
    MODEM.waitForResponse();

    if ((nbAccess.begin(NB_PINNUMBER) == NB_READY) &&
        (gprs.attachGPRS() == GPRS_READY))
    {
      Serial.println("Reconnected to NBClient");
    }
    else
    {
      Serial.println("Failed to restablish connection to NBClient.");
      MODEM.waitForResponse(2000);
    }
  }
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
  URL_REQUEST = String(URL_REQUEST_ADDRESS + "instrument_id=" + INSTRUMENT_ID + "&bmp_temp=" + BMP390_TEMP + "&bmp_pressure=" + BMP390_PRESSURE + /*station level pressure---->*/ "&bmp_slp=" + String(SEA_LEVEL_PRESSURE) + "&bmp_altitude=" + BMP390_ALTITUDE + "&htu21d_temp=" + HTU31D_TEMP + "&htu21d_humidity=" + HTU31D_HUMIDITY + "&mcp9808=" + MCP9808_TEMP + "&wind_direction=" + WIND_DIRECTION + "&wind_speed=" + WIND_SPEED + "&rain=" + RAIN_AMOUNT + "&key=" + SECRET_KEY);
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

/*
  ==============================================
  BEGIN CODE BODY
  ==============================================
*/

void setup()
{
  Serial.begin(115200);
  delay(500);

  pinMode(ANEMOMETER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), WindInterrupt, RISING);

  pinMode(RAIN_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), RainInterrupt, RISING);

  analogReadResolution(12); // 12 bit ADC value

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
      delay(1000);
    }
  }

  I2CSensorInitialize();
}

void loop()
{

  Calibrate_Wind_Direction(); // Uncomment when calibrating wind vane

  lastMillis = millis();

  Serial.println(F("Made it here"));

  NB_Reconnect();

  GetI2CSensorData();
  GetWindSpeedData();
  GetRainData();
  GetWindDirectionData_D();
  CreateUrlRequest();

  // timerMillis = millis();
  HttpClient httpClient = HttpClient(nbclient, HTTP_HOST_NAME, HTTP_PORT);
  Serial.println(URL_REQUEST);
  httpClient.get(String(URL_REQUEST));
  // if(httpClient.connected())
  //{
  //     httpClient.get(String(URL_REQUEST));
  // }
  // long response_time = (millis() - (timerMillis));

  // Serial.println(*MODEM_RESPONSE_STORAGE);

  // HTTP_RESPONSE_TIMEOUT = 0;
  /*
   while (!httpClient.available())
  {
    if (HTTP_RESPONSE_TIMEOUT >= 3600)
    {
      Serial.println(F("HTTP Response Timed Out"));
      break;
    }
    else
      HTTP_RESPONSE_TIMEOUT++;

    delay(100);
  }

  while (httpClient.available())
  {
    httpClient.read();
    READ_COUNTER++;
    delay(10);
  }*/

  // int statusCode = httpClient.responseStatusCode();
  // String response = httpClient.responseBody();
  // Serial.print(F("READ COUNTER: "));
  // Serial.println(F(READ_COUNTER));
  // Serial.print(F("Response Time: "));
  // Serial.println(F(response_time));

  // READ_COUNTER = 0;
  RAIN_COUNTER = 0;
  WIND_COUNTER = 0;

  while ((millis() - lastMillis) <= 60000)
    ;

  httpClient.stop();
}