#include <DFRobot_PH.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DFRobot_EC.h"
#include <SPI.h>
//#include <Adafruit_GFX.h> //圖形程式庫
//#include <Adafruit_SSD1306.h>
//#define OLED_RESET 0  // GPIO0
//Adafruit_SSD1306 display(OLED_RESET);
#include <WiFi.h>
#include <HTTPClient.h>
char ssid[] = "Dog_ice";//"LAPTOP-PRV61I67 1706";
char password[] = "0988898306";
//請修改為你自己的API Key，並將https改為http
String url = "http://api.thingspeak.com/update?api_key=DGK9QLCKLZ8X17HA";
#define pH_Pin 36
#define Temperature_Wire 32
#define EC_Pin 39
#define DO_PIN 34
#define VREF 5000    //VREF (mv)
#define ADC_RES 1024 //ADC Resolution
//Single-point calibration Mode=0
//Two-point calibration Mode=1
#define TWO_POINT_CALIBRATION 0
#define READ_TEMP (25) //Current water temperature ℃, Or temperature sensor function
//Single point calibration needs to be filled CAL1_V and CAL1_T
#define CAL1_V (1600) //mv
#define CAL1_T (25)   //℃
//Two-point calibration needs to be filled CAL2_V and CAL2_T
//CAL1 High temperature point, CAL2 Low temperature point
#define CAL2_V (1300) //mv
#define CAL2_T (15)   //℃
OneWire oneWire(Temperature_Wire);  
DallasTemperature sensors(&oneWire);
float pHVoltage, voltage, EC_Voltage,EC_Value,DOValue, Temperature = 25;
int sensorValue = 0;
unsigned long int avgValue;
float b;
int buf[10],temp;
//oneWire(Temperature_Wire);  
//OneWire //oneWire(Temperature_Wire); 
//DallasTemperature sensors(&oneWire);
DFRobot_PH ph;
DFRobot_EC ec;
const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};

uint16_t ADC_Raw;
uint16_t ADC_Voltage;
uint16_t DO;

int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c)
{
#if TWO_POINT_CALIBRATION == 0
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
  uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}
void setup()
{
 Serial.begin(115200);
  Serial.print("開始連線到無線網路SSID:");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("連線完成");
   sensors.begin();
  ph.begin();
  ec.begin();

}
void loop()
{
  for(int i=0;i<10;i++)
 {
  buf[i]=analogRead(pH_Pin);
  delay(10);
 }
 for(int i=0;i<9;i++)
 {
  for(int j=i+1;j<10;j++)
  {
   if(buf[i]>buf[j])
   {
    temp=buf[i];
    buf[i]=buf[j];
    buf[j]=temp;
   }
  }
 }
 avgValue=0;
 for(int i=2;i<8;i++)
 avgValue+=buf[i];
 float pHVol=(float)avgValue*5.0/1024/6;
 float pHValue = -0.96 * pHVol + 21.34;// -5.70 * pHVol + 21.34;
  static unsigned long timepoint = millis();
  if(millis()-timepoint>1000U){ //time interval: 1s
     sensors.requestTemperatures();
    timepoint = millis();
    Temperature = sensors.getTempCByIndex(0); // read your temperature sensor to execute temperature compensation
    voltage = analogRead(pH_Pin)/1024.0*5000; // read the voltage
    ph.calibration(pHVoltage,Temperature); // calibration process by Serail CMD
    EC_Voltage = analogRead(EC_Pin)/1024.0*5000/6.5;   // read the voltage
    //temperature = readTemperature();          // read your temperature sensor to execute temperature compensation
    //EC_Value =  ec.readEC(EC_Voltage,Temperature);  // convert voltage to EC with temperature compensation
    EC_Value=EC_Voltage/164;
    ADC_Raw = analogRead(DO_PIN);
    ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;
    DOValue=readDO(ADC_Voltage, Temperature);
  } 
  //讀取成功，將溫濕度顯示在序列視窗
  Serial.print("感測器讀取成功: ");
  Serial.print((float)Temperature); Serial.print(" *C, ");
      Serial.print((float)pHValue); Serial.println(" pH");
  Serial.print((float)EC_Value); Serial.println(" ms/cs");
  Serial.print((float)EC_Voltage); Serial.println(" EC_Voltage");
  Serial.print((float)DOValue); Serial.println(" ug/L");
  //開始傳送到thingspeak
  Serial.println("啟動網頁連線");
  HTTPClient http;
  //將溫度及濕度以http get參數方式補入網址後方
  String url1 = url + "&field1=" + (float)Temperature + "&field2=" + (float)pHValue+ "&field3=" + (float)EC_Value+ "&field4=" + (float)DOValue;
  //http client取得網頁內容
  http.begin(url1);
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK)      {
    //讀取網頁內容到payload
    String payload = http.getString();
    //將內容顯示出來
    Serial.print("網頁內容=");
    Serial.println(payload);
  } else {
    //讀取失敗
    Serial.println("網路傳送失敗");
  }
  http.end();
  delay(20000);//休息20秒
  

}
