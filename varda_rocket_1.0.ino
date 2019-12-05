#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <DS1307new.h>
#include <I2Cdev.h>
#include <Adafruit_MMA8451.h>
Adafruit_MMA8451 mma = Adafruit_MMA8451();
// BME_280 tanımlamaları
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 mySensor;
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI
const int buzzer = 48; // Buzzer Arduino pini
const int led = 8; // LED Arduino pini
const int egim = 12;
// RTC tanımlamaları
uint16_t startAddr = 0x0000;           
uint16_t lastAddr;                      
uint16_t TimeIsSet = 0xaa55; 
// GPS baud tanımlamaları   
static const int RXPin = 4, TXPin = 3; // GPS modülü arduino bağlantı pinleri
static const uint32_t GPSBaud = 9600; // GPS modülü haberleşme bandı
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
unsigned long delayTime;
// Telemetri tanımlamaları
float tempC; // Sıcaklık
float basinc; // Basınç
float yukseklik; // Yükseklik
float nem; // Nem
float gps_lat; // Enlem
float gps_lng; // Boylam
float gps_hiz; // GPS Hız
float gps_alt; // GPS Yükseklik
int chipSelect = 53; // SD Kart Arduino cs pini
int takim_no = 3298;
String durum;
int sayac = 0; //hız sayacı
unsigned int paket = 0; // Paket sayacı
int rakim =500; // Yükseklik referansı
int ayr = 0; // Ayrılma sayacı 
float h=0;
float a;
float b;
float hiz=0;
bool acik=false;
//motor
const int motor = 6;
// SD kart tanımlamaları
File sdkart; // SD Kart Telemetri kaydı
void setup(){
Serial.begin(9600); // Yayın yapılacak Seri baud seçimi
mma.begin();
mySensor.begin(); // BME_280 tanımlama
ss.begin(GPSBaud); // GPS bant tanımlama
SD.begin(4); // SD Kart Arduino cs pini
Wire.begin();
pinMode(buzzer, OUTPUT); // Buzzer çıkış olarak ayarlandı
pinMode(led, OUTPUT); // LED çıkış olarak ayarlandı
pinMode(motor, OUTPUT);
// RTC setup tanımlamaları
RTC.setRAM(0, (uint8_t *)&startAddr, sizeof(uint16_t));
RTC.getRAM(54, (uint8_t *)&TimeIsSet, sizeof(uint16_t));
 
  if (TimeIsSet != 0xaa55) 
  {
    RTC.stopClock();
    //RTC.fillByHMS(20,02,00); // Saat ayarı buradan yapılıyor
    RTC.setTime();
    TimeIsSet = 0xaa55;
     RTC.setRAM(54, (uint8_t *)&TimeIsSet, sizeof(uint16_t));
    RTC.startClock();
  }
  else
  {
    RTC.getTime();
  }
}
void loop() {
  // GPS 
if (ayr <=0)
{

}
RTC.getTime(); // Saat ve Tarih verilerini al
tempC = mySensor.readTemperature(); // Sıcaklık değişkeni
basinc = mySensor.readPressure(); // Basınç değişkeni (Pa.)
yukseklik = mySensor.readAltitude(SEALEVELPRESSURE_HPA); // Yükseklik değişkeni (m)
nem = mySensor.readHumidity(); // Nem değişkeni (%)
paket++; // Sayacı her defasında 1 artırır
sdkart = SD.open("5545.csv", FILE_WRITE); // Telemetri SD kart kayıt verisi
h=abs(yukseklik-rakim);

  float vin=0.0;
  float R1 = 10000;
  float R2 = 1000;
  int sensorValue = analogRead(A0); // Gerilim okuma pini
  float voltage = ((float)3.5 / 1023.0)*sensorValue*1.9; // Gerilim denklemi
  vin = voltage / (R2/(R1+R2));
if (h>=0){
  sayac=sayac+1;
  if (sayac==1){
      a=h;
    }
    if (sayac==2){
     b=h;
     hiz=abs(b-a);
      }
      if(sayac==3){
        sayac=0;
      }
}

while (ss.available() > 0 )
  gps.encode(ss.read());
    if (gps.location.isUpdated() || Serial.read() < 0){
float gps_lat = gps.location.lat(); // Enlem değişkeni
float gps_lng = gps.location.lng(); // Boylam değişkeni
float gps_hiz = gps.speed.kmph(); // GPS hız değişkeni (m/s)
float gps_alt = gps.altitude.meters(); // GPS yükseklik değişkeni (m)

Serial.print(takim_no);Serial.print("-");Serial.print(paket);Serial.print("-");Serial.print(RTC.hour, DEC);
Serial.print(":");Serial.print(RTC.minute, DEC);Serial.print(":");Serial.print(RTC.second, DEC);
Serial.print("-");Serial.print(basinc);Serial.print("-");Serial.print(h);Serial.print("-");
Serial.print(hiz);Serial.print("-");Serial.print(tempC);Serial.print("-");Serial.print(vin);
Serial.print("-");Serial.print(gps_lat,6);Serial.print("-");Serial.print(gps_lng,6);Serial.print("-");
Serial.print(gps_alt);Serial.print("-");Serial.print(nem);mma.read();Serial.print("|");Serial.print(mma.x);
Serial.print("|");Serial.print(mma.y);Serial.print("|");Serial.print(mma.z);Serial.print("|");Serial.println(durum);
delay(100);
  }

  if (hiz==0.00){
  durum = String("roket_yerde"); // Uydu yerde verisi
}
else if (b>a) {
  durum = String("roket_yukseliyor");
}
else {
  durum = String("roket_alcaliyor");
}

  // SD karta yazdırılan telemetri paketi verileri
if (sdkart) { 
sdkart.print(takim_no);sdkart.print(",");sdkart.print(paket);sdkart.print(",");
sdkart.print(RTC.hour, DEC);sdkart.print(".");sdkart.print(RTC.minute, DEC);sdkart.print(".");
sdkart.print(RTC.second, DEC);sdkart.print(",");sdkart.print(basinc);sdkart.print(",");
sdkart.print(h);sdkart.print(",");sdkart.print(hiz);sdkart.print(",");
sdkart.print(tempC);sdkart.print(",");sdkart.print(vin);sdkart.print(",");
sdkart.print(gps_lat);sdkart.print(",");sdkart.print(gps_lng);sdkart.print(",");
sdkart.print(gps_alt);sdkart.print(",");sdkart.print(nem);sdkart.print(",");
sdkart.print(mma.x);sdkart.print(",");sdkart.print(mma.y);sdkart.print(",");sdkart.print(mma.z);sdkart.println(durum);sdkart.close();
}


  //eğim verisi ile ayrılma
  if (mma.x>=200 && mma.y>=500 && mma.z>800) {
    durum = String("egim");
    digitalWrite(motor, HIGH);
    delay(3000);
  }
   if (h >=450) // Yükseklik 400 de iken otonom ayrılsın
  { 
    acik=true;
    }
    if (acik==true && h >=370 && h<=410){
      ayr++;
      digitalWrite(motor, HIGH);
      durum = String("otonom_ayrilma");
      delay(3000); // 3 sn servo çalışsın
     
      }
  else {
    digitalWrite(motor, LOW);
  }
  if (Serial.available()) // Serial porttan gelen manuel ayrılma komutunu dinle
  {  
    char ch = Serial.read();
    delay(3000);
    if (ch == 'A') // A karakteri geldiğinde manuel servoyu çalıştır
    {    
        digitalWrite(motor, HIGH);
        ayr++;
        durum = String("manual_ayrilma"); // Servo manuel ayrıldı verisi
        delay(3000);
      }
    } 
  else {   
    digitalWrite(motor, LOW);
    delay(50);
  }


 
  
  if(ayr >=1) // Ayrılma gerçekleştikten sonra yerde bilgisi gelsin
  {  
   if (h >=0 && h<=5)// Yükseklik 0 dan büyük 3 ten küçük olduğunda Buzzer ve Led çalışsın
   { 
    tone(buzzer, 1000);
    digitalWrite(led,HIGH);
    
    }
else {
  noTone(buzzer);
  digitalWrite(led,LOW);
}}

// Led yazılımı
 
 
 }
