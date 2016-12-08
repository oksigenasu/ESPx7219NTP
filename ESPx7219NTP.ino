/*
Buana Karya Instrument Presents
NTP CLOCK WIFI ESP8266
Programmed by: Aviezab, DeGray
under Library WIFI MANAGER, ESP8266

changelog 
v0.4 1 December 2016 10:41 Zeta DeGray
-add counter 60 second no data, read from RTC in parse NTP
-repair RTC code change <RTCLib.h> to "RTCLib.h"
-separate parsing NTP and parsing Time for 7Seg format
-add software reset ESP.reset() after reset button pressed 

v0.3 31 October 2016 14:27 Zeta DeGray
-separate parsing NTP from main loop
-merging code to drive 7Seg and RTC
-change program flow to state diagram
-put reset button code inside loop (for easier reset)


v0.2 27 June 2016 08:27 Zeta DeGray
-update WiFiManager library to version 0.12
-change the way saving variable from wifi manager to EEPROM
-add password to AP mode for security "buanakarya"


v0.1 23 June 2016 08:37 Zeta DeGray
-changing variable name korewak to EEPROM_BUFF
-changing variable name korewa to NTP_ADDR
-changing delay for waiting packet reply to 6s
-changing delay for loop to 4s

*/
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager

//Library for EEPROM
#include <EEPROM.h>

//Library for RTC
#include <Wire.h>
#include "RTClib.h"
RTC_DS1307 rtc;
uint16_t years;
uint8_t months;
uint8_t days;
uint8_t hours;
uint8_t minutes;
uint8_t seconds;
uint8_t dayofTheWeek;

//Library for Max7219 7seg driver (modified for ESP8266)
// pin GPIO 13 is connected to the DataIn 
// pin GPIO 14 is connected to the CLK 
// pin GPIO 12 is connected to LOAD
#include "LedControl.h"
LedControl lc=LedControl(13,14,12,1);

//initialize Wifi Manager
WiFiManager wifim;

unsigned int localPort = 2390;      // local port to listen for UDP packets

/* Don't hardwire the IP address or we won't get the benefits of the pool.
 *  Lookup the IP address for the host name instead */
//IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
IPAddress timeServerIP; // time.nist.gov NTP server address
//char* ntpServerName = ""; //"0.id.pool.ntp.org"; //"ns1.itb.ac.id";  //"ntp.itb.ac.id"; //"time.nist.gov";
char ntpServerName[20]="0.id.pool.ntp.org";

char EEPROM_BUFF[20]="";
int alamat = 0;
int i=0;
  
String NTP_ADDR;
char static_ip[16] = "10.10.1.56";
char static_gw[16] = "10.10.1.1";
char static_sn[16] = "255.255.255.0";
char ntp_server[40];

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// flag for inet connection
int no_NTP =0;

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

char* string2char(String command){
    if(command.length()!=0){
        char *p = const_cast<char*>(command.c_str());
        return p;
    }
}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address)
{
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

//Variable for 7 Segment
int jam1;
int jam2;
int menit1;
int menit2;
int detik1;
int detik2;

//Variable for state machine
int state = 0;

void requestNTP(){
  WiFi.hostByName(ntpServerName, timeServerIP);/**/ 
  Serial.print("Server NTP: ");
  Serial.print(ntpServerName); Serial.print(" ");
  Serial.println(timeServerIP);
  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  // wait to see if a reply is available
  //delay(4000);
  
}

void parseTime(){
  if (hours >= 24){ 
    hours=hours-24;
    }
  if (hours <10) 
  {
    Serial.print('0');
    jam1 = 0;
  }else{
    jam1= (hours/10);
  }
    Serial.print(hours); // print the hour (86400 equals secs per day)
  jam2 = (hours % 10);
    
  Serial.print(':');
  
  if ( minutes < 10 ) {
      // In the first 10 minutes of each hour, we'll want a leading '0'
      Serial.print('0');
    menit1 = 0;
    }else{
    menit1 = (minutes / 10);
  }
    Serial.print(minutes); // print the minute (3600 equals secs per minute)
  menit2 = (minutes % 10);
  
    Serial.print(':');
    if ( (seconds) < 10 ) {
      // In the first 10 seconds of each minute, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.println(seconds); // print the second
  
}

int parseCount = 0;
int parseLimit =120; //120s
//untuk menghitung apakah sudah pernah di parse 60.

void parseNTP(){
  
  int cb = udp.parsePacket();
  
  if (!cb) {
    Serial.println("no packet yet");
    state =2;
    parseCount++;

    if (parseCount >= parseLimit){
      //limit reached, no packet from wifi
      state =4;
      //change to read RTC
      parseCount =0;
      //set flag NTP to 1
      no_NTP = 1;
    }
  }
  else {
    parseCount =0; //reset parse count
    no_NTP = 0;   //reset flag NTP
    state = 3;
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    //Serial.print("Seconds since Jan 1 1900 = " );
   // Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
   // Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    epoch=epoch+13; // Kalibrasi detik dari minta paket sampe waktu ditampilkan
    //Serial.println(epoch);


    // print the hour, minute and second:
    Serial.print("+");       // UTC is the time at Greenwich Meridian (GMT)
    //unsigned long hours;

    hours= ((epoch  % 86400L) / 3600)+7; // Ganti +7 dengan waktu sesuaimu (GMT+berapa)
    minutes= ((epoch % 3600) / 60);
    seconds = (epoch % 60);

    //parse from time to HH:mm:ss for 7 seg
    parseTime();
    
  }
  
}

void nyalakan(){
  lc.setDigit(0,3,menit2,false);
    lc.setDigit(0,2,menit1,false);
    lc.setDigit(0,1,jam2,false);
    lc.setDigit(0,0,jam1,false);
}

void setRTC() {
  rtc.adjust(DateTime(years,months,days,hours,minutes,seconds));
}

void readRTC() {
  DateTime now = rtc.now();
  
  hours =now.hour();
  minutes = now.minute();
  seconds = now.second();

  Serial.print("HASIL RTC:");
  parseTime();

  
//  Serial.print(now.hour(),DEC);
//  Serial.print(":");
//  Serial.print(now.minute(),DEC);  
//  Serial.print(":");
//  Serial.print(now.second(),DEC);
//  Serial.println();
  
}

void resetWifi(){
  
    for (i=0; i < 19; ++i)
      {
        EEPROM.write(i, 0);
      }
    EEPROM.commit();
    Serial.println("EEPROM Cleared");        
    wifim.resetSettings();

    //tampilkan rest di layar
    lc.setRow(0,0,B01000110); //r
    lc.setRow(0,1,B01011011); //S
    lc.setRow(0,2,B00001111); //t
    lc.setRow(0,3,B00000001); //-
}

void setup()
{
  EEPROM.begin(512);
  Serial.begin(9600);
  Serial.println();
  //Serial.println();

   /*
   The MAX72XX is in power-saving mode on startup,
   we have to do a wakeup call
   */
  lc.shutdown(0,false);
  /* Set the brightness to a medium values */
  lc.setIntensity(0,8);
  /* and clear the display */
  lc.clearDisplay(0);

  
  // if pin 2 is LOW, clear eeprom (reset all to default)
  pinMode(2, INPUT);
  
  //add parameter
  WiFiManagerParameter custom_ntp_server("ntpserver","NTP server address", ntp_server,40);

  wifim.addParameter(&custom_ntp_server);

    lc.setRow(0,0,B00010000); //i
    lc.setRow(0,1,0x15); //n
    lc.setRow(0,2,B00010000); //i
    lc.setRow(0,3,B00001111); //t
  // make an AP with password
  wifim.autoConnect("Buana Karya NTPC","buanakarya");
  lc.clearDisplay(0);
  NTP_ADDR=custom_ntp_server.getValue();
  
  for (i=0; i< 19; i++)
    {
      EEPROM_BUFF[i]=EEPROM.read(i);
    }
    Serial.print("Read: ");
    Serial.print(EEPROM_BUFF);
    
   
   if (EEPROM_BUFF[0]!= 0)
   {
       for (i=0; i< 19; i++)
    {
      ntpServerName[i]=EEPROM_BUFF[i];
    } 
   }
   else //write ntp servername to eeprom
   {
     NTP_ADDR.toCharArray(EEPROM_BUFF, 20);
    for (i=0; i < 19; ++i)
            {
              EEPROM.write(i, EEPROM_BUFF[i]);
              Serial.print(" Wrote: ");
              Serial.print(EEPROM_BUFF[i]); 
              Serial.print(" "); 
            }
            EEPROM.commit();
    for (i=0; i< 19; i++)
    {
      ntpServerName[i]=EEPROM_BUFF[i];
    }
   }
            
  
  Serial.print("WiFi connected\n");
  Serial.print("IP address: ");
  Serial.print(WiFi.localIP());

  Serial.print(" Starting UDP");
  udp.begin(localPort);
  Serial.print(" Local port: ");
  Serial.println(udp.localPort());
  
  //Initialize RTC
  rtc.begin();
  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    readRTC();
  }else{
    Serial.println("RTC is Running!");
    readRTC();
  }
  
  state =2;
  requestNTP();

  lc.setLed(0,0,0,1); //turn on Dig1's DP for ON effect
}

unsigned long prevMillis = 0; //millis utk ntp
unsigned long prevMillis2 = 0; //millis utk blink detik
const long interval = 30000;
bool ledDetik = LOW;
int resetCount =0;
int ledCount = 0;
bool ledPesan = HIGH;


void loop()
{

unsigned long currentMillis = millis();


  //request NTP tiap interval    
    if (currentMillis - prevMillis >= interval)
    {
      Serial.println("Millis!");
      prevMillis = currentMillis;
      //if (state == 2)
      {
          requestNTP();
          state = 2;
          Serial.println("state 1 req NTP");
      }
    }

  //subroutine that run once a second without interrupt
  if (currentMillis - prevMillis2 >= 1000)
    {
      prevMillis2 = currentMillis;
      if (ledDetik == LOW){
          ledDetik = HIGH;
      }
      else {
          ledDetik = LOW;
      }
      
      lc.setLed(0,1,0,ledDetik); //turn on Dig2's DP for second effect
      
      if (no_NTP){
          ledCount++;
          if (ledCount >=10){
            if (ledPesan == LOW){
              ledPesan = HIGH;
            }
            else {
              ledPesan = LOW;
            }
            ledCount =0;
      //      lc.setLed(0,0,0,ledPesan); //turn on Dig1's DP for message
          }    
      }
      
      lc.setLed(0,0,0,ledPesan); //turn on Dig1's DP for ON effect
      
      
      if (state == 2){
        //state = 3;
        parseNTP();
        Serial.println("state 2");
        
      }

      if (digitalRead(2)==LOW) {
          resetCount++;
          if (resetCount >=3){
            resetWifi();
            Serial.println("Reset!!!");
            ESP.reset();
          }
      }else{
        resetCount = 0;
      }
    }

  
  //tulis RTC
  if (state == 3){
    setRTC();
    state = 5;  
  }
  
  //baca RTC
  if (state == 4){
    readRTC();
    state = 5; 
  }
  
  
  //update Display
  if (state == 5){
    nyalakan();
    state = 2;  
    Serial.println("state 5");
  }
}


