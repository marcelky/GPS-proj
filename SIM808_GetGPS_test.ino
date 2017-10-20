/*
### Get GPS data
1. This example is used to test SIM808 GPS/GPRS/GSM Shield's reading GPS data.
2. Open the SIM808_GetGPS example or copy these code to your project
3. Download and dial the function switch to Arduino
4. open serial helper
4. Place it outside, waiting for a few minutes and then it will send GPS data to serial

create on 2016/09/23, version: 1.0
by jason

*/
#include <DFRobot_sim808.h>
#include <sim808.h>
#include <SoftwareSerial.h>
#include <SimpleTimer.h>

//#define PIN_TX    10
//#define PIN_RX    11
//SoftwareSerial mySerial(PIN_TX,PIN_RX);
//DFRobot_SIM808 sim808(&mySerial);//Connect RX,TX,PWR,

#define rxPin 2
#define txPin 3
SoftwareSerial GPRS(rxPin, txPin); // RX, TX
DFRobot_SIM808 sim808(&GPRS);

int Powerkey=6;                 //Power pin for SIM808
SimpleTimer timer;

void setup() {
  GPRS.begin(9600);
  Serial.begin(9600);

  GPRS.println("AT+CPOWD=1"); 
  delay(5000);
  pinMode(Powerkey, OUTPUT);   // initialize the digital pin as an output.  
  power();                     // power on the sim808 or power down the sim808  

  Serial.println("reseting");
  delay(10000);

    //GNSS power control
  GPRS.println("AT+CGNSPWR=1");
  delay(290);
  //GNSS navitation, GEO-Fence and speed alarm URC report control
  GPRS.println("AT+CGNSURC=0");
  delay(300);




  //******** Initialize sim808 module *************
  while(!sim808.init()) { 
      delay(1000);
      Serial.print("Sim808 init error\r\n");
  }

  //sim808_init(&GPRS,9600);

  if( sim808.attachGPS())
      Serial.println("Open the GPS power success");
  else 
      Serial.println("Open the GPS power failure");


  delay(5000);
  if(!sim808_check_with_cmd("AT+CGPSINF=2\r\n","OK\r\n",CMD)){  
    Serial.println("AT+CGPSINF => FAIL");    
  }else Serial.println("AT+CGPSINF => OK");
  //************* Turn on the GPS power************
  


  GPRS.println("AT+CGPSRST=1");  //1 = HOT start of GPS or COLD start = 0
  //SerialSim808_Read();
  delay(50);
      
  timer.setInterval(60000, GetGPSLocation);
}

void loop() {
   timer.run();
   /*if(GPRS.available())
      Serial.write(GPRS.read());
   if(Serial.available())
      GPRS.write(Serial.read());
   */     
}

/****************************************************************************************
 * power()
 * Function power on or power of SIM808.
 ****************************************************************************************/
void power(void)
{
  digitalWrite(Powerkey, LOW); 
  delay(1000);               // wait for 1 second
  digitalWrite(Powerkey, HIGH);
}

void GetGPSLocation(){
     //************** Get GPS data *******************
  if( sim808.attachGPS())
      Serial.println("Open the GPS power success");
  else 
      Serial.println("Open the GPS power failure");
   
   delay(20000);
   
   sim808_flush_serial();  
   //GPRS.println("AT+CGPSINF=2");
   if(!sim808_check_with_cmd("AT+CGPSINF=2\r\n","OK\r\n",CMD)){  
     Serial.println("AT+CGPSINF => FAIL");    
   }else Serial.println("AT+CGPSINF => OK");
   
   if (sim808.getGPS()) {
    Serial.print(sim808.GPSdata.year);
    Serial.print("/");
    Serial.print(sim808.GPSdata.month);
    Serial.print("/");
    Serial.print(sim808.GPSdata.day);
    Serial.print(" ");
    Serial.print(sim808.GPSdata.hour);
    Serial.print(":");
    Serial.print(sim808.GPSdata.minute);
    Serial.print(":");
    Serial.print(sim808.GPSdata.second);
    Serial.print(":");
    Serial.println(sim808.GPSdata.centisecond);
    Serial.print("latitude :");
    Serial.println(sim808.GPSdata.lat);
    Serial.print("longitude :");
    Serial.println(sim808.GPSdata.lon);
    Serial.print("speed_kph :");
    Serial.println(sim808.GPSdata.speed_kph);
    Serial.print("heading :");
    Serial.println(sim808.GPSdata.heading);
    Serial.println();

    //************* Turn off the GPS power ************
    
  } 
  else Serial.print(".");
  sim808.detachGPS();
}

