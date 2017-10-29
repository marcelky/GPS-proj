#define DEBUG 1
//#define NO_NETLIGHT 1   //turn on or off netlight of sim808 board
#define PROTOCOL 2   //1 = TCP and 2 = UDP

#ifdef PROTOCOL
  static int const prot = PROTOCOL;
#else
  static int const prot = 1;          // default value TCP
#endif

/*******************************
 * Definition of error codes
 *******************************/
#define CIPSHUT_FAIL 1
#define IPR9600_FAIL 2
#define CSTT_UNINET_FAIL 3
#define CIICR_FAIL 4
#define CDNSGIP_FAIL 5

#define NO_GPRS_SIGNAL 20
#define DNS_ERROR 21
#define PDP_DEACT 22

#define NO_GPS_DATA_AVAILABLE 31

#define START_GPRS_TCP_FAIL 50
#define START_GPRS_UDP_FAIL 51
#define SEND_TCP_DATA_FAIL 52
#define SEND_UDP_DATA_FAIL 53


//The content of messages sent
//#define MESSAGE  "hello,world"
#define MESSAGE_LENGTH 30
//#define MESSAGE_LENGTH 160
char message[MESSAGE_LENGTH];

char phone[16];
char datetime[24];
char gprsBuffer[64];
char *s = NULL;
char smsMessage[64];


#include <DFRobot_sim808.h>
#include <sim808.h>
#include <SoftwareSerial.h>
#include <string.h>
#include <SimpleTimer.h>
#define rxPin 2
#define txPin 3


SoftwareSerial GPRS(rxPin, txPin); // RX, TX
DFRobot_SIM808 sim808(&GPRS);
SimpleTimer timer;

//char* key;
//All this char variable have reserved one extra char to insert \0 at end
char gpsMode[4]="000";
char dateTime[11]="000000.000";
char latitude[10]="0000.0000";
char north_south[2]="N";
char longitude[11]="00000.0000";
char east_west[2]="W";
char MSL_altitude[8]="00000.0";
char Speed[8]="000.000";
char GEOID[5]="00000";
boolean DNS_OK = true;
int currentRadioSig = 10;   //currentRadioSig/previousRadioSig = 10 to avoid configure again the data connection
int previousRadioSig = 10;  //when enter the first time of loop to send data


//byte pos = 0;  //WHAT POSITION WE ARE AT IN THAT BUFFER
//my variables MKY
//const int BUFFER_SIZE = 110;
const int BUFFER_SIZE = 90;
//char infoLocation[BUFFER_SIZE];  

char buffer[20];                 // WHAT WE ARE READING INTO
int posBuf;
int seconds=0;
int Powerkey=6;                 //Power pin for SIM808


/*******************************************************************************************
 * function setup()
 * This function run once at start up of processor, here is made the configuration of 
 * UNO and SIM808.
 *******************************************************************************************/
void setup()
{
  //turn off led of Uno 
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  
  GPRS.begin(9600);   //Set Uno serial connection with SIM808 
  Serial.begin(9600); //Set Uno serial connection with Arduino IDE

  sim808_init(&GPRS,9600);


  //delay(20000); //test to see if this reduce the drain of current of batery

  sim808_check_with_cmd("AT+CPOWD=1","OK\r\n",CMD); 
  //delay(5000);
  pinMode(Powerkey, OUTPUT);   // initialize the digital pin as an output.  
  power();                     // power on the sim808 or power down the sim808  

  //Serial.println("reseting");
  //T delay(60000);
  //delay(5000);

  Serial.print("Starting"); 
  while(!sim808.init()) { 
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Init Complete.");

/*  if(sim808_wait_for_resp("SMS Ready\r\n", DATA, DEFAULT_TIMEOUT * 10, DEFAULT_INTERCHAR_TIMEOUT * 10)) {
    Serial.println("Call/SMS ready");  
  }*/
  
   /******************************************************
   * GPRS Configuration
   ******************************************************/
  //GNSS power control
  sim808_check_with_cmd("AT+CGNSPWR=1","OK\r\n",CMD); 
  //delay(290);
  
  //GNSS navitation, GEO-Fence and speed alarm URC report control
  sim808_check_with_cmd("AT+CGNSURC=0","OK\r\n",CMD); 
  //delay(300);

  //GSM and GPRS configuration
  //check which network GPRS module is connected with 
  //GPRS.println("AT+COPS?");

  //check if mobile number is register in the network
  //GPRS.println("AT+CREG=?");

  /*******************************************************
   * configuration of GPRS data 
   *******************************************************/
  InitGPRS();

  /***********************************************************
  * GPS configuration
  ***********************************************************/
  //GPS configuration
  //GPRS.println("AT+CGPSPWR=1");  //POWER ON of GPS interface
  //SerialSim808_Read();
  sim808_check_with_cmd("AT+CGPSPWR=1\r\n","OK\r\n",CMD); 
  delay(50);
  
  //GPRS.println("AT+CGPSRST=0");  //1 = HOT start of GPS or COLD start = 0
  //SerialSim808_Read();
  sim808_check_with_cmd("AT+CGPSRST=0\r\n","OK\r\n",CMD);
  delay(50);
  
//  GPRS.println("AT+CGPSSTATUS?");
//  // SerialSim808_Read();
//  delay(50);
  
  /*************************************************
   * Special config for SMS and voice call
   *************************************************/
  //PARAM FOR SMS AND RECEIVE CALL
  if(!sim808_check_with_cmd("AT+CMGF=1\r\n","OK\r\n",CMD)){ //Set SMS for text mode not HEX when read it
    delay(1);//Serial.println("fail CMGF");     
  }else {
    delay(1);//Serial.println("ok CMGF");
  }
  delay(25);
  
  if(!sim808_check_with_cmd("AT+CMGDA=\"DEL ALL\"\r\n","OK\r\n",CMD)){ //delete all SMS
    delay(1);//Serial.println("fail del SMS");     
  }else {
    delay(1);//Serial.println("ok del SMS");
  }
  delay(25);

  sim808_check_with_cmd("AT+CLIP=1\r\n","OK\r\n",CMD);
  delay(25);

  //turn off netlight of sim808
  #ifdef NO_NETLIGHT
    sim808_send_cmd("AT+CNETLIGHT=0\r\n");
  #endif  

  //wait 
  wait_GPS_connect(240);  //wait GPS connect 2minutes

  
  GetGPSLocation();
  
  timer.setInterval(60000, GetGPSLocation); //call subroutine to GetGPSLocation every minute
  //timer.setInterval(1000, PrintSecondsElapsed);
}


/*******************************************************************************************
 * function loop()
 * This function keep running in loop, it is the main function which tasks of UNO are placed.
 *******************************************************************************************/
void loop()
{
  timer.run();
  //Serial.print(".");
  //delay(1000);

  //Only for debug purpose, send directly the AT command manually via serial interface
  #ifdef DEBUG
  if(GPRS.available())
      Serial.write(GPRS.read());
   if(Serial.available())
      GPRS.write(Serial.read());
  #endif
  

  
//  //Receiving call 
//  if(sim808.readable()){
//    sim808_read_buffer(gprsBuffer,32,DEFAULT_TIMEOUT);
//    //Serial.print("LoopBuf:");
//    //Serial.println(gprsBuffer);
//  
//    //**** Detect the current state of the telephone or SMS ****************
//    if(NULL != strstr(gprsBuffer,"RING")) {
//        sim808.answer(); 
//        createSMSMessage(smsMessage,latitude, longitude, MSL_altitude, Speed);
//        if(extractCallingNumber(gprsBuffer)){
//          sim808.sendSMS(phone,smsMessage);
//        }else{
//          //Serial.println("No CLIP to send SMS");
//        }
//        delay(2000);
//        sim808_send_cmd("AT+CTTS=2,\"hello,欢迎使用语音合系统\"\r\n");            
//  }
//
//  //check if GPRS signal still good
//  if(NULL != (s = strstr(gprsBuffer,"CDNSGIP: 0,8"))) {
//    Serial.println("No signal or DNS fault loop");
//    DNS_OK = false; 
//  }        
//  

  //Receive SMS message   
//  if(NULL != (s = strstr(gprsBuffer,"+CMTI: \"SM\""))) { //SMS: $$+CMTI: "SM",24$$
//    sim808_clean_buffer(gprsBuffer,32); 
//    createSMSMessage(smsMessage,latitude, longitude, MSL_altitude, Speed);
//    if(!sim808_check_with_cmd("AT+CMGF=1\r\n","OK\r\n",CMD)){ //Set SMS for text mode not HEX when read it
//      delay(1); //Serial.println("fail CMGF");     
//    }else {
//      delay(1);//Serial.println("ok CMGF");
//    }
//    delay(25);
//
//    GPRS.println("AT+CMGR=1");
//    sim808_read_buffer(gprsBuffer,64,DEFAULT_TIMEOUT);
//    //Serial.print(gprsBuffer);
//
//    if(!sim808_check_with_cmd("AT+CMGDA=\"DEL ALL\"\r\n","OK\r\n",CMD)){ //delete all SMS
//      delay(1);//Serial.println("fail del SMS");     
//    }else {
//      delay(1);//Serial.println("ok del SMS");
//    }
//
//    
//    if(extractCallingNumberSMS(gprsBuffer)){      
//      sim808.sendSMS(phone,smsMessage);
//    }else{
//      Serial.println("No valid # for SMS");
//    }
//  }


    
//    sim808_clean_buffer(gprsBuffer,32);       
//  }  


}

/**************************************************************************************
* InitGPRS()
* This function initialize the GPRS during start/restart of arduino/sim808 or when
* during normal operation it lost connection signal with GPRS.
* Return error code:
* 0 = Init successfull
* 1 = Error with AT+IPR=9600
* 2
***************************************************************************************/
int InitGPRS(){
 byte error = 0;  
 /**************************************
  *AT+CGREG?
  *Request Network registration status 
  *Response:
  *+CGREG: 0,1   //1 indicate registered
  *OK
  *************************************/
  //GPRS.println("AT+CGREG?");

  /************************************
   *AT+CGATT? 
   *Attach or detach from GPRS service 
   *Response:
   *+CGATT: 1
   *OK
   ************************************/
   //GPRS.println("AT+CGATT?");

   /************************************
    *AT+CIPSHUT 
    *Detach PDP context 
    *OK
    ************************************/

   if(!sim808_check_with_cmd("AT+CIPSHUT\r\n","OK\r\n",CMD)){  
    error = CIPSHUT_FAIL;   
   }
   delay(50);

  /***********************************
   *Set GPRS data rate
   *AT+IPR=9600
   *OK
   ***********************************/
   if(!sim808_check_with_cmd("AT+IPR=9600\r\n","OK\r\n",CMD)){  
    error = IPR9600_FAIL;   
   }
   delay(50);
  
  /******************************************
   *Set the APN id to connect to gprs network 
   *China UNICOM = UNINET
   *AT+CSTT="UNINET"
   *OK
   ******************************************/  
  if(!sim808_check_with_cmd("AT+CSTT=\"UNINET\"\r\n","OK\r\n",CMD)){ 
    error = CSTT_UNINET_FAIL;   
  }
  delay(50);

  /******************************************* 
   *bring up the wireless connection with GPRS  
   *AT+CIICR
   *OK
   *******************************************/ 
  if(!sim808_check_with_cmd("AT+CIICR\r\n","OK\r\n",CMD)){  
    error = CIICR_FAIL;
  }
  delay(50);

  /*******************************************  
   *get the local ip number   
   *AT+CIFSR
   *10.x.x.x
   *******************************************/
  sim808_send_cmd("AT+CIFSR\r\n");
  delay(50);

  /*******************************************  
   *Get the remote IP address   
   *AT+CDNSGIP="ardugps.hopto.org"
   *OK
   *+CDNSGIP: 1,"ardugps.hopto.org","58.100.82.70"
   *******************************************/
  if(!sim808_check_with_cmd("AT+CDNSGIP=\"ardugps.hopto.org\"\r\n","OK\r\n",CMD)){  
    error = CDNSGIP_FAIL; 
  }
  delay(50);
  return error;
}



/****************************************************************************************
* extractCallingNumber(char *messages)
* This function when an incoming call come and CLIP is enabled, it will prompt at serial
* the message:
*   RING
*   
*   +CLIP: "18968192062",161,"",0,"",0
* 
* Here we extract the phone number
*****************************************************************************************/
boolean extractCallingNumber(char *inMessage){
char *token;

   /* get the first token */
   //Serial.println("valor do buffer :");
   //Serial.println(inMessage);

   if(NULL == strstr(inMessage,"+CLIP")){
     return false;
   }
   
   //Get Command sent
   token = strtok(inMessage,"\n\r" );
   if(token == NULL)  return false;

   //Get "+CLIP: " string
   token = strtok(NULL, "\"");   
   if(token == NULL)  return false;
   
   //Get phone number value
   token = strtok(NULL, "\"");
   if(token != NULL) {    
      strncpy(phone,token,sizeof(phone));
      phone[sizeof(phone)-1] = '\0';

      //strcpy(gpsMode,token);
      Serial.print("ph#: ");
      Serial.println(phone);   
   }else return false;
   
   return true;
  
}

/****************************************************************************************
* extractCallingNumberSMS(char *messages)
* This function when receive an SMS extract the calling number from SMS.
* AT+CMGR=#           //read the message # 
* 
* +CMGR: "REC READ","18968192062","","17/10/14,14:07:10+32"* 
* 
* Here we extract the phone number
*****************************************************************************************/
boolean extractCallingNumberSMS(char *inMessage){
char *token;
char temp[16];



   /* get the first token */
   //Serial.println("valor do buffer :");
   //Serial.println(inMessage);

   if(NULL == strstr(inMessage,"+CMGR")){
     return false;
   }
   
   //Get Command sent
   token = strtok(inMessage,"\n\r" );
   if(token == NULL)  return false;

   //Get "+CMGR:" string
   token = strtok(NULL, ":");   
   if(token == NULL)  return false;

   //Get type of SMS "REC READ" or other type not important here 
   token = strtok(NULL, ",");   
   if(token == NULL)  return false;


   
   //Get phone number value
   token = strtok(NULL, ",");
   if(token != NULL) {    
      //removing the " at begining and end of phone number
      //token = phone;
      token++;
      token[strlen(token)-1]='\0'; //now the " is not at beginning/end of toke and in phone either
      
      strncpy(phone,token,sizeof(phone));
      phone[sizeof(phone)-1] = '\0';

      //getAllButFirstAndLast(token,phone);

      
      //strcpy(gpsMode,token);
      Serial.print("ph#: ");
      Serial.println(phone);   
   }else return false;
   
   return true;
 
}

/*************************************************************************************************
*  getAllButFirstAndLast(const char *input, char *output)
*  Ths procedure remove the first and last character of char array 
*  "12938484848"
*  for exemplo for a phone number
************************************************************************************************/
void getAllButFirstAndLast(const char *input, char *output)
{
  int len = strlen(input);
  if(len > 0)
    strcpy(output, ++input);
  if(len > 1)
    output[len - 2] = '\0';
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

/****************************************************************************************
 * powerGPS(int onOff)
 * This procedure power on GPS 1, power on and 0 power off.
 ****************************************************************************************/
/*boolean powerGPS(int onOff){
  if (onOff == 1) 
    if(!sim808_check_with_cmd("AT+CGNSPWR=1\r\n", "OK\r\n", CMD)) return false;
  else 
    if(!sim808_check_with_cmd("AT+CGNSPWR=0\r\n", "OK\r\n", CMD)) return false;

  return true;
}*/
 
/****************************************************************************************
 * GetGPSLocation()
 * This function request the GPS information from SIM808, parse information in global 
 * variables and finally print the information at Serial interface for debugging.
 ****************************************************************************************/
int GetGPSLocation(){
char infoLocation[BUFFER_SIZE];  
posBuf=0;
int i = 0;
int j=0;

//  //waking up GPS after sleep
  sim808_check_with_cmd("AT+CGPSPWR=1\r\n","OK\r\n",CMD); 
  delay(1000);
  sim808_check_with_cmd("AT+CGPSRST=1\r\n","OK\r\n",CMD); 
  delay(1000);

  if(!wait_GPS_connect(25)){ //timeout of 25 seconds to acquire signal  
    sim808_check_with_cmd("AT+CGPSPWR=0\r\n","OK\r\n",CMD); 
    return NO_GPS_DATA_AVAILABLE;
  }


  sim808_clean_buffer(infoLocation,BUFFER_SIZE);
  
  sim808_send_cmd("AT+CGPSINF=2\r\n");
  delay(50);
  if(sim808.readable()) {
    sim808_read_buffer(infoLocation,BUFFER_SIZE,DEFAULT_TIMEOUT);
  }
  Serial.println("buffer2:");
  Serial.println(infoLocation);
  
  
//  while(GPRS.available()){
//    GPRS.read();
//  } 
//  GPRS.println("AT+CGPSINF=2");
//   
//  //Read response of SIM808 at Serial Port, Uno buffer all data without other activities
//  //otherwise it may lose some bytes 
//  //after send the command to GPS, it takes about 100ms to all serial information arrives UNO   
//  delay(50);
   

//  //for(int i=0;i<7;i++){
//  while((posBuf<=99)&&(i<15)){ 
//    while((GPRS.available()>0)){
//      infoLocation[posBuf++]=GPRS.read();
//    }
//    delay(10);
//    i++;
//    //delay(15); 
//  }

  if(!GPSAnalyzer(infoLocation)){
    //sim808_clean_buffer(infoLocation,BUFFER_SIZE);
    sim808_check_with_cmd("AT+CGPSPWR=0\r\n","OK\r\n",CMD);     
    return NO_GPS_DATA_AVAILABLE;
  }


  //debug until here




  sim808_clean_buffer(infoLocation,BUFFER_SIZE);
   //Get Speed of SIM808 from GPS
  while(GPRS.available()){
    GPRS.read();
  } 
  GPRS.println("AT+CGPSINF=32");
  delay(50);
  
  sim808_read_buffer(infoLocation,BUFFER_SIZE,DEFAULT_TIMEOUT); 
  GPSAnalyzer32(infoLocation);

  //Send collected data to mysql server 
  Serial.println("SendError:");
  Serial.println(SendGPSLocation());
  //createSMSMessage(smsMessage,latitude, longitude, MSL_altitude, Speed);

  //put GPS to sleep
  sim808_check_with_cmd("AT+CGPSPWR=0\r\n","OK\r\n",CMD);     
  //delay(5000);
  return 0; //success 
}


/****************************************************************************
 * wait_GPS_connect(int seconds)
 * wait GPS acquire signal
 ****************************************************************************/
boolean wait_GPS_connect(int timeout){
  unsigned long timerStart;    
  timerStart = millis();

  sim808_flush_serial();
  delay(1000);
  
  while (1){
    sim808_clean_buffer(gprsBuffer,64); 
    sim808_send_cmd("AT+CGPSSTATUS?\r\n");
    if(sim808.readable()) {
      sim808_read_buffer(gprsBuffer,64,DEFAULT_TIMEOUT);
    }

    Serial.println(gprsBuffer);
    if(strstr(gprsBuffer,"3D Fix")){
      sim808_clean_buffer(gprsBuffer,64); 
      sim808_flush_serial();
      return true;
     }
    if ((unsigned long) (millis() - timerStart) > timeout * 1000UL) {
      sim808_clean_buffer(gprsBuffer,64); 
      sim808_flush_serial();
      return false;
    }

    delay(10000);
    //sim808_flush_serial();
  }
 }
 

/*void resetBuffer() 
{
  memset(buffer, 0, sizeof(buffer));
  //Serial.print("Valor do pos: ");
  //Serial.println(pos);
  pos = 0;
}//BASICALLY TO RESET THE BUFFER
*/
/*void resetInfoLocationBuffer() 
{
  memset(infoLocation, 0, sizeof(buffer));
  posBuf = 0;
}//BASICALLY TO RESET THE BUFFER*/

void PrintSecondsElapsed(){
  Serial.print("*");
  //Serial.print("*Seconds: ");
  //Serial.println(seconds);
  seconds++;
} 

/******************************************************************************************
 * int SendGPSLocation()
 * return 0  No error, GPRS UP
 *        1 =< Error < 20 see error code initialization of GPRS, see InitGPS() for error code
 *        Error = 20, DNS Failed or weak GPRS signal
 *        
 * This procedure open a GPRS data connection and send latitude and longitude for server.
 * After data is sent it disconnect from server.
 * Send data to server using format "latitude,longitude" = "llll.llll,lllll.llll"
 * obs. no \0 at end like string.
 ******************************************************************************************/

int SendGPSLocation(){  
  char data2db[36]="0000.0000,00000.0000,00000.0,000.000"; //"latitude,longitude,MSL_altitude,Speed"
  byte error;
  int signalStrenght;
  /************************************************************
   * CIPSTATUS result
   * 0 IP STATUS    ready to initiate connection
   * 8 CONNECT OK   ready to send data
   ************************************************************/
  //get signal strength, if < 7 not possible to send data to server
  sim808_clean_buffer(gprsBuffer,64); 
  sim808_send_cmd("AT+CSQ\r\n");
  if(sim808.readable()) {
    sim808_read_buffer(gprsBuffer,24,DEFAULT_TIMEOUT);
    currentRadioSig = getSignalQuality(gprsBuffer);
    Serial.print("sig:");
    Serial.println(currentRadioSig);
  }
  sim808_clean_buffer(gprsBuffer,64); 


/*******************************************************************  
 * previousRadioSignal currentRadioSignal   
 * 0                   0                  nao faz nada e sai
 * 1                   0                  nao faz nada e sai
 * 0                   1                  initGPRS e send data
 * 1                   1                  nao faz nada e send data
 *******************************************************************/
  sim808_send_cmd("AT+CIPSTATUS\r\n");
  if(sim808.readable()) {
    sim808_read_buffer(gprsBuffer,64,DEFAULT_TIMEOUT);
    //Serial.print(gprsBuffer);
  }
  if(currentRadioSig <= 7) {
       Serial.println("NOGPRSsig");
       sim808_send_cmd("AT+CDNSGIP=\"ardugps.hopto.org\"\r\n");
       previousRadioSig = currentRadioSig;                
       return NO_GPRS_SIGNAL;            //error of weak signal or DNS failed                     
  }else{                                 //radio signal > 7
    if(previousRadioSig > 7){            //signal still good
      previousRadioSig = currentRadioSig;                
    }else {                              //signal recovered
      error = InitGPRS();
      previousRadioSig = currentRadioSig;
      if(error > 0) {                    //initGPRS failed
        Serial.print("GPRSfail: ");
        Serial.println(error);
        return error;        
      } 
      Serial.println("GprsInitOK");       
    }            
  }
  

    
  //get the local ip number
  //GPRS.println("AT+CIFSR");
  sim808_send_cmd("AT+CIFSR\r\n");
  delay(50);  
  sim808_send_cmd("AT+CDNSGIP=\"ardugps.hopto.org\"\r\n");
  delay(50);
  
  //start a connection to TCP to URL and port 
  if (prot == TCP){
    if(!SendCIPSTART(TCP)){            //TCP protocol
      return START_GPRS_TCP_FAIL;      //"AT+CIPSTART => FAIL"; 
    } 
  }else {                              //UDP protocol
    if(!SendCIPSTART(UDP)){
      return START_GPRS_UDP_FAIL;      //"AT+CIPSTART => FAIL"; 
    }
  }
  
  //delay(1000);
  delay(100);

  //copy latitude and longitude to send via TCP server
  memset(data2db,'\0',sizeof(data2db));
  strncpy(data2db,latitude,sizeof(latitude));
  strcat(data2db,",");
  strcat(data2db,longitude);

  strcat(data2db,",");
  strcat(data2db,MSL_altitude);

  strcat(data2db,",");
  strcat(data2db,Speed);

  Serial.println("valor de data2db: ");
  Serial.println(data2db);
  
  
  
  if(!SendDataCIPSEND(data2db,sizeof(data2db))){
    if(prot == TCP){                  //TCP protocol
      if(!SendCIPSTART(TCP)){         //Trying to restart TCP protocol     
        return START_GPRS_TCP_FAIL;        
      } 

      //delay(1000);
      delay(100);
      if(!SendDataCIPSEND(data2db,sizeof(data2db))){    //Trying to resend TCP data
        return SEND_TCP_DATA_FAIL;                      //"CIPSEND =>RESEND FAIL"        
      } 

    }else{                             //UDP protocol
      return SEND_UDP_DATA_FAIL;      
    }
  } 

  delay(50);

  if (CheckConnectionStatus()) {
    sim808_check_with_cmd("AT+CIPCLOSE\r\n", "CLOSE OK\r\n", CMD);
    //Serial.println("Uno have disconnected from Server");
  }

  return 0; // no error on GPRS 

}//end procedure

/*************************************************************************************
 * SendCIPSTART()
 *************************************************************************************/
boolean SendCIPSTART (Protocol ptl)
{
  //char bufferC[30];
  
  if(ptl == TCP) {
    if(!sim808_check_with_cmd("AT+CIPSTART=\"TCP\",\"ardugps.hopto.org\",6789\r\n","OK\r\n",CMD)){  
      return false;    
    }
    else{
      return true;  
    }
  } 
  else { 
    if(ptl == UDP) {
      if(!sim808_check_with_cmd("AT+CIPSTART=\"UDP\",\"ardugps.hopto.org\",6789\r\n","ERROR\r\n\r\nALREADY CONNECT\r\n",CMD)){  
        return false;    
      }
      else { 
        
        //void sim808_read_buffer(char *buffer, int count, unsigned int timeout, unsigned int chartimeout)
        /*sim808_send_cmd("AT+CDNSGIP=\"ardugps.hopto.org\"\r\n");
        sim808_read_buffer(bufferC,30,5,500);
        Serial.println("Result of CDNSGIP");
        Serial.println(bufferC);*/
  
               
        return true;
      }
    } 
    else {
      return false;
    }
  }


}



/*************************************************************************************
 * SendDataCIPSEND()
 * This procedure send data through the wireless connection server.
 *************************************************************************************/

int SendDataCIPSEND(const char * str, int len)
{
char num[4];
if(len > 0){
  sim808_send_cmd("AT+CIPSEND=");
  itoa(len, num, 10);
  sim808_send_cmd(num);
  if(!sim808_check_with_cmd("\r\n",">",CMD)) {
    return 0;
  }
  /*if(0 != sim808_check_with_cmd(str,"SEND OK\r\n", DEFAULT_TIMEOUT * 10 ,DATA)) {
        return 0;
  }*/
  //delay(1000);
  delay(100);
  sim808_send_cmd(str);
  //delay(1000);
  delay(100);
  sim808_send_End_Mark();
/*  if(!sim808_wait_for_resp("SEND OK\r\n", DATA, DEFAULT_TIMEOUT * 10, DEFAULT_INTERCHAR_TIMEOUT * 10)) {
    return 0;
  }*/
}
return len;
}

/***************************************************************************************
 * CheckConnectionStatus()
 * This procedure check if TCP connection via GPRS is active or not.
 ***************************************************************************************/
bool CheckConnectionStatus(void)
{
  char resp[96];
  sim808_send_cmd("AT+CIPSTATUS\r\n");
  sim808_read_buffer(resp,sizeof(resp),DEFAULT_TIMEOUT);
  if(NULL != strstr(resp,"CONNECTED")) {
      return true;
  } else {
      return false;
  }
}


/********************************************************************************
 * boolean GPSAnalyzer(char *gpsBuffer)
 * Analyze the Serial information received after request the information 
 * AT+CCPSINF=0
 * +CGPSINF: 2,050656.000,0030.2706,N,00120.1007,E,1,9,0.90,51.9,M,7.0,M,,
 * AT+CGPSINF=2, get GPS location with format
 * Message ID       = 2
 * UTC time         = [hhmmss.sss]
 * Latitude         = [+/-[0dd.dddd]]
 * N/S indicator    = [N/S]
 * Longitude        = [+/-[0ddd.dddd]]
 * E/W indicator    = [W/E]
 * Position fix     = 1
 * Satellites used  = range 0 to 12
 * HDOP             = x.xx
 * MSL Altitude     = xxx.x
 * Units            = M
 * Geoid Separation = xxx
 * Units            = M
  ********************************************************************************/
boolean GPSAnalyzer(char *gpsBuffer) {
   char *token;
   char *validation;

   validation = gpsBuffer; 
   /* get the first token */
   //Serial.println("**Content of gpsBuffer**");

//   Serial.println("buffer3:");
//   Serial.println(gpsBuffer);

   
   if(!strstr(validation,"+CGPSINF: 2")) { //not valid gps data
     return false; 
   }
    
   if(strstr(validation,"0000.0000,N,00000.0000,E")) { //no lat or lng available
     return false; 
   }
   

   
   //Get Command sent
   token = strtok(gpsBuffer,"\n\r" );
   if(token == NULL)  return false;

   //Get "+CGPSINF:" string
   token = strtok(NULL, ":");   
   if(token == NULL)  return false;
   
   //Get CGPSINF value
   token = strtok(NULL, ",");
   if(token != NULL) {    
      //strncpy(gpsMode,token,sizeof(gpsMode));
      //gpsMode[sizeof(gpsMode)-1] = '\0';

      strcpy(gpsMode,token);
      //Serial.print("CGPSINF_MODE    : ");
      //Serial.println(gpsMode);   
   }else return false;


   //Get time    
   token = strtok(NULL, ",");
   if(token != NULL) {
      strncpy(dateTime,token,sizeof(dateTime));
      dateTime[sizeof(dateTime)-1] = '\0';        
      //Serial.print("TIME            : ");
      //Serial.println(dateTime); 
            
   }else return false;

   //Get Latitude
   token = strtok(NULL, ",");
   if(token != NULL) {
      strncpy(latitude,token,sizeof(latitude));
      latitude[sizeof(latitude)-1] = '\0';      
      Serial.print("LATITUDE        : ");
      Serial.println(latitude);      
   }else return false;   

   //Get North or South
   token = strtok(NULL, ",");
   if(token != NULL) {     
      strncpy(north_south,token,sizeof(north_south));
      north_south[sizeof(north_south)-1] = '\0';     
      //Serial.print("N/S INDICATOR  : ");
      //Serial.println(north_south);  
         
   }else return false;   

   //Get Longitude
   token = strtok(NULL, ",");
   if(token != NULL) {    
      strncpy(longitude,token,sizeof(longitude));
      longitude[sizeof(longitude)-1] = '\0';   
     // Serial.print("LONGITUDE       : ");
      //Serial.println(longitude);
      
   }else return false;  

   //Get East or West
   token = strtok(NULL, ",");
   if(token != NULL) {
      strncpy(east_west,token,sizeof(east_west));
      east_west[sizeof(east_west)-1] = '\0';      
      //Serial.print("E/W INDICATOR   : ");
      //Serial.println(east_west);    
      //Serial.println("************************************************************ ");  
        
   }else return false;


   //Skip Position Fix Indicator
   token = strtok(NULL, ",");
   if(token == NULL) {
      return false;
   }

   //Skip Satellites Used
   token = strtok(NULL, ",");
   if(token == NULL) {
      return false;
   }

   //Skip HDOP
   token = strtok(NULL, ",");
   if(token == NULL) {
      return false;
   }

   //Get MSL Altitude
   token = strtok(NULL, ",");
   if(token != NULL) {
      strncpy(MSL_altitude,token,sizeof(MSL_altitude));
      MSL_altitude[sizeof(MSL_altitude)-1] = '\0';      
      //Serial.print("MSL_ALTITUDE   : ");
      //Serial.println(MSL_altitude);            
   }else return false;


   //Skip unit
   token = strtok(NULL, ",");
   if(token == NULL) {
      return false;
   }

   //Get GEOID
   token = strtok(NULL, ",");
   if(token != NULL) {
      strncpy(GEOID,token,sizeof(GEOID));
      GEOID[sizeof(GEOID)-1] = '\0';      
      //Serial.print("GEOID  : ");
      //Serial.println(GEOID);            
   }else return false;   

   return true;     
}

/********************************************************************************
* createSMSMessage(char *message,char lat, char lng, char alt, char sp)
* This function format the message to be sent to user
*********************************************************************************/
void createSMSMessage(char *message,char *lat, char *lng, char *alt, char *sp)
{
  memset(message,'\0',sizeof(message));
  strncpy(message,"* LAT=",sizeof("Lat.: "));  
  strcat(message,lat);
  strcat(message,", ");
  strcat(message,"LNG=");
  strcat(message,lng);
  strcat(message,", ");
  strcat(message,"ALT=");  
  strcat(message,alt);
  strcat(message,"m, ");
  strcat(message,"SPEED=");
  strcat(message,sp);  
  strcat(message,"kmh *");
  //Serial.println("SMS");
  Serial.println(message);
}



/********************************************************************************
* GPSAnalyzer32(infoLocation);
* GPRMC
* AT+CGPSINF=32
* +CGPSINF: 32,032432.000,A,3016.2261,N,12006.0382,E,0.185,116.11,131017,,,A
* Message ID          : $GPRMC = 32 (RMC protocol header)
* UTC Time            : hhmmss.sss
* Status              : A = data valid or V = data not valid
* Latitude            : ddmm.mmmmmm
* N/S Indicator       : N/S
* Longitude           : dddmm.mmmmmm
* E/W Indicator       : E/W
* Speed Over Ground   : knots
* Course Over Ground  : degrees (TRUE)
* Date                : ddmmyy
* Magnetic Variation  : degrees (E/W)
* East/West Indicator : E/W
* Mode                : A = Autonomous
*                       D = DGPS
*                       E = DR
*                       N = Output Data Not Valid
*                       R = Coarse Position
********************************************************************************/
boolean GPSAnalyzer32(char *gpsBuffer) {
   char *token;
   byte i=0;
   float Speedkmh;
   /* get the first token */
   //Serial.println("****************Content of gpsBuffer*******************");
   //Serial.println("valor do buffer :");
   //Serial.println(gpsBuffer);
   
   //Get Command sent
   token = strtok(gpsBuffer,"\n\r" );
   if(token == NULL)  return false;

   //Get "+CGPSINF:" string
   token = strtok(NULL, ":");   
   if(token == NULL)  return false;
   
   //Get CGPSINF value
   token = strtok(NULL, ",");
   
   while (i<8) {     
     if((token != NULL) && (i==7)){
       strncpy(Speed,token,sizeof(Speed)); //Speed in Knot
       Speed[sizeof(Speed)-1] = '\0';     
       //Serial.print("SPEED  KNOT      : ");
       //Serial.println(Speed);             
       
       Speedkmh = atof(Speed);
       Speedkmh = Speedkmh*1.852;
       dtostrf(Speedkmh,3,3,Speed);   

       //Serial.print("SPEED  KMH       : ");
       //Serial.println(Speed);             
       
       return true;
     }else {
       if (token == NULL){
         return false;
       }else{
        token = strtok(NULL, ",");
        i++;       
       }
     }    
   }  
}


/*************************************************************
 * CheckSignalQuality(char *)
 * scan and print the signal quality
 * return 99 unknow or failed to extract 
 *        0 to 98 valid result 
 *************************************************************/

int getSignalQuality(char *signalBuffer){
  char *token;
  char sig[3]="99";
  /* get the first token */
  //Serial.println("****************Content of gpsBuffer*******************");
  //Serial.println("bufferCSQ:");
  //Serial.println(signalBuffer);
  
  //Get Command sent
  token = strtok(signalBuffer,"\n\r" );
  if(token == NULL)  return 99;
  
  //Get "+CSQ:" string
  token = strtok(NULL, " ");   
  if(token == NULL)  return 99;
  
  //Get +CSQ: value
  token = strtok(NULL, ",");
  if(token == NULL) {
    return 99;
  }else{
    strncpy(sig,token,sizeof(sig)); //Speed in Knot
    sig[sizeof(sig)-1] = '\0';  
    return atoi(sig);
  }  
}
 


/********************************************************************************
* GPVTG
* AT+CGPSINF=64
* +CGPSINF: 64,299.79,T,,M,0.072,N,0.133,K,A
* Message ID    : $CPVTG = 64
* Course        : degrees
* Reference     : TRUE
* Course        : degrees (Measured heading)
* Reference     : M (Magnetc)
* Speed         : knots (Measured horizontal speed)
* Units         : N (Knots)
* Speed         : Km/h (Measured horizontal speed)
* Units         : Kilometers/h
* Mode          : A = Autonomous
*                 D = DGPS
*                 E = DR
*                 N = Output Data Not Valid
*                 R = Coarse Position
********************************************************************************/


