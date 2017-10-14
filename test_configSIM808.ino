#define DEBUG 1
#define PROTOCOL 2 //1 = TCP and 2 = UDP

#ifdef PROTOCOL
  static int const prot = PROTOCOL;
#else
  static int const prot = 1;          // default value TCP
#endif


//The content of messages sent
//#define MESSAGE  "hello,world"
#define PHONE_NUMBER "18968192062"
#define MESSAGE_LENGTH 30
//#define MESSAGE_LENGTH 160
char message[MESSAGE_LENGTH];
int messageIndex = 0;
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

/*enum Protocol {  //if enable #include <DFRobot_sim808.h> and DFRobot_SIM808 sim808(&GPRS); disable this enum
    CLOSED = 0,
    TCP    = 1,
    UDP    = 2,
};*/


char* key;
//All this char variable have reserved one extra char to insert \0 at end
char gpsMode[4]="000";
char dateTime[11]="000000.000";
char latitude[10]="0000.0000";
char north_south[2]="N";
char longitude[11]="00000.00000";
char east_west[2]="W";
char MSL_altitude[8]="00000.0";
char Speed[8]="000.000";
/*char positionFix[2];
char numSatelite[2];
char hdop[5];
char altitude[5];
char altitudeUnit[1];
char geoid[3];
char geoid_unit[1];
*/


byte pos = 0;  //WHAT POSITION WE ARE AT IN THAT BUFFER
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
  GPRS.begin(9600);   //Set Uno serial connection with SIM808 
  Serial.begin(9600); //Set Uno serial connection with Arduino IDE

  /*while(!sim808.init(GPRS,9600)) { 
    delay(1000);
    Serial.print("Sim808 init error\r\n");
  }
  Serial.println("Sim808 init success");
  Serial.println("Start to call ...");
  */
  sim808_init(&GPRS,9600);
    
  GPRS.println("AT+CPOWD=1"); 
  delay(10000);
  pinMode(Powerkey, OUTPUT);   // initialize the digital pin as an output.  
  power();                     // power on the sim808 or power down the sim808  

  Serial.println("reseting");
  //T delay(60000);
  delay(10000);
  /******************************************************
   * GPS Configuration
   ******************************************************/
  //GNSS power control
  GPRS.println("AT+CGNSPWR=1");
  delay(290);
  //GNSS navitation, GEO-Fence and speed alarm URC report control
  GPRS.println("AT+CGNSURC=0");
  delay(300);

  //Select SMS message format
  /*GPRS.println("AT+CMGF=1"); 
  delay(300);*/


  //GSM and GPRS configuration
  //check which network GPRS module is connected with 
  GPRS.println("AT+COPS?");
  //check if mobile number is register in the network
  GPRS.println("AT+CREG=?");
  //check if GPRS module is registered
  GPRS.println("AT+CGREG=?");
  //GSM Service's status
  GPRS.println("AT+CGATT?");
  //
  GPRS.println("AT+GSV");
  //set the data rate of gprs
  //GPRS.println("AT+IPR=9600");
  if(!sim808_check_with_cmd("AT+IPR=9600\r\n","OK\r\n",CMD)){  
    delay(1);
    //Serial.println("AT+IPR=9600 => FAIL");    
  }else {
    delay(1);
    //Serial.println("AT+IPR=9600 => OK");
  }
  delay(50);
  
  //Set the APN id to connect to gprs network
  //GPRS.println("AT+CSTT=\"UNINET\"");
  if(!sim808_check_with_cmd("AT+CSTT=\"UNINET\"\r\n","OK\r\n",CMD)){ 
    delay(1); 
    //Serial.println("AT+IPR => FAIL");    
  }else{ 
    delay(1);
    //Serial.println("AT+IPR => OK");
  }
  delay(50);
    
  //bring up the wireless connection
  //GPRS.println("AT+CIICR");
  if(!sim808_check_with_cmd("AT+CIICR\r\n","OK\r\n",CMD)){  
    delay(1);
    Serial.println("AT+CIICR => FAIL");    
  }else{ 
    delay(1);
    //Serial.println("AT+CIICR => OK");
  }
  delay(50);
    
  //get the local ip number
  //GPRS.println("AT+CIFSR");
  sim808_send_cmd("AT+CIFSR\r\n");
  delay(50);
    
  //Get the remote IP address
  //GPRS.println("AT+CDNSGIP=\"ardugps.hopto.org\"");
  if(!sim808_check_with_cmd("AT+CDNSGIP=\"ardugps.hopto.org\"\r\n","OK\r\n",CMD)){  
    delay(1);
    //Serial.println("AT+CDNSGIP => FAIL");    
  }else {
    delay(1);
    //Serial.println("AT+CDNSGIP => OK");  
  }
  delay(50);
 
  //GPS configuration
  GPRS.println("AT+CGPSPWR=1");  //POWER ON of GPS interface
  //SerialSim808_Read();
  delay(50);
  
  GPRS.println("AT+CGPSRST=1");  //1 = HOT start of GPS or COLD start = 0
  //SerialSim808_Read();
  delay(50);
  
  GPRS.println("AT+CGPSSTATUS?");
  // SerialSim808_Read();
  delay(50);
  

  //PARAM FOR SMS AND RECEIVE CALL
  if(!sim808_check_with_cmd("AT+CMGF=1\r\n","OK\r\n",CMD)){ //Set SMS for text mode not HEX when read it
    Serial.println("fail CMGF");     
  }else {
    Serial.println("ok CMGF");
  }
  delay(25);
  
  if(!sim808_check_with_cmd("AT+CMGDA=\"DEL ALL\"\r\n","OK\r\n",CMD)){ //delete all SMS
    Serial.println("fail del SMS");     
  }else {
    Serial.println("ok del SMS");
  }
  delay(25);

  if(!sim808_check_with_cmd("AT+CLIP=1\r\n","OK\r\n",CMD)){ //HABILITAR O CLIP
    Serial.println("fail CLIP");     
  }else {
    Serial.println("ok CLIP");
  }
  delay(25);
  
  timer.setInterval(60000, GetGPSLocation);
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
  

  //Handling of make and receive call
  if(sim808.readable()){
    sim808_read_buffer(gprsBuffer,32,DEFAULT_TIMEOUT);
    Serial.print(gprsBuffer);
  
    //**** Detect the current state of the telephone or SMS ****************
    if(NULL != strstr(gprsBuffer,"RING")) {
        sim808.answer(); 
        if(extractCallingNumber(gprsBuffer)){
          sim808.sendSMS(phone,smsMessage);
        }else{
          Serial.println("No CLIP to send SMS");
        }
        
        //sim808_send_cmd("AT+CTTS=2,\"hello,欢迎使用语音合系统\"\r\n");  
        //sim808.sendSMS(PHONE_NUMBER,smsMessage);
    
    }else{
      if(NULL != (s = strstr(gprsBuffer,"+CMTI: \"SM\""))) { //SMS: $$+CMTI: "SM",24$$
        sim808_clean_buffer(gprsBuffer,32); 

        if(!sim808_check_with_cmd("AT+CMGF=1\r\n","OK\r\n",CMD)){ //Set SMS for text mode not HEX when read it
          Serial.println("fail CMGF");     
        }else {
          Serial.println("ok CMGF");
        }
        delay(25);
   
        GPRS.println("AT+CMGR=1");
        sim808_read_buffer(gprsBuffer,64,DEFAULT_TIMEOUT);
        Serial.print(gprsBuffer);

        if(!sim808_check_with_cmd("AT+CMGDA=\"DEL ALL\"\r\n","OK\r\n",CMD)){ //delete all SMS
          Serial.println("fail del SMS");     
        }else {
          Serial.println("ok del SMS");
        }

        
        if(extractCallingNumberSMS(gprsBuffer)){
          sim808.sendSMS(phone,smsMessage);
        }else{
          Serial.println("No valid # for SMS");
        }
      }
     
    }
    sim808_clean_buffer(gprsBuffer,32);       
  }  


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
   Serial.println(inMessage);

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
      Serial.print("phone    : ");
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
   Serial.println(inMessage);

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
      Serial.print("phone: ");
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
void GetGPSLocation(){
char infoLocation[BUFFER_SIZE];  
posBuf=0;
int i = 0;

  //Get position
  while(GPRS.available()){
    GPRS.read();
  } 
  GPRS.println("AT+CGPSINF=2");
   
  //Read response of SIM808 at Serial Port, Uno buffer all data without other activities
  //otherwise it may lose some bytes 
  //after send the command to GPS, it takes about 100ms to all serial information arrives UNO   
  delay(50);
   

  //for(int i=0;i<7;i++){
  while((posBuf<=99)&&(i<15)){ 
    while((GPRS.available()>0)){
      infoLocation[posBuf++]=GPRS.read();
    }
    delay(10);
    i++;
    //delay(15); 
  }
  GPSAnalyzer(infoLocation);





  //Get Speed
  while(GPRS.available()){
    GPRS.read();
  } 
  GPRS.println("AT+CGPSINF=32");
   
  //Read response of SIM808 at Serial Port, Uno buffer all data without other activities
  //otherwise it may lose some bytes 
  //after send the command to GPS, it takes about 100ms to all serial information arrives UNO   
  delay(50);
   

/*  posBuf=0;
  i=0;
  //for(int i=0;i<7;i++){
  while((posBuf<=99)&&(i<15)){ 
    while((GPRS.available()>0)){
      infoLocation[posBuf++]=GPRS.read();
    }
    delay(10);
    i++;
    //delay(15); 
  }*/
  
  sim808_clean_buffer(infoLocation,sizeof(infoLocation));
  sim808_read_buffer(infoLocation,sizeof(infoLocation),DEFAULT_TIMEOUT,DEFAULT_INTERCHAR_TIMEOUT); 
  GPSAnalyzer32(infoLocation);

   

  //Send collected data to mysql server 
  SendGPSLocation();
  createSMSMessage(smsMessage,latitude, longitude, MSL_altitude, Speed);
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
 * SendGPSLocation()
 * This procedure open a GPRS data connection and send latitude and longitude for server.
 * After data is sent it disconnect from server.
 * Send data to server using format "latitude,longitude" = "llll.llll,lllll.llll"
 * obs. no \0 at end like string.
 ******************************************************************************************/

void SendGPSLocation(){  
  //char latitude[10]="0000.0000";
  //char longitude[11]="00000.00000";
  //char MSL_altitude[8]="00000.0";
  //char Speed[8]="000.000";
  //char data2db[21]="0000.0000,00000.0000";  //"latitude,longitude"
  char data2db[36]="0000.0000,00000.0000,00000.0,000.000"; //"latitude,longitude,MSL_altitude,Speed"
    
  //get the local ip number
  //GPRS.println("AT+CIFSR");
  sim808_send_cmd("AT+CIFSR\r\n");  
   
  //Get the remote IP address
  //GPRS.println("AT+CDNSGIP=\"ardugps.hopto.org\"");
  if(!sim808_check_with_cmd("AT+CDNSGIP=\"ardugps.hopto.org\"\r\n","OK\r\n",CMD)){  
    delay(1);
    //Serial.println("AT+CDNSGIP => FAIL");    
  }else {
    delay(1);
    //Serial.println("AT+CDNSGIP => OK");  
  }
  //delay(1000);
  delay(100);
  
  //start a connection to TCP to URL and port 
  /*if(!sim808_check_with_cmd("AT+CIPSTART=\"TCP\",\"ardugps.hopto.org\",6789\r\n","OK\r\n",CMD)){  
    Serial.println("AT+CIPSTART => FAIL");    
  }else Serial.println("AT+CIPSTART => OK");
  delay(1000);*/

  if (prot == TCP){
    if(!SendCIPSTART(TCP)){
      delay(1);
      //Serial.println("AT+CIPSTART => FAIL"); 
    } else {
      delay(1);
      //Serial.println("AT+CIPSTART => OK");
    }
  }else {
    if(!SendCIPSTART(UDP)){
      delay(1);
      //Serial.println("AT+CIPSTART => FAIL"); 
    } else {
      delay(1);
      //Serial.println("AT+CIPSTART => OK");    
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

  //Serial.println("valor de data2db: ");
  //Serial.println(data2db);
  
  
  
  if(!SendDataCIPSEND(data2db,sizeof(data2db))){
    //Serial.println("CIPSEND => FAIL");
    //Try to resend    
    /*if(!sim808_check_with_cmd("AT+CIPSTART=\"TCP\",\"ardugps.hopto.org\",6789\r\n","OK\r\n",CMD)){  
      Serial.println("AT+CIPSTART => FAIL");    
    }else Serial.println("AT+CIPSTART => OK");*/

    if(prot == TCP){
      if(!SendCIPSTART(TCP)){
        delay(1);
        //Serial.println("AT+CIPSTART => FAIL"); 
      } else {
        delay(1);
        //Serial.println("AT+CIPSTART => OK");
      }
  
      //delay(1000);
      delay(100);
      if(!SendDataCIPSEND(data2db,sizeof(data2db))){
        //Serial.println("CIPSEND =>RESEND FAIL");
        delay(50);
      } else {
        delay(1);
        //Serial.println("CIPSEND => RESEND OK");
      }
    }    
  } else {
    delay(1);
    //Serial.println("CIPSEND => OK");
  }

  delay(50);

  if (!CheckConnectionStatus()) {
    //Serial.println("Uno was already disconnected from Server");
    delay(1);
  }
  else{
    sim808_check_with_cmd("AT+CIPCLOSE\r\n", "CLOSE OK\r\n", CMD);
    //Serial.println("Uno have disconnected from Server");
  }
  //delay(3000);

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

   /* get the first token */
   //Serial.println("**Content of gpsBuffer**");
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
      Serial.print("LONGITUDE       : ");
      Serial.println(longitude);
      
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
      Serial.print("MSL_ALTITUDE   : ");
      Serial.println(MSL_altitude);    
        
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

       Serial.print("SPEED  KMH       : ");
       Serial.println(Speed);             
       
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


