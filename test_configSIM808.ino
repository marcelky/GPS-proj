#include <DFRobot_sim808.h>
#include <sim808.h>
#include <SoftwareSerial.h>
#include <string.h>
#include <SimpleTimer.h>
#define rxPin 2
#define txPin 3
//#define CTRL(x) (#x[0]-'a'+1)

SoftwareSerial GPRS(rxPin, txPin); // RX, TX
//DFRobot_SIM808 sim808(&GPRS);
SimpleTimer timer;

char* key;
String gpsMode;
//String latitude;
char latitude[11];
String north_south;
//String longitude;
char longitude[11];
String east_west;
String positionFix;
String numSatelite;
String hdop;
String altitude;
String altitudeUnit;
String geoid;
String geoid_unit;
String dateTime;


byte pos = 0;  //WHAT POSITION WE ARE AT IN THAT BUFFER
//my variables MKY
//const int BUFFER_SIZE = 110;
const int BUFFER_SIZE = 80;
boolean matchLocationMessage = true;
String content = "";
char character;
char bufferTCP[10];
//T char bufferSerial[BUFFER_SIZE];  // CREATE THIS BUFFER IN ORDER TO READ SERIAL AT ONCE
//T1 char infoLocation[BUFFER_SIZE];  
char buffer[20];                 // WHAT WE ARE READING INTO
int posTemp;
int posBuf;
int seconds=0;
int Powerkey=6;                 //Power pin for SIM808

enum _parseState 
{
  PS_DETECT_NEW_LINE, //first byte to init sync
  PS_READ_GPS_MODE,
  PS_READ_TIME,  
  PS_READ_LATITUDE,
  PS_READ_NORTH_SOUTH,  
  PS_READ_LONGITUDE,
  PS_READ_EAST_WEST,
  PS_READ_POSITION_FIX,
  PS_READ_NUM_SAT,
  PS_READ_HDOP,
  PS_READ_ALTITUDE,
  PS_READ_ALT_UNIT,
  PS_READ_GEOID_SEPARATION,
  PS_READ_GEOID_UNIT,
  PS_READ_TTFF,
  PS_DETECT_MSG_TYPE,
  PS_DETECT_MSG_TYPE2
};
byte state =  PS_DETECT_NEW_LINE; //PS_DETECT_MSG_TYPE;   


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
    Serial.println("AT+IPR=9600 => FAIL");    
  }else Serial.println("AT+IPR=9600 => OK");
  
  //Set the APN id to connect to gprs network
  //GPRS.println("AT+CSTT=\"UNINET\"");
  if(!sim808_check_with_cmd("AT+CSTT=\"UNINET\"\r\n","OK\r\n",CMD)){  
    Serial.println("AT+IPR => FAIL");    
  }else Serial.println("AT+IPR => OK");
    
  //bring up the wireless connection
  //GPRS.println("AT+CIICR");
  if(!sim808_check_with_cmd("AT+CIICR\r\n","OK\r\n",CMD)){  
    Serial.println("AT+CIICR => FAIL");    
  }else Serial.println("AT+CIICR => OK");
    
  //get the local ip number
  //GPRS.println("AT+CIFSR");
  if(!sim808_check_with_cmd("AT+CIFSR\r\n","OK\r\n",CMD)){  
    Serial.println("AT+CIFSR => FAIL");    
  }else Serial.println("AT+CIFSR => OK");
    
  //Get the remote IP address
  //GPRS.println("AT+CDNSGIP=\"ardugps.hopto.org\"");
  if(!sim808_check_with_cmd("AT+CDNSGIP=\"ardugps.hopto.org\"\r\n","OK\r\n",CMD)){  
    Serial.println("AT+CDNSGIP => FAIL");    
  }else Serial.println("AT+CDNSGIP => OK");  
  
 
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

  //GPRS.println("AT+CGPSINF=0");  
  //SerialSim808_Read();

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

//Only for debug purpose, send directly the AT command manually via serial interface
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

/****************************************************************************************
 * GetGPSLocation()
 * This function request the GPS information from SIM808, parse information in global 
 * variables and finally print the information at Serial interface for debugging.
 ****************************************************************************************/
void GetGPSLocation(){
char infoLocation[BUFFER_SIZE];  
  
  while(GPRS.available()){
    GPRS.read();
  } 
   GPRS.println("AT+CGPSINF=2");
   
   //Read response of SIM808 at Serial Port, Uno buffer all data without other activities
   //otherwise it may lose some bytes 
   //after send the command to GPS, it takes about 100ms to all serial information arrives UNO   
   //delay(50);
   
   posBuf=0;
   int i = 0;
   //for(int i=0;i<7;i++){
   while((posBuf<=99)&&(i<15)){ 
     while((GPRS.available()>0)){
        infoLocation[posBuf++]=GPRS.read();
     }
     delay(10);
     i++;
     //delay(15); 
   }
   //Serial.print("valor i= ");
   //Serial.println(i);

   //Copy buffered information in GPRS serial port and the last position of array.
   //T strcpy(bufferSerial,infoLocation);
   posTemp = posBuf;
   //T resetInfoLocationBuffer(); //clean the just received data in the GPRS buffer


   //exit condition for parse the GPS location
   matchLocationMessage = true;

   //parse each character of buffer to extract GPS information
   for(int i =0; i<posTemp; i++){
    if(matchLocationMessage==true){
      //T GPSAnalyzer(bufferSerial[i]);
      GPSAnalyzer(infoLocation[i]);
    }
   }

   SendGPSLocation();
}


void resetBuffer() 
{
  memset(buffer, 0, sizeof(buffer));
  //Serial.print("Valor do pos: ");
  //Serial.println(pos);
  pos = 0;
}//BASICALLY TO RESET THE BUFFER

/*void resetInfoLocationBuffer() 
{
  memset(infoLocation, 0, sizeof(buffer));
  posBuf = 0;
}//BASICALLY TO RESET THE BUFFER*/

void PrintSecondsElapsed(){
  Serial.print("******************Seconds : ");
  Serial.println(seconds);
  seconds++;
} 

/******************************************************************************************
 * SendGPSLocation()
 * This procedure open a GPRS data connection and send latitude and longitude for server.
 * After data is sent it disconnect from server.
 ******************************************************************************************/

void SendGPSLocation(){  
  //char data2db[]="410.869,111.222";
  char data2db[20];
  //start a connection to TCP to URL and port 
  //get the local ip number
  //GPRS.println("AT+CIFSR");
  if(!sim808_check_with_cmd("AT+CIFSR\r\n","OK\r\n",CMD)){  
    Serial.println("AT+CIFSR => FAIL");    
  }else Serial.println("AT+CIFSR => OK");
    
  //Get the remote IP address
  //GPRS.println("AT+CDNSGIP=\"ardugps.hopto.org\"");
/*  if(!sim808_check_with_cmd("AT+CDNSGIP=\"ardugps.hopto.org\"\r\n","OK\r\n",CMD)){  
    Serial.println("AT+CDNSGIP => FAIL");    
  }else Serial.println("AT+CDNSGIP => OK"); */ 
  //delay(3000);
  //GPRS.println("AT+CIPSTART=\"TCP\",\"ardugps.hopto.org\",6789");
  if(!sim808_check_with_cmd("AT+CIPSTART=\"TCP\",\"ardugps.hopto.org\",6789\r\n","OK\r\n",CMD)){  
    Serial.println("AT+CIPSTART => FAIL");    
  }else Serial.println("AT+CIPSTART => OK");

  delay(3000);
    
  //Send data to remote connection
  //GPRS.println("AT+CIPSEND"); 
  /*if(!sim808_check_with_cmd("AT+CIPSEND\r\n","\>",CMD)){  
    Serial.println("AT+CIPSEND => FAIL");    
  }else Serial.println("AT+CIPSEND => OK");  

  delay(1000);
  GPRS.write(data2db); 
  //sim808_send_cmd(data2db);
  //Equivalent to sending Ctrl+Z
  //GPRS.write(char(26)); 
  //delay(3000);
  //delay(500);
  sim808_send_End_Mark(); */
  /*if(!sim808_wait_for_resp("OK\r\n", CMD)){  
    Serial.println("AT+CIPSEND EndMark => FAIL");    
  }else Serial.println("AT+CIPSEND EndMark => OK");  */

  //data2db=latitude;


  memset(data2db,'\0',sizeof(data2db));
  strcpy(data2db,latitude);
  Serial.print("valor data2db1");
  Serial.println(data2db);

  
  strcat(data2db,",");
  strcat(data2db,longitude);
  Serial.print("valor data2db2");
  Serial.println(data2db);
  
  if(!SendDataCIPSEND(data2db,sizeof(data2db))){
  //if(!SendDataCIPSEND(latitude,sizeof(data2db))){ 
    Serial.println("CIPSEND => FAIL");
  }
  else{
    Serial.println("CIPSEND => OK");
  }

  delay(3000);


  if (!CheckConnectionStatus()) {
    Serial.println("Uno was not connected to Server");
  }
  else{
    sim808_check_with_cmd("AT+CIPCLOSE\r\n", "CLOSE OK\r\n", CMD);
    Serial.println("Uno was disconnected from Server");
  }
  delay(1000);
  
 
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
  if(!sim808_check_with_cmd("\r\n","\>",CMD)) {
    return 0;
  }
  /*if(0 != sim808_check_with_cmd(str,"SEND OK\r\n", DEFAULT_TIMEOUT * 10 ,DATA)) {
        return 0;
  }*/
  delay(500);
  sim808_send_cmd(str);
  delay(500);
  sim808_send_End_Mark();
  if(!sim808_wait_for_resp("SEND OK\r\n", DATA, DEFAULT_TIMEOUT * 10, DEFAULT_INTERCHAR_TIMEOUT * 10)) {
      return 0;
  }        
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
        //+CIPSTATUS: 1,0,"TCP","216.52.233.120","80","CONNECTED"
        return true;
    } else {
        //+CIPSTATUS: 1,0,"TCP","216.52.233.120","80","CLOSED"
        //+CIPSTATUS: 0,,"","","","INITIAL"
        return false;
    }
}

// =====================================================================
// Analyze the Serial information received after request the information
// AT+CCPSINF=2
// =====================================================================
//AT+CGPSINF=2, get GPS location with format
//Message ID       = 2
//UTC time         = [hhmmss.sss]
//Latitude         = [+/-[0dd.dddd]]
//N/S indicator    = [N/S]
//Longitude        = [+/-[0ddd.dddd]]
//E/W indicator    = [W/E]
//Position fix     = 1
//Satellites used  = range 0 to 12
//HDOP             = x.xx
//MSL Altitude     = xxx.x
//Units            = M
//Geoid Separation = xxx
//Units            = M
void GPSAnalyzer(byte b) {

   if ((b != ',')){
       buffer[pos++] = b;
   }
   
   if ( pos >= sizeof(buffer) )
    resetBuffer();// just to be safe
    
   //Serial.println(buffer);
   //Serial.println(state);
  
  switch (state) 
  {
  case PS_DETECT_NEW_LINE:
  {
    //if(b == '\n'){
    if(b == '+'){  
      resetBuffer();
      state = PS_DETECT_MSG_TYPE2;
    }
  }
  break;
    
  case PS_DETECT_MSG_TYPE2: 
    {
      if ((b == '\n'))
        resetBuffer();
      else {        
        if ( pos == 8 ) {
          //Serial.print("Checking message type: ");
          if ( strcmp(buffer, "UGNSINF:") ) {
            //Serial.println("Received CGNSINF:");            
            state = PS_READ_GPS_MODE;
            resetBuffer();
          }
          else{
            resetBuffer();
            state = PS_DETECT_NEW_LINE;
            matchLocationMessage = false;
          }
        }
      }
    }
    break;

//THIS WOULD READ FROM +CGNSINF: (TO THE COMMA),
  case PS_READ_GPS_MODE:
    {
      if ( b== '\n'){
        state = PS_DETECT_NEW_LINE;
        resetBuffer();
      }
      else {
        if ( b == ',' ) {
          Serial.println("************************************************************ ");        
          Serial.print("CGPSINF_MODE   : ");
          Serial.println(buffer);
          gpsMode=buffer;
          //state = PS_READ_LONGITUDE;
          state = PS_READ_TIME;
          resetBuffer();
        }
      }
    }
    break;

  case PS_READ_TIME:
  {
    if ( b== '\n'){
      state = PS_DETECT_NEW_LINE;
      resetBuffer();
    }
    else {
      if ( b == ',' ) {
        Serial.print("TIME           : ");
        Serial.println(buffer);
        dateTime=buffer;
        //state = PS_READ_TTFF;
        state = PS_READ_LATITUDE;
        resetBuffer();
      }
    }
  }
  break; 
   
  case PS_READ_LATITUDE:
    {
      if ( b== '\n'){
        state = PS_DETECT_NEW_LINE;
        resetBuffer();
      }
      else {
      
        if ( b == ',' ) {
          strcpy(latitude,buffer);
          Serial.print("LATITUDE       : ");
          Serial.println(latitude);
                    
          state = PS_READ_NORTH_SOUTH;
          resetBuffer();
        }
      }
    }
  break; 

  case PS_READ_NORTH_SOUTH:
  {
    if ( b== '\n'){
      state = PS_DETECT_NEW_LINE;
      resetBuffer();
    }
    else {
      if ( b == ',' ) {
        Serial.print("N/S indicator  : ");
        Serial.println(buffer);
        north_south=buffer;
        state = PS_READ_LONGITUDE;
        resetBuffer();
      }
    }
  }
  break; 

  case PS_READ_LONGITUDE:
    {
      if ( b== '\n'){
        state = PS_DETECT_NEW_LINE;
        resetBuffer();
      }
      else {  
        if ( b == ',' ) {
          strcpy(longitude,buffer);
          Serial.print("LONGITUDE      : ");
          Serial.println(longitude);
          
         
  
          state = PS_READ_EAST_WEST;
          resetBuffer();
        }
      }
    }
    break;



  case PS_READ_EAST_WEST:
  {
      if ( b== '\n'){
        state = PS_DETECT_NEW_LINE;
        resetBuffer();
      }
      else {
        if ( b == ',' ) {
          Serial.print("E/W indicator  : ");
          Serial.println(buffer);
          east_west=buffer;
          state = PS_READ_POSITION_FIX;
          resetBuffer();
        }
      }
    }
    break;



   case PS_READ_POSITION_FIX:
   {
      if ( b== '\n'){
        state = PS_DETECT_NEW_LINE;
        resetBuffer();
      }
      else {
        if ( b == ',' ) {
          Serial.print("POSITION FIX   : ");
          Serial.println(buffer);
          positionFix=buffer; 
          state = PS_READ_NUM_SAT;
          resetBuffer();
          //delay(500); don't do this!
        }
      }
    }
    break;

    case PS_READ_NUM_SAT:
    {
      if ( b== '\n'){
        state = PS_DETECT_NEW_LINE;
        resetBuffer();
      }
      else {   
       if ( b == ',' ) {
          Serial.print("NUM SATELLITE  : ");
          Serial.println(buffer);
          numSatelite=buffer;
          state =PS_READ_HDOP;
          resetBuffer();
  
        }
      }
    }
    break;

    case PS_READ_HDOP:
    {
      if ( b== '\n'){
        state = PS_DETECT_NEW_LINE;
        resetBuffer();
      }
      else {   
       if ( b == ',' ) {
          Serial.print("HDOP           : ");
          Serial.println(buffer);
          hdop=buffer;
          state =PS_READ_ALTITUDE;
          resetBuffer();
  
        }
      }
    }
    break;

    case PS_READ_ALTITUDE:
    {
      if ( b== '\n'){
        state = PS_DETECT_NEW_LINE;
        resetBuffer();
      }
      else {   
       if ( b == ',' ) {
          Serial.print("ALTITUDE       : ");
          Serial.println(buffer);
          altitude=buffer;
          state =PS_READ_ALT_UNIT;
          resetBuffer();
  
        }
      }
    }
    break;
    
    case PS_READ_ALT_UNIT:
    {
      if ( b== '\n'){
        state = PS_DETECT_NEW_LINE;
        resetBuffer();
      }
      else {   
       if ( b == ',' ) {
          Serial.print("ALTITUDE unit  : ");
          Serial.println(buffer);
          altitudeUnit=buffer;
          state =PS_READ_GEOID_SEPARATION;
          resetBuffer();
  
        }
      }
    }
    break;

    case PS_READ_GEOID_SEPARATION:
    {
      if ( b== '\n'){
        state = PS_DETECT_NEW_LINE;
        resetBuffer();
      }
      else {   
       if ( b == ',' ) {
          Serial.print("GEOID SEP.     : ");
          Serial.println(buffer);
          geoid=buffer;
          state =PS_READ_GEOID_UNIT;
          resetBuffer();
  
        }
      }
    }
    break;


    case PS_READ_GEOID_UNIT:
    {
      if ( b== '\n'){
        state = PS_DETECT_NEW_LINE;
        resetBuffer();
      }
      else {   
       if ( b == ',' ) {
          Serial.print("GEOID SEP. unit: ");
          Serial.println(buffer);
          geoid_unit=buffer;
          state =PS_DETECT_NEW_LINE;
          resetBuffer();
          matchLocationMessage = false;
  
        }
      }
    }
    break;
  
  //use goto to put it at sms begining
  }
  //return;
 }  



