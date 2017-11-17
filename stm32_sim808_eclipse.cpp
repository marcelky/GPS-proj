// Do not remove the include below

/*********************************************************PIN # ***/
//    {&gpioa, &timer2, &adc1,  0, 1,    0}, /* PA0  */    0
//    {&gpioa, &timer2, &adc1,  1, 2,    1}, /* PA1  */    1
//    {&gpioa, &timer2, &adc1,  2, 3,    2}, /* PA2  */    2
//    {&gpioa, &timer2, &adc1,  3, 4,    3}, /* PA3  */    3
//    {&gpioa,   NULL, &adc1,  4, 0,    4},  /* PA4  */    4
//    {&gpioa,   NULL, &adc1,  5, 0,    5},  /* PA5  */    5
//    {&gpioa, &timer3, &adc1,  6, 1,    6}, /* PA6  */    6
//    {&gpioa, &timer3, &adc1,  7, 2,    7}, /* PA7  */    7
//    {&gpioa, &timer1, NULL,  8, 1, ADCx},  /* PA8  */    8
//    {&gpioa, &timer1, NULL,  9, 2, ADCx},  /* PA9  */    9
//    {&gpioa, &timer1, NULL, 10, 3, ADCx},  /* PA10 */    10
//    {&gpioa, &timer1, NULL, 11, 4, ADCx},  /* PA11 */    11
//    {&gpioa,   NULL, NULL, 12, 0, ADCx},   /* PA12 */    12
//    {&gpioa,   NULL, NULL, 13, 0, ADCx},   /* PA13 */    13
//    {&gpioa,   NULL, NULL, 14, 0, ADCx},   /* PA14 */    14
//    {&gpioa,   NULL, NULL, 15, 0, ADCx},   /* PA15 */    15
//    {&gpiob, &timer3, &adc1,  0, 3,    8}, /* PB0  */    16
//    {&gpiob, &timer3, &adc1,  1, 4,    9}, /* PB1  */    17
//    {&gpiob,   NULL, NULL,  2, 0, ADCx},   /* PB2  */    18
//    {&gpiob,   NULL, NULL,  3, 0, ADCx},   /* PB3  */    19
//    {&gpiob,   NULL, NULL,  4, 0, ADCx},   /* PB4  */    20
//    {&gpiob,   NULL, NULL,  5, 0, ADCx},   /* PB5  */    21
//    {&gpiob, &timer4, NULL,  6, 1, ADCx},  /* PB6  */    22
//    {&gpiob, &timer4, NULL,  7, 2, ADCx},  /* PB7  */    23
//    {&gpiob, &timer4, NULL,  8, 3, ADCx},  /* PB8  */    24
//    {&gpiob, &timer4, NULL,  9, 4, ADCx},  /* PB9  */    25
//    {&gpiob,   NULL, NULL, 10, 0, ADCx},   /* PB10 */    26
//    {&gpiob,   NULL, NULL, 11, 0, ADCx},   /* PB11 */    27
//    {&gpiob,   NULL, NULL, 12, 0, ADCx},   /* PB12 */    28
//    {&gpiob,   NULL, NULL, 13, 0, ADCx},   /* PB13 */    29
//    {&gpiob,   NULL, NULL, 14, 0, ADCx},   /* PB14 */    30
//    {&gpiob,   NULL, NULL, 15, 0, ADCx},   /* PB15 */    31
//    {&gpioc,   NULL, NULL, 13, 0, ADCx},   /* PC13 */    32
//    {&gpioc,   NULL, NULL, 14, 0, ADCx},   /* PC14 */    33
//    {&gpioc,   NULL, NULL, 15, 0, ADCx},   /* PC15 */    34
/*******************************************************************/

#include "stm32_sim808_eclipse.h"
#include <stdlib.h>

#define DEBUG 1
//#define NO_NETLIGHT 1   //turn on or off netlight of sim808 board
#define PROTOCOL 2        //1 = TCP and 2 = UDP

#ifdef PROTOCOL
  static int const prot = PROTOCOL;
#else
  static int const prot = 1;          // default value TCP
#endif

#define Powerkey 28      //pin PB12 Power pin for SIM808
//int DTRkey=8;          //pin 8 of arduino will be connected to DTR of SIM808 pin (9)
//#define rxPin 27       //pin PB11
//#define txPin 26       //pin PB10

fault_type currentError; //if value 0, there is no error, otherwise check the "Definition of error codes"

const int ERROR_SIZE = 25;
int errorStatus[ERROR_SIZE]={0};

// start reading from the first byte (address 0) of the EEPROM
int addWrite = 0; //write
int addRead = 0;  //position to read from EEPROM

//The content of messages sent
//#define MESSAGE  "hello,world"
#define MESSAGE_LENGTH 30
char message[MESSAGE_LENGTH];

char phone[16];
char datetime[24];
char gprsBuffer[64];
char *s = NULL;
char smsMessage[64];

//SoftwareSerial GPRS(rxPin, txPin); // RX, TX
//DFRobot_SIM808 sim808(&GPRS);
int timerID;

//All this char variable have reserved one extra char to insert \0 at end
char gpsMode[4]="000";
char dateTime[11]="000000.000";
char latitude[10]="0000.0000";
char north_south[2]="N";
char longitude[11]="00000.0000";
char east_west[2]="W";
char MSL_altitude[8]="00000.0";
char Speed[8]="000.000";
char GEOID[5]="\0";
char startDateTime[22] = "000000000000000000000";//"hhmmss.sss-dd-mm-yyyy"; start time of HW

boolean DNS_OK = true;
int currentRadioSig = 10;   //currentRadioSig/previousRadioSig = 10 to avoid configure again the data connection
int previousRadioSig = 10;  //when enter the first time of loop to send data
//USED FOR EEPROM
int addr = 0;

//time to GPS acquire signal
int timeGetGPS=0;  //in seconds
const int BUFFER_SIZE = 90;

char buffer[20];                 // WHAT WE ARE READING INTO
int posBuf;
int seconds=0;

char celularDateTime[22]={0};

//Used inside the timer
volatile boolean triggerGetGPSLocation = true;   //variable changed inside the timer(2), so need to be volatile
                                                 //it is the main loop to on/off the getGPSLocation()
volatile boolean triggerConnectGPS = true;

volatile boolean Tmachine1 = true;
volatile boolean Tmachine2 = true;
volatile boolean Tmachine3 = true;
volatile boolean Tmachine4 = true;



HardwareTimer timer_GetGPSLocation(2);
HardwareTimer timer_ConnectGPS(3);
HardwareTimer timer_GPS_machine(4);

int timeStartGetGPSLocation = 0;
int timeExecuteGetGPSLocation = 0;

DFRobot_SIM808_stm32 sim808(&Serial2);

/*******************************************************************************************
 * function setup()
 * This function run once at start up of processor, here is made the configuration of
 * UNO and SIM808.
 *******************************************************************************************/
void setup() {

/************************************
 * Define Serial parameters
 ************************************/
  Serial.begin(9600);
  Serial2.begin(9600);
  sim808_init(&Serial2,9600);
//  DFRobot_SIM808_stm32 sim808(&Serial2);
/*************************************
 * Power on the module SIM808
 *************************************/
  pinMode(Powerkey,OUTPUT);
  PowerSIM808();

/*************************************
 * Check start sequence of SIM808
 *************************************/
  Serial.println("Starting.");
  //loop to wait serial become available
  while(!Serial2.available()){
    Serial.print(".");
    delay(1000);
  }

  if((sim808_init_check())==false) {
    Serial.println();
    Serial.println("Initialization Failed.");
    printAllError();
  }else{
    Serial.println();
    Serial.println("Initialization Check Successful !!");
  }
  /*****************************************************
  * Enable charging battery
  ****************************************************/
  //sim808_send_cmd("AT+ECHARGE=1");


   /******************************************************
   * GPRS Configuration
   ******************************************************/
  //delay(5000);//try to save battery burst
  //GNSS power control
  sim808_check_with_cmd("AT+CGNSPWR=1","OK\r\n",CMD);
  //delay(5000);

  //GNSS navitation, GEO-Fence and speed alarm URC report control
  sim808_check_with_cmd("AT+CGNSURC=0","OK\r\n",CMD);
  //delay(5000);

  //GSM and GPRS configuration
  //check which network GPRS module is connected with
  //GPRS.println("AT+COPS?");

  //check if mobile number is register in the network
  //GPRS.println("AT+CREG=?");

  /*******************************************************
   * configuration of GPRS data
   *******************************************************/
  currentError=InitGPRS(FIRST_TIME);
  if(currentError != NO_ERROR){
    Serial.println("Error: ");
    Serial.println(currentError);
  }

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


  /***********************************************************
  * GPS configuration
  ***********************************************************/
  //connect_GPS(240, FIRST_TIME);  //wait GPS connect 2minutes

  if(!GPSAnalyzer128()){ // Get current time from GPS
    setCurrentError(INITIAL_TIME_NOT_SET);
    printAllError();
  }else{
    Serial.println(startDateTime);
  }


/*****************************************************************************************
 *Configure SIM808 to power save mode, AT+CSCLK.
 *It can exit sleep mode  Configure SIM808
 * Pull down DTR
 * Receive voice or data call from network
 * Receive SMS from network
 * Receive external interrupt
 * Charge VBUS pin
 * AT+CSCLK=0 disable slow clock
 *          1 enable slow clock controlled by DTR. when DTR=1 can enter sleep mode
 *                                                 when DTR=0 exit sleep mode
 *****************************************************************************************/
  //configure DTR pin to normal HIGH
//  pinMode(DTRkey, OUTPUT);
//  digitalWrite(DTRkey, LOW);      //can not enter sleep mode

  //sim808_check_with_cmd("AT+CSCLK=1\r\n","OK\r\n",CMD);  //activate the sleep mode

  //GetGPSLocation();

  /*************************************************************************************************
   * Configure timer 6 to fire up the execution of GetGPSLocation()
   *************************************************************************************************/

  timer_GetGPSLocation.pause();
  timer_GetGPSLocation.setPeriod(30000000);                           // Set up periodin microseconds
  timer_GetGPSLocation.setChannel1Mode(TIMER_OUTPUT_COMPARE);         // Set up an interrupt on channel 1
  timer_GetGPSLocation.setCompare(TIMER_CH1, 1);                      // Interrupt 1 count after each update
  timer_GetGPSLocation.attachCompare1Interrupt(handlerGetGPSLocation);
  timer_GetGPSLocation.refresh();                                     // Refresh the timer's count, prescale, and overflow
  //timer_GetGPSLocation.resume();                                      // Start the timer counting

  timer_ConnectGPS.pause();
  timer_ConnectGPS.setPeriod(30000000);                       		// Set up periodin microseconds
  timer_ConnectGPS.setChannel1Mode(TIMER_OUTPUT_COMPARE);     		// Set up an interrupt on channel 1
  timer_ConnectGPS.setCompare(TIMER_CH1, 1);                  		// Interrupt 1 count after each update
  timer_ConnectGPS.attachCompare1Interrupt(handlerConnectGPS);
  timer_ConnectGPS.refresh();                                 		// Refresh the timer's count, prescale, and overflow
  //timer_ConnectGPS.resume();                                  		// Start the timer counting

  //TODO create timer to set clock
  timer_GPS_machine.pause();
  //timer_GPS_machine.setPeriod(30000000);    // Set up periodin microseconds
  timer_GPS_machine.setOverflow(60000);     // max time 30s
  timer_GPS_machine.setPrescaleFactor(36000);  //72000000/36000 = 2000 Hz new frequency after pre-scaler

  timer_GPS_machine.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  timer_GPS_machine.setChannel2Mode(TIMER_OUTPUT_COMPARE);
  timer_GPS_machine.setChannel3Mode(TIMER_OUTPUT_COMPARE);
  timer_GPS_machine.setChannel4Mode(TIMER_OUTPUT_COMPARE);


  timer_GPS_machine.setCompare(TIMER_CH1,1);
  timer_GPS_machine.setCompare(TIMER_CH2,10000); //2000 * 5sec = 10000
  timer_GPS_machine.setCompare(TIMER_CH3,20000); //2000 * 10sec = 20000
  timer_GPS_machine.setCompare(TIMER_CH4,40000); //2000 * 20sec = 40000

  timer_GPS_machine.attachInterrupt(TIMER_CH1,ClockSeconds);
  timer_GPS_machine.attachInterrupt(TIMER_CH2,handlerMachine2);
  timer_GPS_machine.attachInterrupt(TIMER_CH3,handlerMachine3);
  timer_GPS_machine.attachInterrupt(TIMER_CH4,handlerMachine4);

  timer_GPS_machine.refresh();                                 		// Refresh the timer's count, prescale, and overflow
  timer_GPS_machine.resume();                                  		// Start the timer counting

  //TODO criar timers para fazer fech the estado do GPS


  //TODO criar relogios para fazer o chaveamento do GPS

}



/***********************************************************************************
 * Main Loop, this funcion runs over and over again forever
 ***********************************************************************************/
void loop() {
//  digitalWrite(PC13, HIGH);   // turn the LED on (HIGH is the voltage level)
//  delay(1000);                       // wait for a second
//  digitalWrite(PC13, LOW);    // turn the LED off by making the voltage LOW
//  delay(1000);                       // wait for a second

  //Only for debug purpose, send directly the AT command manually via serial interface
  #ifdef DEBUG
  if(Serial2.available())
    Serial.write(Serial2.read());
  if(Serial.available())
    Serial2.write(Serial.read());
  #endif

  /***********************************************
   * Main code to get GPS information
   ***********************************************/


//    if(sim808.getDateTime(celularDateTime)){
//      Serial.println();
//      Serial.println();
//      Serial.println(celularDateTime);
//
//      Serial.print("triggerGetGPSLocation: ");
//      Serial.println(triggerGetGPSLocation);
//
//      Serial.print("triggerConnectGPS: ");
//      Serial.println(triggerConnectGPS);
//
//      Serial.print("timer_GetGPSLocation: ");
//      Serial.println(timer_GetGPSLocation.getCount());
//
//      Serial.print("timer_ConnectGPS: ");
//      Serial.println(timer_ConnectGPS.getCount());
//    }


  //TODO check period of timer no serial monitor, pois nao esta aparecendo nada do machine1 2 3 4
  if (Tmachine1 == true){
	Tmachine1 = false;
	Serial.print("Machine1: ");
	//Serial.println(timer_GPS_machine.getCount());
  }

  if (Tmachine2 == true){
	Tmachine2 = false;
	Serial.print("Machine2: ");
	//Serial.println(timer_GPS_machine.getCount());
  }

  if (Tmachine3 == true){
	Tmachine3 = false;
	Serial.print("Machine3: ");
	//Serial.println(timer_GPS_machine.getCount());
  }

  if (Tmachine4 == true){
	Tmachine4 = false;
	Serial.print("Machine4: ");
	//Serial.println(timer_GPS_machine.getCount());
  }





  if (triggerGetGPSLocation==true){
    timeStartGetGPSLocation = millis();
    timer_GetGPSLocation.pause();

    GetGPSLocation();
    triggerGetGPSLocation = false;
    timer_GetGPSLocation.resume();

    timeExecuteGetGPSLocation = ((unsigned long) (millis() - timeStartGetGPSLocation))/1000UL;

    Serial.print("timeExecuteGetGPSLocation: ");
    Serial.println(timeExecuteGetGPSLocation);
  }
  currentError = (fault_type)readCurrentError(GPS_SYNC_FAIL);
  if ((currentError == 1) && (triggerConnectGPS==true)){
    timer_GetGPSLocation.pause();                             //timer_GetGPSLocation getGPSLocation disabled
    triggerGetGPSLocation = false;                            //it only toggle to true inside the handler3() with timer(2) enabled
    Serial.println("GetGPSDisabled");

    if(connect_GPS(120, RUNNING_TIME)){    //leave GPS ON/COLD START until timeout of 120 sec the function connect_GPS disable alarm GPS_SYNC_FAIL if sync
      timer_GetGPSLocation.resume();                          //timer getGPSLocation enabled
      timer_ConnectGPS.pause();                               //timer connectGPS disabled, this routine only run again here in case of GPSsync timeout
      Serial.println("GetGPSEnabled");
    }else{                                                    //still fault GPSsync so prepare to run again next cyclo
      timer_ConnectGPS.refresh();                             //timer connectGPS restart/enabled
      timer_ConnectGPS.resume();
      Serial.println("GetGPSDisabled");
      triggerConnectGPS=false;                                //trigger_connectGPS will only be activate again in handlerConnectGPS
    }
    sim808_check_with_cmd("AT+CGPSPWR=0\r\n","OK\r\n",CMD);   //put GPS to sleep either have acquired signal or not
  }
}

/****************************************************************************************
 * power()
 * Function power on or power of SIM808.
 ****************************************************************************************/
void PowerSIM808(){
  digitalWrite(Powerkey, LOW);
  delay(1000);               // wait for 1 second
  digitalWrite(Powerkey, HIGH);
}

/***************************************************************************************
 * This function SET/UNSET the trigger of function getGPSLocation in the main loop
 * It is called from timer(1) function.
 * It is necessary because the delay() or millis() function does not work if called
 * directly from the timer(1) function. This is a limitation of timer functionality.
 ***************************************************************************************/
void handlerGetGPSLocation(){
  if (triggerGetGPSLocation == true){
    triggerGetGPSLocation = false;
    digitalWrite(PC13, HIGH);
  }else{
    triggerGetGPSLocation = true;
    digitalWrite(PC13, LOW);
  }
}

/***************************************************************************************
 * This function SET/UNSET the trigger of function connectGPS in the main loop
 * It is called from timer(2) function.
 * It is necessary because the delay() or millis() function does not work if called
 * directly from the timer(2) function. This is a limitation of timer functionality.
 ***************************************************************************************/
void handlerConnectGPS(){
  if (triggerConnectGPS == true){
    triggerConnectGPS = false;
  }else{
    triggerConnectGPS = true;
  }
}

/****************************************************************************************
 * Create handler for GPS machine
 ****************************************************************************************/
void ClockSeconds(){
	Serial.print("Machine1: ");
	Tmachine1 = true;
	//Serial.println(timer_GPS_machine.getCount());
}

void handlerMachine2(){
	Serial.print("Machine2: ");
	Tmachine2 = true;
	//Serial.println(timer_GPS_machine.getCount());
}

void handlerMachine3(){
	Serial.print("Machine3: ");
	Tmachine3 = true;
	//Serial.println(timer_GPS_machine.getCount());
}

void handlerMachine4(){
	Serial.print("Machine4: ");
	Tmachine4 = true;
	//Serial.println(timer_GPS_machine.getCount());
}

//TODO criar rotina para fazer single status of GPS



/****************************************************************************************
 * sim808_init_check
 * Check if initialization of SIM808 was successfull
 * -Check AT command
 * -Check CFUN=1 (full functionality, SMS and voice are working)
 * -Check CPIN, if SIM card is operational
 *
 *  Normal messages during start of SIM808
 *  NORMAL POWER DOWN
 *  RDY
 *  +CFUN: 1
 *  +CPIN: READY
 *  Call Ready
 *  SMS Ready
 ****************************************************************************************/
boolean sim808_init_check(){
boolean checkOK=true;
byte i;

  for(i=0;i<10; i++){
    if(!sim808_check_with_cmd("AT\r\n","OK\r\n",CMD)){
      if(i==9){
        setCurrentError(SIM808_SERIAL_FAILED);
        return false;
      }
//      Serial.print("AT_F:");
//      Serial.println(i);
      //delay(1000);
    }else{
//      Serial.print("AT_OK:");
//      Serial.println(i);
      break;
    }
  }


  sim808_clean_buffer(gprsBuffer,64);
  sim808_flush_serial();
  sim808_send_cmd("AT+CFUN?\r\n");
  sim808_read_buffer(gprsBuffer,25,30);
  if(!(strstr(gprsBuffer,"+CFUN: 1"))){
    setCurrentError(CFUN1_FAILED);
    checkOK = false;
  }
//  Serial.print("Buffer01: ");
//  Serial.println(gprsBuffer);
//  Serial.print("**********");


  for(i=0;i<10; i++){
    sim808_clean_buffer(gprsBuffer,64);
    sim808_flush_serial();
    sim808_send_cmd("AT+CPIN?\r\n");
    sim808_read_buffer(gprsBuffer,25,30);
    if(!(strstr(gprsBuffer,"+CPIN: READY"))){
      Serial.print("CPIN_F:");
      Serial.println(i);
      if(i==9) {
        setCurrentError(CPIN_SIMCARD_FAILED);
        checkOK = false;
        break;
      }
      delay(1000);
    }else{
        Serial.print("CPIN_OK:");
        Serial.println(i);
        break;
    }
  }

//  Serial.print("Buffer02: ");
//  Serial.println(gprsBuffer);
//  Serial.print("**********");


  return checkOK;
}



/**************************************************************************************
* InitGPRS(byte firstTime)
* This function initialize the GPRS during start/restart of arduino/sim808 or when
* during normal operation it lost connection signal with GPRS.
* param: firstTime = 1, when is called during setup()
*        firstTime = 0, when called inside the code
* Return error code: see error definition
***************************************************************************************/
fault_type InitGPRS(initTime_type t){
 fault_type error = NO_ERROR;
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

   sim808_check_with_cmd("AT\r\n","OK\r\n",CMD);

   if(t==FIRST_TIME){
     if(!sim808_check_with_cmd("AT+CIPSHUT\r\n","OK\r\n",CMD)){
       error = CIPSHUT_FAIL;
       setCurrentError(CIPSHUT_FAIL);
     }else{
       clearCurrentError(CIPSHUT_FAIL);
     }
     delay(50);
   }

  /***********************************
   *Set GPRS data rate
   *AT+IPR=9600
   *OK
   ***********************************/
   if(!sim808_check_with_cmd("AT+IPR=9600\r\n","OK\r\n",CMD)){
     error = IPR9600_FAIL;
     setCurrentError(IPR9600_FAIL);
     //Serial.println("IPR9600_FAIL");
   }else{
     clearCurrentError(IPR9600_FAIL);
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
    setCurrentError(CSTT_UNINET_FAIL);
    //Serial.println("CSTT_UNINET_FAIL");
  }else{
    clearCurrentError(CSTT_UNINET_FAIL);
  }
  delay(50);

  /*******************************************
   *bring up the wireless connection with GPRS
   *AT+CIICR
   *OK
   *******************************************/
  if(!sim808_check_with_cmd("AT+CIICR\r\n","OK\r\n",CMD)){
    error = CIICR_FAIL;
    setCurrentError(CIICR_FAIL);
  }else{
    clearCurrentError(CIICR_FAIL);
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
    setCurrentError(CDNSGIP_FAIL);
  }else{
    clearCurrentError(CDNSGIP_FAIL);
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
//      Serial.print("ph#: ");
//      Serial.println(phone);
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
//      Serial.print("ph#: ");
//      Serial.println(phone);
   }else return false;

   return true;

}

/*************************************************************************************************
*  getAllButFirstAndLast(const char *input, char *output)
*  Ths procedure remove the first and last character of char array
*  "12938484848"
*  for exemplo for a phone number
************************************************************************************************/
void getAllButFirstAndLast(const char *input, char *output){
  int len = strlen(input);
  if(len > 0)
    strcpy(output, ++input);
  if(len > 1)
    output[len - 2] = '\0';
}



/****************************************************************************************
 * DTR_low50ms()
 * Function power on or power of SIM808.
 ****************************************************************************************/
//void DTR_low50ms(void)
//{
//  digitalWrite(DTRkey, LOW);
//  delay(50);               // wait for 1 second
//  digitalWrite(DTRkey, HIGH);
//}


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


//  digitalWrite(DTRkey, LOW);  //exit sleep mode
//  delay(50);                  //time to exit sleep mode
//
  //sim808_send_cmd("AT+CFUN=1\r\n");


  //readAllError();  //print all errors present in the previous loop

  //  //waking up GPS after sleep
  //if(!connect_GPS(25, RUNNING_TIME)){ //timeout of 25 seconds to acquire signal, 1h worst case lost 2x

  Serial.println("*******************LOOP********************");

  //timer.pause(); //pause interruption of this function while executing it.

  if(!connect_GPS(30, RUNNING_TIME)){ //timeout of 25 seconds to acquire signal, 1h worst case lost 2x
    sim808_check_with_cmd("AT+CGPSPWR=0\r\n","OK\r\n",CMD);
    setCurrentError(GPS_SYNC_FAIL);
    printAllError();

    //timer.resume(); //resume interruption of this function.
    return; //   GPS_SYNC_FAIL;
  }



//  sim808_clean_buffer(infoLocation,BUFFER_SIZE);
//  sim808_send_cmd("AT+CGPSINF=2\r\n");
//  sim808_read_buffer(infoLocation,BUFFER_SIZE,DEFAULT_TIMEOUT);

//Routine to try to avoid error of NO_GPS_DATA_AVAILABLE, try to fetch 3x times
  for(int i=0;i<3; i++){
    sim808_clean_buffer(infoLocation,BUFFER_SIZE);
    sim808_flush_serial();
    sim808_send_cmd("AT+CGPSINF=2\r\n");
    sim808_read_buffer(infoLocation,BUFFER_SIZE,DEFAULT_TIMEOUT);
    if(!(strstr(infoLocation,"+CGPSINF: 2")) || (strstr(infoLocation,"0000.0000,N,00000.0000,E"))){
      Serial.print("NO GPS DATA: ");
      Serial.println(i);
      Serial.println(infoLocation);
      if(i==2) {
        setCurrentError(NO_GPS_DATA_AVAILABLE);
        printAllError();
        return;
      }
      delay(1000);
    }else{
        Serial.print("GPS DATA_OK: ");
        Serial.println(i);
        Serial.println(infoLocation);
        break;
    }
  }

//  Serial.println("buffer2:");
//  Serial.println(infoLocation);


  if(!GPSAnalyzer(infoLocation)){                   //power off GPS and wait another loop of getGPSLocation to turn GPS on again
    //sim808_clean_buffer(infoLocation,BUFFER_SIZE);
    sim808_check_with_cmd("AT+CGPSPWR=0\r\n","OK\r\n",CMD);

    setCurrentError(NO_GPS_DATA_AVAILABLE);
    printAllError();
    //Serial.println("GPSAnalyzer failed");
    //timer.resume(); //resume interruption of this function.
    return; // NO_GPS_DATA_AVAILABLE;

  }

//


  //Send collected data to mysql server



  currentError = SendGPSLocation();  //error already saved inside the SendGPSLocation()

  Serial.print("ERROR:");
  Serial.println(currentError);
  //createSMSMessage(smsMessage,latitude, longitude, MSL_altitude, Speed);

  //put GPS to sleep
  sim808_check_with_cmd("AT+CGPSPWR=0\r\n","OK\r\n",CMD);

  //test if make difference1
  sim808_check_with_cmd("AT+CGNSPWR=0\r\n", "OK\r\n", CMD);

  //delay(5000);


  printAllError();

  //sim808_send_cmd("AT+CFUN=4\r\n");
//  Serial.println("DTR high");
//  digitalWrite(DTRkey, HIGH); //can enter sleep mode

  //timer.resume(); //resume interruption of this function.
  return; // 0; //success

}


/****************************************************************************
 * setCurrentError(enum fault)
 * param
 * error = is the error code (position in the array)
 *  ****************************************************************************/
 void setCurrentError(fault_type currentError){
   errorStatus[NO_ERROR]=1;
   errorStatus[currentError]=1;
}

/****************************************************************************
 * clearCurrentError(enum fault)
 * param
 * error = is the error code (position in the array)
 *  ****************************************************************************/
 void clearCurrentError(fault_type error){
   errorStatus[error]=0;
}

/****************************************************************************
 * clearAllError()
 * param
 * error = is the error code (position in the array)
 *  ****************************************************************************/
 void clearAllError(){

  //memset(errorStatus, 0, ERROR_SIZE);
  for(int i=0; i<ERROR_SIZE;i++){
    errorStatus[i]=0;
  }
}

/****************************************************************************
 * readErrorStatus(enum fault)
 * readu current error
 ****************************************************************************/
 int readCurrentError(fault_type error){
   return errorStatus[error];
 }

/****************************************************************************
 * printAllError()
 ****************************************************************************/
void printAllError(){
   int i;
   boolean first = true;
//   addWrite = EEPROMReadInt(0);               //position 0 always have the last position to write data
//   addWrite=storeString(addWrite,"L");   //store letter 'L' of Loop to indicate init of one loop


   Serial.print("ALARM: ");
   for (i=1; i<ERROR_SIZE;i++){ //skip the position 0 where only indicate there is fault no indication where
     if(errorStatus[i]==1){
       if(first){
         first=false;
       }else{
         Serial.print(",");
       }
       Serial.print(i);
     }
   }
   return;
//   Serial.println();
//   Serial.println();

//   addWrite=storeString(addWrite,"S");   //store letter 'S' GPRS signal to indicate level of signal 0-99
//   addWrite=EEPROMWriteInt(addWrite,currentRadioSig);
//
//   addWrite=storeString(addWrite,"G");              //store letter 'G' to indicate time to acquire signal from GPS after cold/hot start
//   addWrite=EEPROMWriteInt(addWrite,timeGetGPS);
//
//   EEPROMWriteInt(0,addWrite);          //Write last position of memory to write
 }

///****************************************************************************
// * storeLog(byte c )
// * UNO memory 1K
// ****************************************************************************/
// void storeData(byte val){
//  EEPROM.write(addr, val);
//  addr = addr + 1;
//  if (addr == EEPROM.length()) {
//    addr = 0;
//  }
// }


/****************************************************************************
 * boolean connect_GPS(int timeout, int interval)
 * wait GPS acquire signal
 * parameters
 * timeout = max time to check if gps acquired in seconds
 * interval = time between the check of GPSSTATUS command
 * return
 * true or false. true if GPS acquired or false if GPS was not acquired
 ****************************************************************************/
boolean connect_GPS(int timeout, initTime_type t){
  unsigned long timerStart;
  timerStart = millis();
  int interval =5000; //interval to send command AT+CGPSSTATUS?

  //sim808_flush_serial(); //test
  Serial.print("GPS SYNC: ");
  //sim808_check_with_cmd("AT\r\n","OK\r\n",CMD);

  //test if make difference2
  sim808_check_with_cmd("AT+CGNSPWR=1\r\n", "OK\r\n", CMD);
  //delay(1000);


  if(!sim808_check_with_cmd("AT+CGPSPWR=1\r\n","OK\r\n",CMD)){
    setCurrentError(POWER_GPS_FAIL);
    return false;
  }else{
    clearCurrentError(POWER_GPS_FAIL);
  }
  //delay(1000);

  if (t == FIRST_TIME){
    if(!sim808_check_with_cmd("AT+CGPSRST=0\r\n","OK\r\n",CMD)){   //COLD START
      setCurrentError(RESTART_GPS_FAIL);
      return false;
    }else{
      clearCurrentError(RESTART_GPS_FAIL);
      interval = 10000; //interval of 10 seconds to send AT+CGPSSTATUS?

    }
  }else{
    if(t == RUNNING_TIME){
      if(!sim808_check_with_cmd("AT+CGPSRST=1\r\n","OK\r\n",CMD)){ //HOT START
        setCurrentError(RESTART_GPS_FAIL);
        return false;
      }else{
        clearCurrentError(RESTART_GPS_FAIL);
        interval = 5000; //interval of 10 seconds to send AT+CGPSSTATUS?

      }
    }
  }
  //delay(1000);

  while (1){
    sim808_clean_buffer(gprsBuffer,64);
    sim808_send_cmd("AT+CGPSSTATUS?\r\n");
    sim808_read_buffer(gprsBuffer,64,2); //change default timeout to 1s instead of 5s

    //Serial.println(gprsBuffer);

    if(strstr(gprsBuffer,"3D Fix")){
      sim808_clean_buffer(gprsBuffer,64);
      clearCurrentError(GPS_SYNC_FAIL);

      timeGetGPS = ((unsigned long) (millis() - timerStart))/1000UL;
      Serial.println(timeGetGPS);
      return true;
     }

    if ((unsigned long) (millis() - timerStart) > timeout * 1000UL) {
      sim808_clean_buffer(gprsBuffer,64);
      setCurrentError(GPS_SYNC_FAIL);
      Serial.print(((unsigned long) (millis() - timerStart))/1000UL);
      Serial.println(" (timeout)");
      return false;
    }

    delay(interval);
  }
  return false;
 }


/*******************************************
 * Print dot on serial port to indicate
 * elapsed time.
 *******************************************/

void PrintSecondsElapsed(){
//  Serial.print("*");
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

fault_type SendGPSLocation(){
  //char data2db[36]="0000.0000,00000.0000,00000.0,000.000"; //"latitude,longitude,MSL_altitude,Speed"
  char data2db[36]="\0"; //"latitude,longitude,MSL_altitude,Speed"
  fault_type error;
  int sendNumber;

  /************************************************************
   * CIPSTATUS result
   * 0 IP STATUS    ready to initiate connection
   * 8 CONNECT OK   ready to send data
   ************************************************************/


//    Serial.print("Buffer2: ");
//    Serial.print(gprsBuffer);
  //get signal strength, if < 7 not possible to send data to server
  sim808_clean_buffer(gprsBuffer,64);
  sim808_send_cmd("AT+CSQ\r\n");
//  delay(100);
//  if(Serial2.available()) {
    sim808_read_buffer(gprsBuffer,64);  //3rd timeout of 1second
    currentRadioSig = getSignalQuality(gprsBuffer);
    Serial.print("SIG:");
    Serial.println(currentRadioSig);
//    Serial.print("Buffer3: ");
//    Serial.print(gprsBuffer);
//  }
  sim808_clean_buffer(gprsBuffer,64);


/*******************************************************************
 * previousRadioSignal currentRadioSignal
 * 0                   0                  nao faz nada e sai
 * 1                   0                  nao faz nada e sai
 * 0                   1                  initGPRS e send data
 * 1                   1                  nao faz nada e send data
 *******************************************************************/
//  //sim808_send_cmd("AT+CIPSTATUS\r\n");
//  if(Serial2.available()) {
//    sim808_send_cmd("AT+CIPSTATUS\r\n");
//    sim808_read_buffer(gprsBuffer,64,DEFAULT_TIMEOUT);
//    //Serial.print(gprsBuffer);
//  }
  if((currentRadioSig <= 7) || (currentRadioSig == 99)) {  // (smaller than 7) signal too weak and (equall 99) could not measure radio signal
       Serial.println("NOGPRSsig");
       //sim808_send_cmd("AT+CDNSGIP=\"ardugps.hopto.org\"\r\n");
       previousRadioSig = currentRadioSig;

       setCurrentError(NO_GPRS_SIGNAL);
       return NO_GPRS_SIGNAL;            //error of weak signal or DNS failed
  }else{                                 //radio signal > 7
    clearCurrentError(NO_GPRS_SIGNAL);
    if(previousRadioSig > 7){            //signal still good
      previousRadioSig = currentRadioSig;
    }else {                              //signal recovered
      error = InitGPRS(RUNNING_TIME);   //errors already logged with setCurrentError(error) inside this InitGPRS
      previousRadioSig = currentRadioSig;
      if(error > 0) {                    //initGPRS failed
//      Serial.print("GPRSfail: ");
//      Serial.println(error);
        return error;
      }
//      Serial.println("GprsInitOK");
    }
  }



  //get the local ip number
  //GPRS.println("AT+CIFSR");
  //if(sim808.readable()) {
  sim808_send_cmd("AT+CIFSR\r\n");
  //}
  delay(50);

  //if(sim808.readable()) {
  sim808_send_cmd("AT+CDNSGIP=\"ardugps.hopto.org\"\r\n");
  //}
  //sim808_flush_serial();
  delay(50);

  //start a connection to TCP to URL and port
  if (prot == TCP){
    if(!SendCIPSTART(TCP)){            //TCP protocol
      setCurrentError(START_GPRS_TCP_FAIL);
      return START_GPRS_TCP_FAIL;      //"AT+CIPSTART => FAIL";
    }
  }else {                              //UDP protocol
    if(!SendCIPSTART(UDP)){
      setCurrentError(START_GPRS_UDP_FAIL);
      return START_GPRS_UDP_FAIL;      //"AT+CIPSTART => FAIL";
    }
  }


  delay(200);

  //copy latitude and longitude to send via TCP server
  memset(data2db,'\0',sizeof(data2db));
  strncpy(data2db,latitude,sizeof(latitude));
  strcat(data2db,",");
  strcat(data2db,longitude);

  strcat(data2db,",");
  strcat(data2db,MSL_altitude);

  strcat(data2db,",");
  strcat(data2db,Speed);

//  Serial.println("valor de data2db: ");
//  Serial.println(data2db);



//  if(!SendDataCIPSEND(data2db,sizeof(data2db))){
//    if(prot == TCP){                  //TCP protocol
//      if(!SendCIPSTART(TCP)){         //Trying to restart TCP protocol
//
//        setCurrentError(START_GPRS_TCP_FAIL);
//        return START_GPRS_TCP_FAIL;
//      }
//
//      //delay(1000);
//      delay(100);
//      if(!SendDataCIPSEND(data2db,sizeof(data2db))){    //Trying to resend TCP data
//
//        setCurrentError(SEND_TCP_DATA_FAIL);
//        return SEND_TCP_DATA_FAIL;                      //"CIPSEND =>RESEND FAIL"
//      }
//
//    }else{                             //UDP protocol
//      setCurrentError(SEND_UDP_DATA_FAIL);
//      return SEND_UDP_DATA_FAIL;
//    }
//  }
  sendNumber = SendDataCIPSEND(data2db,sizeof(data2db));
//  Serial.print("sendNumber: ");
//  Serial.println(sendNumber);
//  Serial.print("data2db: ");
//  Serial.println(data2db);

  if(sendNumber == 0){
  //if((SendDataCIPSEND(data2db,sizeof(data2db)))== 0){
    if(prot == TCP){                  //TCP protocol
      if(!SendCIPSTART(TCP)){         //Trying to restart TCP protocol
        setCurrentError(START_GPRS_TCP_FAIL);
        return START_GPRS_TCP_FAIL;
      }
      //delay(100);
      sendNumber = SendDataCIPSEND(data2db,sizeof(data2db));
      if(sendNumber == 0){   //Trying to resend TCP data
        setCurrentError(SEND_TCP_DATA_FAIL);
        return SEND_TCP_DATA_FAIL;                      //"CIPSEND =>RESEND FAIL"
      }

    }else{                             //UDP protocol
      setCurrentError(SEND_UDP_DATA_FAIL);
      return SEND_UDP_DATA_FAIL;
    }
  }else{
    if(prot == TCP){
      clearCurrentError(SEND_TCP_DATA_FAIL);
    }else{
      clearCurrentError(SEND_UDP_DATA_FAIL);
    }
  }

//  delay(50);

  if (CheckConnectionStatus()) {
    sim808_check_with_cmd("AT+CIPCLOSE\r\n", "CLOSE OK\r\n", CMD);
    //Serial.println("Uno have disconnected from Server");
  }

  clearAllError(); //the routine proceed successfully, no error occured
                   //only place to clear all errors

  return NO_ERROR; // no error on GPRS

}//end procedure

/*************************************************************************************
 * SendCIPSTART()
 *************************************************************************************/
boolean SendCIPSTART (Protocol ptl)
{
  //char bufferC[30];
  //sim808_check_with_cmd(F("AT\r\n"),"OK",CMD);

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
      sim808_send_cmd("AT+CIPSTART=\"UDP\",\"ardugps.hopto.org\",6789\r\n");
      return true;
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
  char command[20]="AT+CIPSEND=";

  itoa(len, num);
  strcat(command,num);
  strcat(command,"\r\n");
  command[sizeof(command)-1] = '\0';
//  Serial.print("command: ");
//  Serial.println(command);

  if(len > 0){
    //sim808_send_cmd("AT+CIPSEND=");
    //sim808_send_cmd(num);
    if(!sim808_check_with_cmd(command,">",CMD)) {
      return 0;
    }
    //delay(100);
    sim808_send_cmd(str);
    //delay(100);
    sim808_send_End_Mark();
    return len;
  }else{
    return 0;
  }












}

/**************************************************************************************
 * itoa() not in standard library
 **************************************************************************************/
static char* itoa(int n, char s[])
{
    int i, sign;
    if ((sign = n) < 0)
        n = -n;
    i = 0;
    do
    {
      s[i++] = n % 10 + '0';
    } while ((n /= 10) > 0);
    if (sign < 0)
        s[i++] = '-';
    s[i] = '\0';
    reverse(s);
    return s;
}

static void reverse(char s[])
{
    int i, j;
    char c;
    for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
        c = s[i];
        s[i] = s[j];
        s[j] = c;
    }
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
//      Serial.print("LATITUDE        : ");
//      Serial.println(latitude);
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
//      Serial.print("LONGITUDE       : ");
//      Serial.println(longitude);

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

   if(!strstr(gpsBuffer,"+CGPSINF: 32")) { //not valid gps data
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
   }//while
   return false;
}


/*************************************************************
 * CheckSignalQuality(char *)
 * scan and print the signal quality
 * return 99 unknow or failed to extract
 *        0 to 98 valid result
 * AT+CSQ
 * +CSQ: 21,0
 * OK
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
    strncpy(sig,token,sizeof(sig));
    sig[sizeof(sig)-1] = '\0';
    return atoi(sig);
  }
}



/********************************************************************************
* GPSAnalyzer128(infoLocation);
* GPZDA
* AT+CGPSINF=128
* at+cgpsinf=128
* +CGPSINF: 128,045140.000,02,11,2017,00,00
* Message ID          : $GPZDA = 128 (ZDA protocol header)
* UTC Time            : hhmmss.sss
* Day                 : Day of month 1-31
* Month               : Month of the year 1-12
* Year                : 1980-2079
* Local zone hour     : hour (offset from UTC set to 0)
* Local zone minutes  : minute (offset from UTC set to 0)
* Checksum            : knots
* <CR><LF>            : end of message
 ********************************************************************************/
boolean GPSAnalyzer128() {
   char *token;
  //delay(8000);  //5000 no GPS data 10 times
  sim808_clean_buffer(gprsBuffer,sizeof(gprsBuffer));
  sim808_send_cmd("AT+CGPSINF=128\r\n");
  sim808_read_buffer(gprsBuffer,55,DEFAULT_TIMEOUT);


   /* get the first token */
//   Serial.println("****************Content of gpsBuffer*******************");
//   Serial.println("valor do buffer :");
//   Serial.println(gprsBuffer);

   //Get Command sent AT_CGPSINF=128
   token = strtok(gprsBuffer,"\n\r" );
   if(token == NULL)  return false;

   //Get "+CGPSINF:" string
   token = strtok(NULL, ":");
   if(token == NULL)  return false;

   //Get CGPSINF value 128
   token = strtok(NULL, ",");

   //Get time and create string of time and date "hhmmss.sss-dd-mm-yyyy";
   token = strtok(NULL, ",");
   if(token != NULL){
       strncpy(startDateTime,token,sizeof(startDateTime)); //Time
       strcat(startDateTime,"-");
   }else{
     return false;
   }

   //Get Day and create string of time and date "hhmmss.sss-dd-mm-yyyy";
   token = strtok(NULL, ",");
   if(token != NULL){
       //strncpy(startDateTime,token,sizeof(startDateTime)); //Time
       strcat(startDateTime,token);
       strcat(startDateTime,"-");
   }else{
     return false;
   }

   //Get Month and create string of time and date "hhmmss.sss-dd-mm-yyyy";
   token = strtok(NULL, ",");
   if(token != NULL){
       //strncpy(startDateTime,token,sizeof(startDateTime)); //Time
       strcat(startDateTime,token);
       strcat(startDateTime,"-");
   }else{
     return false;
   }

   //Get Year and create string of time and date "hhmmss.sss-dd-mm-yyyy";
   token = strtok(NULL, ",");
   if(token != NULL){
       //strncpy(startDateTime,token,sizeof(startDateTime)); //Time
       strcat(startDateTime,token);
       startDateTime[sizeof(startDateTime)-1] = '\0';
   }else{
     return false;
   }

//   Serial.println("startDateTime:");
//   Serial.println(startDateTime);
   return true;
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


///****************************************************************************
// * storeLog(int add, char *val)
// * UNO memory 1K
// * Param
// * add address to write data
// * val value to write
// ****************************************************************************/
// int storeData(int add, byte val){
//  int addTemp;
//  addTemp = add;
//
//  EEPROM.write(add, char(val));
//
//  addTemp = addTemp + 1;
//  if (addTemp == EEPROM.length()) {
//    addTemp = 2;                    //reserve address 0 and 1 to write last position
//   }
//  return  addTemp;
// }
//
///****************************************************************************
// * voi storeString(
// *
// ***************************************************************************/
//int storeString(int add, char *val){
//  int addNext, addCurr;
//  char * name = val;
//  int size = 0;
//
//  while(true)
//  {
//    if(*(val + size) == '\0') // returns 5...
//      break;
//
//     size++;
//  }
//  addCurr=add;
//  addNext = add + size;
//  if (addNext >= EEPROM.length()) { //data do not fit EEPROM
//    addCurr = 2;                     //address 0 and 1 is reserved for lastWriteAddress
//    addNext = addCurr + size;
//  }
//
//  for(int j=0;j<size;j++){
//    EEPROM.write(addCurr,name[j]);
//    addCurr++;
//  }
//  return addNext;
//}
//
///******************************************************************************
// * Routines to write and read integer long 4 bytes max
// * 32,767 hold 2^16 bits
// * Needs 2 bytes
// * ****************************************************************************/
// unsigned int EEPROMWriteInt(int p_address, long p_value)
//  {
//  int addNext, addCurr;
//  byte lowByte0 = ((p_value >> 0) & 0xFF);
//  byte lowByte1 = ((p_value >> 8) & 0xFF);
//
//  addCurr=p_address;
//  addNext = p_address + 2;
//  if (addNext >= EEPROM.length()) { //data do not fit EEPROM
//    addCurr = 2;                     //address 0 and 1 is reserved for lastWriteAddress
//    addNext = addCurr + 2;
//  }
//
//  EEPROM.write(addCurr, lowByte0);
//  EEPROM.write(addCurr + 1, lowByte1);
//
//
//  return addNext;
//  }
//
///******************************************************************************
// * Routines to write and read integer long 4 bytes max
// * 32,767 hold 2^16 bits
// * Needs 2 bytes
// * ****************************************************************************/
//unsigned int EEPROMReadInt(int p_address){
//  byte lowByte0 = EEPROM.read(p_address);
//  byte lowByte1 =  EEPROM.read(p_address + 1);
//  return ((lowByte0 << 0) & (0xFF)) + ((lowByte1 << 8) & (0xFF00));
//  }

