// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _stm32_sim808_eclipse_H_
#define _stm32_sim808_eclipse_H_
#include "Arduino.h"

#include "arduino.h"
#include "HardwareSerial.h"
#include "HardwareTimer.h"
#include <sim808_stm32f1.h>    //lib adapted to works with STM32F1
#include <string.h>

//#include <EEPROM.h>
#include "DFRobot_sim808_stm32.h"
//#include <sim808_GPS.h>





//add your includes for the project stm32_sim808_eclipse here

//end of add your includes here

/*********************************************************
 * Definition of error codes, currently array of 20 errors
 *********************************************************/
typedef enum {
  NO_ERROR              = 0,

  CIPSHUT_FAIL          = 1,   //GPRS related faults
  IPR9600_FAIL          = 2,
  CSTT_UNINET_FAIL      = 3,
  CIICR_FAIL            = 4,
  CDNSGIP_FAIL          = 5,
  NO_GPRS_SIGNAL        = 6,
  DNS_ERROR             = 7,
  PDP_DEACT             = 8,
  START_GPRS_TCP_FAIL   = 9,
  START_GPRS_UDP_FAIL   = 10,
  SEND_TCP_DATA_FAIL    = 11,
  SEND_UDP_DATA_FAIL    = 12,

  NO_GPS_DATA_AVAILABLE = 13,   //GPS related faults
  POWER_GPS_FAIL        = 14,
  RESTART_GPS_FAIL      = 15,
  GPS_SYNC_FAIL         = 16,
  INITIAL_TIME_NOT_SET  = 17,

  SIM808_SERIAL_FAILED  = 18,
  CFUN1_FAILED          = 19,
  CPIN_SIMCARD_FAILED   = 20,
} fault_type;

typedef enum {
  GPS_off          = 0,
  GPS_on           = 1,
  GPS_on_searching = 2,
  GPS_on_notsync   = 3,
  GPS_on_signal    = 4,
}GPS_states_type ;



typedef enum {
  FIRST_TIME,
  RUNNING_TIME,
}initTime_type;




//add your function definitions for the project stm32_sim808_eclipse here
void PowerSIM808();
void handlerGetGPSLocation();
void handlerConnectGPS();
void getAllButFirstAndLast(const char *input, char *output);
void power(void);
void GetGPSLocation();
void PrintSecondsElapsed();
void createSMSMessage(char *message,char *lat, char *lng, char *alt, char *sp);
void setCurrentError(fault_type currentError);
void clearCurrentError(fault_type error);
void clearAllError();
void printAllError();

static char* itoa(int n, char s[]);
static void reverse(char s[]);

boolean CheckConnectionStatus(void);
int SendDataCIPSEND(const char * str, int len);
int getSignalQuality(char *signalBuffer);
int readCurrentError(fault_type error);
boolean sim808_init_check();
boolean extractCallingNumber(char *inMessage);
boolean extractCallingNumberSMS(char *inMessage);
boolean connect_GPS(int timeout, initTime_type t);
boolean SendCIPSTART (Protocol ptl);
boolean GPSAnalyzer(char *gpsBuffer);
boolean GPSAnalyzer32(char *gpsBuffer) ;
boolean GPSAnalyzer128();
fault_type InitGPRS(initTime_type t);
fault_type SendGPSLocation();

void ClockSeconds();
void handlerMachine2();
void handlerMachine3();
void handlerMachine4();


//TODO ver como funciona

//Do not add code below this line
#endif /* _stm32_sim808_eclipse_H_ */
