/**
*  ***********************************************************************
*  @file    auxSIOcomm.h
*  @author  B. Mysliwetz
*  @version V0.01
*  @brief   Functions and defines for control related PC communication.
*
*  @date    2016/08/19
**************************************************************************/


#ifndef AUX_SIO_COMM_H
#define AUX_SIO_COMM_H


/** Enum for setting LEDs.
*   Range of LEDs 8-15.
*/
typedef enum STATUS_LEDS  // enum for setting LEDs & ext. timing measurement via scope
{ serialSend      =  8,   //!< sending process of data over RS232 interface to PC
  calcFSFAndObs   = 10,   //!< controller calculations (FSF and Observer/Kalman Filter)
  calcPWM         = 11,   //!< calculations for the timer/PWM parameters
  serialReception = 12,   //!< reception process of data over RS232 interface
  oneCompleteLoop = 15    //!< runtime of whole loop
} STATUS_LEDS;

#define   LED_ON    0     // turns LED ON
#define   LED_OFF   1     // turns LED OFF


/* Function Prototypes */

void   sendLogData( const struct SS_VAR *xAxis, const struct SS_VAR *yAxis );
void   sendLogFloat( UINT8, float );
void   sendLogShort( UINT8, short );

UINT16 receiveMeasurements( UINT8* pRxBuffer );
UINT8  checkReceivedOK( UINT8* );

#endif /* AUX_SIO_COMM_H */

/************************************************************************\
*    END OF MODULE auxSIOcomm.h
\************************************************************************/
