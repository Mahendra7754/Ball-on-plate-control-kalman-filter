/**
*  ***********************************************************************
*  @file    BallPlate.h
*  @author  mys (based on WS2014 Master's Project by I. Lienhardt)
*  @version V1.0
*  @brief   Application specific symbolic consts/config params and typedefs.
*
*  @date    17/03/2026   (orig. 06/12/2016)
**************************************************************************/


#ifndef   BALLPLATE_H
#define   BALLPLATE_H

// activate debug messages to console
//#define   DEBUG_MSG_ON

// define one of "_SYS_2ND_ORDER_OBS_", "_SYS_3RD_ORDER_OBS_" or 
//  "_SYS_3RD_ORDER_KF_" to run the specific model

//#define _SYS_2ND_ORDER_OBS_ 
//#define _SYS_3RD_ORDER_OBS_ 
//#define _SYS_3RD_ORDER_KF_
#define  _SYS_4TH_ORDER_EKF_

#include  "..\basicIO\mctDefs.h" // useful constants, macros, type shorthands 

#include  "bbTypeDefs.h"         // application specific typdefs
#include  "bbSSParams.h"         // state space model parameters



#define   TERM_BUFSIZE    16           //!< buffer size for terminal input
#define   AUX_RX_BUFSIZE  32           //!< buffer size for measurement data packets from PC


// various execution- and test modes
#define   OPMODE_INIT_SYSTEM              1       //!< board & servos initialized
#define   OPMODE_CALIBRATE_XAXIS          4       //!< calibrate x-axis mode
#define   OPMODE_CALIBRATE_YAXIS          8       //!< calibrate y-axis mode
#define   OPMODE_TEST_STEP_SEQUENCE      10       //!< test step sequence mode
#define   OPMODE_CLOSED_LOOP_CONTROL     20       //!< closed loop control mode
#define   OPMODE_RECEIVE_MEASUREMENTS    50       //!< closed loop control mode
#define   OPMODE_STATE_OBSERVER          70       //!< closed loop control mode
#define   OPMODE_STATE_CONTROLLER       100       //!< closed loop control mode


//#define INVERT_SERVO_X   //!< invert servo turning sense, eg. for HITEC-645 servo 
//#define INVERT_SERVO_Y   //!< invert servo turning sense, eg. for HITEC-645 servo 



// sample time in seconds - here: 50 ms equal to 20 FPS
#define   T                      (float) (1.0/20)      //!< sample period in [s]
#define   T22                    (float) (T*T/2.0)     //!< term used in F & G

// conversion factor (1 meter == 1385 pxl)  ??? FOR WHICH CAMERA vgl. ZiSo ???
#define   PIXEL_PER_METER        (float) 1385.0 //!< conversion factor meter > pxl

#define   PLATE_XMIN_PXL          80         //!< visible ballplate image X min 
#define   PLATE_XMAX_PXL         680         //!< visible ballplate image X max

#define   PLATE_YMIN_PXL          30         //!< visible ballplate image Y min 
#define   PLATE_YMAX_PXL         570         //!< visible ballplate image Y max

// servo related PWM/Timer constants
#define   CENTER_PULSE_WIDTH     1500        //!< servo zero position pulse width in [us]
#define   MIN_PULSE_WIDTH        1000        //!< min pulse width in [us]
#define   MAX_PULSE_WIDTH        2000        //!< max pulse width in [us]

// servo angle offsets for calibrating ballplate horizontal position
#define   PWM_SERVO_X_OFFSET      -50        //!< in PWM [us] units -50 seems OK
#define   PWM_SERVO_Y_OFFSET        0        //!< in PWM [us] units   0


// PWM pulse repetition period (default 20000 = 20ms OK, 15000 = 15ms OK)
#define     DEFAULT_PULSE_PERIOD   15000     //!< PWM pulse repeat period in [us]
//#define   DEFAULT_PULSE_PERIOD   13000     //!< NOK -> STRONG JITTER W VS-11AMB Modelcraft Servo
//#define   DEFAULT_PULSE_PERIOD   10000     //!< NOK -> STRONG JITTER W VS-11AMB Modelcraft Servo

// mechanical dimensions/limits
// max servo angle in radians ( -50 deg <= servo angle <= 50 deg )
#define   MAX_SERVO_ANGLE_DEG    50          //!< max servo angle in [o]
#define   MAX_SERVO_ANGLE_RAD    DEG2RAD(MAX_SERVO_ANGLE_DEG)  //!< max servo angle in [rad]
#define   SERVO_ARM_LENGTH       0.03        //!< servo arm length in [m]
#define   HALF_BOARD_LENGTH      0.20        //!< plate axis to servo arm attachment point in [m]


/************************************************************************\
*    FUNCTION PROTOTYPES
\************************************************************************/

void bbSSControlAndObserver( SS_VAR *xAxis,  SS_VAR *yAxis,  SS_PAR *ssModel );


#endif  /* BALLPLATE_H */

/************************************************************************\
*    END OF MODULE BallPlate.h
\************************************************************************/
