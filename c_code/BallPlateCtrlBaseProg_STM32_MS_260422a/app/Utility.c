/**
*  ************************************************************************************************
*  @file    utiltity.c
*  @author  B. Mysliwetz
*  @version V1.00
*
*  @brief   Helper functions for BallPlate application: 
*
*  Changes:
*  07.12.2016 mys   initial version
*
*  @date    18/03/2026
***************************************************************************************************
*/

#include  <stdio.h>              // C-StdLib sprintf(), scanf() prototypes
#include  <stdint.h>             // std C types for portability 6 MISRA compliance

#include  "stm32f10x.h"          // STM32F10x Peripherals Definitions

#include  "platform_cfg.h"       // target specific peripherals config
#include  "bsp.h"                // low level IO, clock & int related defs for uC/OS
#include  "BallPlate.h"          // defines system order

#include  "..\basicIO\basicIO.h" // console text IO & LED control


/*************************************************************************************************\
*    EXTERNAL FUNCTION PROTOTYPES
\*************************************************************************************************/

UINT32 get_pc_from_intstack(void);  //!< used in HardFault_Handler(), reads PC from interrupt stack

/*************************************************************************************************\
*    EXTERNAL FUNCTION PROTOTYPES
\*************************************************************************************************/

// Compute PWMcmd scaled in [us] from servo angle cmd in [rad]; xAxis +1, yAxis -1 sign !
UINT16 setPWMSignalAngle( SS_VAR *axis, int sign );


/*************************************************************************************************\
*    LOCAL FUNCTION DEFINITIONS
\*************************************************************************************************/

UINT8 initSystem( SS_VAR *xAxis, SS_VAR *yAxis )
{
  BSP_Init( );          // peripherals initialization
  BSP_SetLED( 0x55 );   // set initial LED on/off pattern

  // init servo command horizontal calibration offsets
  xAxis->servoAngleCalOffsetPWM = PWM_SERVO_X_OFFSET;
  yAxis->servoAngleCalOffsetPWM = PWM_SERVO_Y_OFFSET;

  // init TIMER channels to generate PWM signals for X-/Y-axis servo actuator
  TIM_GPIO->ARR    = DEFAULT_PULSE_PERIOD;     // default PWM repeat period = 15ms
  TIM_XAXIS_RSTVAL = setPWMSignalAngle( xAxis, +1 ); // unified version
  TIM_YAXIS_RSTVAL = setPWMSignalAngle( yAxis, -1 ); // unified version
	
  return OPMODE_INIT_SYSTEM;
}


UINT8 commandInterpreter( SS_VAR *xAxis, SS_VAR *yAxis )
{
  UINT8  c;                        //!< user keyboard input char 
  UINT8  RunMode;                  //!< current run mode
	
  char   cmdLineBuf[TERM_BUFSIZE]; //!< for user console input

  INT16  calXServoOffsetDeg;       //!< xAxis servo horizontal cal offset in [o] 
  INT16  calXServoOffsetPWM;       //!< xAxis servo horizontal cal offset in [us] 
  UINT16 cmdXServoPWM;             //!< xAxis servo PWM cmd in [us] 

//INT16  XServoStepDeg;            //!< xAxis servo test step amplitude in [o]    added 200327
//INT32  XServoStepPWM;            //!< xAxis servo test step amplitude in [us] 
//UINT16 XServoStepPCount;         //!< xAxis servo test step width time in [T]   50ms sample periods   
//UINT16 XServoStepWCount;         //!< xAxis servo test step wait  time in [T]   50ms sample periods   
 
// Mini command interpreter options
  print_string("\n\rc = X-Servo Horiz. Calibration   r = Run Control Loop\n\r");
//print_string("\n\rc = X-Servo Horiz. Calibration   t = X-Servo Test-Steps   r = Run Control Loop\n\r");

  while (TRUE)     // mini command interpreter
  {
    c = _keyhit(); // read input from keyboard
    if ( c != 0 ) 
    {
      switch (c)                  
      {
      case 'r':    // start closed loop position control mode
        print_string("\n\rr -> Run Control Loop\n\n\n\r");
        RunMode = OPMODE_CLOSED_LOOP_CONTROL;                   
        break;
/**********************************************************************************      
      case 't':    // enter X servo step amplitude in [o]         added 200327
      //print_string("\n\rt -> X-Servo Test Step Sequence\n\r");
        print_string("\n\rX-Servo TestStep Amplt in [o] = ?  ");
        getint(cmdLineBuf);                                    // read value string
        XServoStepDeg = asciitoint(cmdLineBuf);                // convert to integer
      //print_string("     Input was ");                       // verify
      //print_int(XServoStepDeg);
        print_string("\n\r");

        XServoStepPWM = 
          (XServoStepDeg * (MAX_PULSE_WIDTH-MIN_PULSE_WIDTH)) / (2*MAX_SERVO_ANGLE_DEG); 
        print_string("\n\rX-Servo TestStep Amplt in [us] = "); // verify
        print_int(XServoStepPWM);
        print_string("\n\r");
      
        print_string("\n\rX-Servo TestStep Duration in [T] = ? 20T=1sec   ");
        getint(cmdLineBuf);                                    // read value string
        XServoStepPCount = asciitoint(cmdLineBuf);             // convert to integer
      //print_string("     Input was ");                       // verify
      //print_int(XServoStepPCount);
        print_string("\n\r");
      
        print_string("\n\rX-Servo TestStep WaitTime in [T] = ? 20T=1sec   ");
        getint(cmdLineBuf);                                    // read value string
        XServoStepWCount = asciitoint(cmdLineBuf);             // convert to integer
      //print_string("     Input was ");                       // verify
      //print_int(XServoStepWCount);
        print_string("\n\r");
      
        RunMode = OPMODE_TEST_STEP_SEQUENCE;
        break;
*************************************************************************************/

      case 'c':    // enter X servo cal offset in [o]
        print_string("\n\rc -> X-Servo Cal Offset in [o] = ?  ");
        getint(cmdLineBuf);                                    // read value
        calXServoOffsetDeg = asciitoint(cmdLineBuf);           // convert to integer
        print_string("     Input was ");                       // verify
        print_int(calXServoOffsetDeg);
        print_string("\n\r");

        calXServoOffsetPWM = 
          (calXServoOffsetDeg * (MAX_PULSE_WIDTH-MIN_PULSE_WIDTH)) / (2*MAX_SERVO_ANGLE_DEG); 
        print_string("X-Servo Cal Offset in [us] = ");         // verify
        print_int(calXServoOffsetPWM);
        //print_string("\n\r");
      
        xAxis->servoAngleCalOffsetPWM   = calXServoOffsetPWM;
        TIM_XAXIS_RSTVAL = cmdXServoPWM = setPWMSignalAngle( xAxis, +1 );			
        print_string("   X-Servo PWM in [us] = ");             // verify
        print_int(DEFAULT_PULSE_PERIOD-cmdXServoPWM+1);
        print_string("\n\r");
        break;
      
      default:
        print_string("\nc = X-Servo Horiz. Calibration   r = Run Control Loop\n\r");
      //print_string("\nc = X-Servo Horiz. Calibration   t = X-Servo Test-Steps   r = Run Control Loop\n\r");
        break;
      }
    }
    if ( (c == 'r') L_OR (c == 't') ) break;  // leave command interpreter & start control loop
  } // end while loop of mini command interpreter

  if ( RunMode == 0 )
  {
     print_string("\n\n\rILLEGAL RunMode == 0 !!\n\n\n\r");
    _getch();
  }
	return RunMode;

} // end commandInterpreter()


UINT16  setPWMSignalAngle( SS_VAR *axis, int sign )
/**
*  ************************************************************************************************
*
*  @brief  Compute PWMcmd value in [us] units from servo angle cmd in [rad].
*
*  @param  axis : uses/updates SS_VAR struct members servoAngleCmdRad & servoAngleCmdPWM
*  @param  sign : +1 for X-axis, -1 for Y-axis needed (!)
*  @retval UINT16 timer CCR value scaled in [us] 
*
*  260317  mys  unified version, replaces setPWMSignalAngleX() &setPWMSignalAngleY()
***************************************************************************************************
*/
{
  axis->servoAngleCmdPWM =                                                   // in [us]
	      CENTER_PULSE_WIDTH + axis->servoAngleCalOffsetPWM - 1 - 
        sign * (INT16) (
	        (axis->servoAngleCmdRad * (MAX_PULSE_WIDTH-MIN_PULSE_WIDTH) / 2*MAX_SERVO_ANGLE_RAD)
           // scaling       [rad]                 [us]                           [rad]
        );
  
  if ( axis->servoAngleCmdPWM > MAX_PULSE_WIDTH ) axis->servoAngleCmdPWM = MAX_PULSE_WIDTH;
  if ( axis->servoAngleCmdPWM < MIN_PULSE_WIDTH ) axis->servoAngleCmdPWM = MIN_PULSE_WIDTH;
    
  return DEFAULT_PULSE_PERIOD - axis->servoAngleCmdPWM;  // CCRval = ARRperiod-PWM in [us]

} // end setPWMSignalAngle()


void print_float(float var)
/**
*  ************************************************************************************************
*  @brief  Print floating point number with 6 decimals to CONSOLE 
*          slightly strange legacy code by I. Lienhardt ... 
*
*  @param  var : floating point variable to print
*  @retval none
***************************************************************************************************
*/
{
  char str[16];
  
  int intPart;         // integer part (678).
  float f2;            // fractional part (0.01234567).
  int d2;              // fract part converted to integer (123).
  if (var < 0. ){
    var = -var;
    _putch('-');       // print - sign  
  }  
  else _putch('+');    // print + sign  
  
  intPart = var;
  f2 = var - intPart;
  d2 = (f2*100000);    // ie. print 6 decimals Mr. L ...

//  print_int(intPart);
//  _putch('.');
//  print_int(d2);
//  
  
  _sprintf (str, "%d.%05d", intPart, d2);
  print_string(str);
  
}


void print_TwoFloat( const float A, const float B)
/**
*  ************************************************************************************************
*  @brief  Output of two floating point numbers on serial interface
*
*  @param  A : first  floating point variable to print
*  @param  B : second floating point variable to print
*  @retval none
***************************************************************************************************
*/
{
  print_float(A);
  _putch(' ');
  print_float(B);
  _putch(' ');
}


void HardFault_Handler(void)
/**
*  ************************************************************************************************
*  @brief  Prints warning message to terminal and 'halts' in infinite loop.
*          Overwrites original/dummy hardfault handler from STM32F10x.s !
*
*  @retval none
***************************************************************************************************
*/
{
    UINT32 pc;
    UINT32 *hfsr = (UINT32*) 0xE000ED2C;     // Hard Fault Status Register

    // read PC value from interrupt stack
    pc = get_pc_from_intstack( );
    print_string("\n\n\n\rHARDFAULT occurred @ PC = ");
    print_hex32(pc);
    print_string(" triggered by ");

  //if ( (*hfsr & (1<<31)) )           // case DEBUGEVT
    if ( (*hfsr & 0x80000000) )        // case DEBUGEVT
    {
      print_string("debug event");
    }
    else if ( *hfsr & (1<<30) )        // case FORCED
    {
      print_string("unhandled bus/mem/usage fault");
    }
    else if ( *hfsr & (1<<1) )         // case VECTBL
    {
      print_string("failed vector fetch");
    }
    else // illegal case
    {
      print_string("UNKNOWN/ILLEGAL case");
    }

    print_string("\n\r");

    while (1) {}   // 'halt' in infinite loop

} // end HardFault_Handler()


void    delay(UINT32 delaycount)
/**
*  ************************************************************************************************
*
*  @brief  Simple active waiting loop,
*          effective delay time depends on CPU type & clock frequency !
*
*  @param  delaycount : 32-Bit delay loop starting value
*  @retval none
***************************************************************************************************
*/
{
  while ( delaycount-- );    // decrement until 0
}

/************************************************************************\
*    END OF MODULE utility.c
\************************************************************************/
