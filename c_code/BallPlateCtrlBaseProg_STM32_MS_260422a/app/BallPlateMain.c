/**
*  ************************************************************************************************
*  @file    BallPlateMain.c
*  @author  mys (based on WS2014 Master's Project by I. Lienhardt) 
*  @brief   Main module of BallPlate Controller Project.
*
*  @version V1.10  2nd/3rd order full state feedback & observer
*
* Peripherals & configuration settings used:
*
*  Target   MCBSTM32           MCBSTM32C       STM32F4_DISCOVERY
*  MCU      STM32F103RB        STM32F107VC     STM32F407VG
*
*  Console-SIO for user-/debug-/status-messages to terminal:
*  Console  USART1             USART2          USART6
*           115200Bd 8N1       115200Bd 8N1
*           TxD - PA9          TxD - PD5
*           RxD - PA10         RxD - PD6
*
*  Aux-SIO for control related communication w image-processing PC:
*  AuxSIO   USART2             USART3          ???
*           115200Bd 8N1       115200Bd 8N1
*           TxD - PA2          TxD - PB10
*           RxD - PA3          RxD - PB11
*
*  Timers to generate PWM signals for servo actuator control:
*  X-Servo  TIM3_CH1 - PA6     TIM4_CH3 - PB8
*  Y-Servo  TIM3_CH2 - PA7     TIM4_CH4 - PB9
*
*  Timer-Settings: 
*           - PWM-mode         1 
*           - Pulse period     10ms -> 15ms mys 190502 !
*           - Pulse width      1ms ... 2ms
*           - Pre-scaler:      72d    -> 1 MHz count freq
*           - Auto-reload:     15000d -> 15 ms 
*           - Output           active low, edge-aligned
*           - Up-counting
*
*  LEDs for indicating/measuring activity/timing of certain functions:
*  LED1     - ...
*  LED2     - ... see auxSIOcomm.h
*  LED3     - ...
*  LED4     - ...
*
*  Modifications
*  160901   mys   cmd computation corrected (UINT16) to (INT16)
*                 use ZiSo controller gains: Kp = -1.0, Kd = -1.5
*  160905   mys   use ZiSo controller gains: Kp = -1.5, Kd = -1.5
*                 use 10ms PulsPeriod, -50us/-50us horiz. offsets 
*  160909   mys   use OBS3 & ZiSo controller gains: Kp = -1.5, Kd = -1.5
*  160909c  mys   use KF3  & ZiSo controller gains: Kp = -1.5, Kd = -1.5
*  160912a  mys   use OBS2 & ZiSo con/obs gains:    Kp = -1.5, Kd = -1.5
*                                                   h1 =  1.0  h2 =  5.6
*                                                 -50us/ 0us horiz. offsets
*  161109   mys   inverted X servo turning sense for Graupner DES 806 in 
*                 Controller.c
*  161111   mys   streamlined var & const names, plate balanced in x
*  161111   mys   use      controller gains: Kp = -1.5, Kd = -1.0
*                 for HITEC645-Servo on X-axis     Kd = -1.5 oszillates !
*  161206   mys   variable names streamlining, cmdServoAngleCalOffsetPWM
*  161207   mys   _aux_putch() & _aux_getch() via AUX_UART, removed L's legacy
*                 sh.. functions DB_XXX()
*  161208   mys   command interpreter, set servo angle horiz. cal offsets
*  161221   mys   T, MAX_SERVO_ANGLE_RAD; init ALL struct vars 
*                 init XY.xDotEstMS !!CORRECTLY!! to avoid servo jerks 
*  170126   mys   Keep control loop static at init condition for first 3 sycles 
*                 if ( kSampleIndex < 3 )    // keep everything static
*  170206   mys   a if ( kSampleIndex < 10 ) // keep everything static
*                 b xDotEstk1 = 2000 after/in 1st cycle -> sh.. in est-formula
*  170227   mys   c if ( kSampleIndex < 5 )  // keep everything static
*  170312   mys   Stack size set to 0x300 in STM32F10x.s to avoid stack overflow
*  170313   mys   Added HardFault_Handler() & test illegal adress via *err_ptr
*  171108   mys   Chuong's controller gains: Kp = -2.2, Kd = -0.7  almost instable !!
*  171115   mys   Rescale servoAngleCmdRad to [o] in sendLogData() for serial output !!
*                 Rescale plateAngleRad to [o] in sendLogData() Kp=-1.5 Kd=-1.1
*  171115   mys   Removed tabs, updated comments
*  190111   mys   compute X->servoAngleCmdPWM = CENTER_PULSE_WIDTH + X->servoAngleCalOffsetPWM - 
*   (INT16) (X->servoAngleCmdRad * (MAX_PULSE_WIDTH-MIN_PULSE_WIDTH) / 2*MAX_SERVO_ANGLE_RAD);  // in [us]
*  190205   mys   changed PWM repeat period from 10ms to 20ms to avoid
*                 servo jitter; DEFAULT_PULSE_PERIOD = 20000 in BallPlate.h 
*  190205   mys   controller gains: Kp = -5.0, Kd = -3.0 for Modelcraft VS-11AMB servo
*  190206   mys   changed PWM repeat period to 15ms to avoid servo jitter via
*                 DEFAULT_PULSE_PERIOD = 15000 in BallPlate.h 
*  190206   mys   Update of doxygen compatible comments, (re-)added HardFault_Handler()
*  200327   mys   Added ThetaServo test step generation 'T' to command interpreter
*  220117   mys   CCR-1 for PWM generation
*  260315   mys   Refactoring: SS_VAR & SS_PAR structs static ! Partitioning of main() into
*                 -> initSystem(), printHello(), commandInterpreter(), controlLoop()
*  260317   mys   UINT16 setPWMSignalAngle( SS_VAR *axis, int sign ); unified version
*  260318   mys   moved function controlLoop() ->  module Controller.c 
*  260318   mys   split up bbSSControlAndObserver() ->  StateObserver() + StateController()
*	 260412   ms    added 4th order Extended Kalman Filter code
*
*  @date   12/04/2026
***************************************************************************************************
*/

#define     SW_VERSION        "V260420a_ms"    //!< version string


/*************************************************************************************************\
*    HEADERFILES for runtime environment
\*************************************************************************************************/

#include  <stdio.h>              // C-StdLib sprintf(), scanf() prototypes
#include  <stdint.h>             // std C types for portability 6 MISRA compliance

#include  "stm32f10x.h"          // STM32F10x Peripherals Definitions

#include  "platform_cfg.h"       // target specific peripherals config
#include  "bsp.h"                // low level IO, clock & int defs for uC/OS

#include  "..\basicIO\basicIO.h" // console text- & number-IO & LED control

/*************************************************************************************************\
*    DEFINITIONS application related
\*************************************************************************************************/

#include  "BallPlate.h"
#include  "auxSIOcomm.h"         //!< control related IO with PC via AuxSIO

#define   K_TESTSTEP      20     //!< wait 1 sec to start test step sequence

/*************************************************************************************************\
*    GLOBAL VARIABLES
\*************************************************************************************************/

UINT8  OpMode; //!< current run mode, also usable for TRACE32 timing analysis

/*************************************************************************************************\
*    EXTERNAL FUNCTION PROTOTYPES
\*************************************************************************************************/

UINT8 initSystem( SS_VAR *xAxis, SS_VAR *yAxis );

UINT8 commandInterpreter( SS_VAR *xAxis, SS_VAR *yAxis );

// Compute PWMcmd scaled in [us] from servo angle cmd in [rad]; xAxis +1, yAxis -1 sign !
UINT16 setPWMSignalAngle( SS_VAR *axis, int sign );

void   controlLoop( SS_VAR xAxis, SS_VAR yAxis, SS_PAR ssModel);

void   print_float( float var );                      // L's legacy stuff
void   print_TwoFloat( const float, const float );


/*************************************************************************************************\
*    LOCAL FUNCTION PROTOTYPES
\*************************************************************************************************/

static void  printHello( void );


/*************************************************************************************************\
*    MAIN
\*************************************************************************************************/

int  main (void)
/**
*  ************************************************************************************************
*  @brief  Initializes MCU peripherals & servos, adjusts/calibrates ballplate horizontal position, 
*          then runs controller & estimation loop.
*  @retval none
***************************************************************************************************
*/
{
  static const SS_PAR ssModel = { BB_SS_F, BB_SS_G, BB_SS_C, BB_SS_K, BB_SS_V, BB_SS_Q, BB_SS_R};
  /* uses model/observer/controller parameters from "BallPlate.h"     */

  static SS_VAR xAxis = { PLATE_XMAX_PXL, PLATE_XMIN_PXL,  0.0, 0.0,  0.0, 0.0,  0, 0, 0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 1500 }; 
  static SS_VAR yAxis = { PLATE_YMAX_PXL, PLATE_YMIN_PXL,  0.0, 0.0,  0.0, 0.0,  0, 0, 0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 1500 };

/*************************************************************************************************\
*    initialize peripherals & servos, print hello message, calibrate horizontal, run control loop 
\*************************************************************************************************/

  OpMode = initSystem( &xAxis, &yAxis );         // init board & servos

  printHello( );  

  OpMode = commandInterpreter( &xAxis, &yAxis ); // calibrate plate horizontal

  controlLoop( xAxis, yAxis, ssModel );          // state estimation & state feedback controller 

  print_string("\n\n\rExecution should NEVER get here ! STOPPED\n");
  _getch();

} // END main()



/*************************************************************************************************\
*    LOCAL FUNCTION DEFINITIONS
\*************************************************************************************************/

void printHello( )
{
// print hello message & selected estimator algorithm
  print_string( "\n\r\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\r" );
  print_string( "BallPlate Controller using " );

#ifdef        _SYS_2ND_ORDER_OBS_
  print_string( " 2nd Order Observer\n\n\r" );
#elif defined _SYS_3RD_ORDER_OBS_
  print_string( " 3rd Order Observer\n\n\r" );
#elif defined _SYS_3RD_ORDER_KF_
  print_string( " 3rd Order Kalman Filter\n\n\r" );
#elif defined _SYS_4TH_ORDER_EKF_
	print_string( " 4th Order Extended Kalman Filter\n\n\r" );
#else
	#error: Undefined estimation method !
#endif

// print board type / clock rate / SW version string 
  print_string( "on " TARGET_BOARD " board with " TARGET_MCU " MCU running at " );
  print_uint( BSP_CPU_ClkFreq() / 1000000UL );  // print CPU clock in MHz
  print_string( " MHz ");
  print_string( "\n\n\rProgram BallPlateMain.c " SW_VERSION "\n\n\r" );

} // end printHello()


/*************************************************************************************************\
*    END OF MODULE BallPlateMain.c
\*************************************************************************************************/
