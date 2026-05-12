/**
*  ************************************************************************************************
*  @file    Controller.c
*  @author  mys (based on WS2014 Master's Project by I. Lienhardt) 
*  @version V1.00
*  @brief   Full state feedback controller and observer for ballplate using
*           2nd/3rd order model.
*
*  Modified
*  161109   mys   new comp. flags INVERT_SERVO_XY to +/- servo turning sense
*  161221   mys   jerks -> XY.xDotEstMS init CORRECTLY; T, MAX_SERVO_ANGLE_RAD
*  260318   mys   function controlLoop() moved here from module BallPlateMain.c
*  260426   ms    updated code for Extended Kalman Filter
*
*  @date    26/04/2026
***************************************************************************************************
*/
/*************************************************************************************************\
\*************************************************************************************************/

/*************************************************************************************************\
*    HEADERFILES for runtime environment
\*************************************************************************************************/

#include  <stdio.h>              // C-StdLib sprintf(), scanf() prototypes
#include  <stdint.h>             // std C types for portability 6 MISRA compliance

#include  "stm32f10x.h"          // STM32F10x Peripherals Definitions

#include  "platform_cfg.h"       // target specific peripherals config
#include  "bsp.h"                // low level IO, clock & int defs for uC/OS

#include  "..\basicIO\basicIO.h" // console text- & number-IO & LED control
#include <math.h>
#include "BallplateKFlib.h"

/*************************************************************************************************\
*    DEFINITIONS application related
\*************************************************************************************************/

#include "BallPlate.h"           //!< applic. specific defs, structs, plus model/observer/controller
#include "auxSIOcomm.h"          //!< control related IO with PC via AuxSIO

#define   K_INITWAIT       5     //!< wait #of cycles for stable measurements

//#define kga   (-0.6f * 9.81f * (0.03f / 0.2f) * 1380.0f * (3.1415926f / 180.0f)) // model acc. const. scaled for pixels and degrees
//#define kga (-0.6f*9.81f*(0.030f/0.20f))
#define kga (float)(-0.8829)																				// model acc. const. scaled for meter/s� per radian
#define as    (1.0f / 0.2f) // servo time const

/*************************************************************************************************\
*    GLOBAL VARIABLES
\*************************************************************************************************/

extern UINT8  OpMode; //!< current run mode, also usable for TRACE32 timing analysis


/*************************************************************************************************\
*    EXTERNAL FUNCTION PROTOTYPES
\*************************************************************************************************/

// Compute PWMcmd scaled in [us] from servo angle cmd in [rad]; xAxis +1, yAxis -1 sign !
UINT16  setPWMSignalAngle( SS_VAR *axis, int sign );

/*************************************************************************************************\
*    LOCAL FUNCTION PROTOTYPES
\*************************************************************************************************/

//static void controlLoop( SS_VAR xAxis, SS_VAR yAxis, SS_PAR SSmodel);

void StateObserver( SS_VAR *xAxis, SS_VAR *yAxis, SS_PAR *ssModel );
void StateController( SS_VAR *xAxis, SS_VAR *yAxis, SS_PAR *ssModel );

/*************************************************************************************************\
*    LOCAL FUNCTION DEFINITIONS
\*************************************************************************************************/

void controlLoop( SS_VAR xAxis, SS_VAR yAxis, SS_PAR SSmodel )
{
  static UINT32 kSampleIndex = 0;            //!< sample index, controller active
  static UINT8  auxSIORxBuf[AUX_RX_BUFSIZE]; //!< receive buffer for measurement data from PC	

  OpMode = OPMODE_RECEIVE_MEASUREMENTS;
	
  while ( TRUE ) // do forever  -  real-time control & estimation loop
  {
#ifdef DEBUG_MSG_ON
    print_string("\n\r");
#ifdef _SYS_2ND_ORDER_OBS_
    _putch('2');
#elif defined _SYS_3RD_ORDER_OBS_
    _putch('3');
#elif defined _SYS_3RD_ORDER_KF_
    _putch('3');
#elif defined _SYS_4TH_ORDER_EKF_
		_putch('4');
#endif
    _putch(' ');
#endif

    // receive image processing data set, i.e. ball position measurements, from PC
    BSP_TurnOnOffLED( oneCompleteLoop, LED_ON );   // LEDs 12&15 ON ???

    receiveMeasurements( auxSIORxBuf );             // wait for NEW measurements from PC 

    BSP_TurnOnOffLED( serialReception, LED_OFF );   // LEDs 12&15 ON ???


    if ( checkReceivedOK( auxSIORxBuf ) ) // checksum verified, received PC message is OK
    {
      // save last/old position measurement values
      xAxis.xPosOldPxl = xAxis.xPosMeasPxl;
      yAxis.xPosOldPxl = yAxis.xPosMeasPxl;
 
      // 're-assemble' UINT16 coordinate values from NEW RECEIVED PC message
      xAxis.xPosMeasPxl = (auxSIORxBuf[1] << 8) + auxSIORxBuf[2];
      yAxis.xPosMeasPxl = (auxSIORxBuf[3] << 8) + auxSIORxBuf[4];
      xAxis.xPosRefPxl  = (auxSIORxBuf[5] << 8) + auxSIORxBuf[6];
      yAxis.xPosRefPxl  = (auxSIORxBuf[7] << 8) + auxSIORxBuf[8];

      // rescale current and reference position from [pxl] to [m]
      xAxis.xPosMeasM   = xAxis.xPosMeasPxl / PIXEL_PER_METER;
      yAxis.xPosMeasM   = yAxis.xPosMeasPxl / PIXEL_PER_METER;
      xAxis.xPosRefM    = xAxis.xPosRefPxl  / PIXEL_PER_METER;
      yAxis.xPosRefM    = yAxis.xPosRefPxl  / PIXEL_PER_METER;

      kSampleIndex++;
      
      if ( kSampleIndex < K_INITWAIT ) // keep everything static in first N control cycles
      {
        xAxis.xPosOldPxl   = xAxis.xPosMeasPxl; // so that 1st xDotEstMS == 0
        xAxis.ssvEst_k[0]  = xAxis.xPosMeasM;
        xAxis.ssvEst_k[1]  = 0.0;
        xAxis.ssvEst_k1[0] = xAxis.xPosMeasM;   // ERROR in 1st cycle for [k+1] values !!!!
        xAxis.ssvEst_k1[1] = 0.0;               // see INIT-JERK logfiles 170126 & 170206

        yAxis.xPosOldPxl   = yAxis.xPosMeasPxl; // so that 1st y.xDotEstMS == 0
        yAxis.ssvEst_k[0]  = yAxis.xPosMeasM;
        yAxis.ssvEst_k[1]  = 0.0;
        yAxis.ssvEst_k1[0] = yAxis.xPosMeasM;
        yAxis.ssvEst_k1[1] = 0.0;

#if ORDER == 3        
        xAxis.ssvEst_k[2]  = 0.0;
        yAxis.ssvEst_k[2]  = 0.0;
        xAxis.ssvEst_k1[2] = 0.0;
        yAxis.ssvEst_k1[2] = 0.0;
#endif
				
#if ORDER == 4 // ms
				

				xAxis.ssvEst_k[0]  = xAxis.xPosMeasM;
        xAxis.ssvEst_k[1]  = 0.0;
				
        xAxis.ssvEst_k1[0] = xAxis.xPosMeasM;   // ERROR in 1st cycle for [k+1] values !!!!
        xAxis.ssvEst_k1[1] = 0.0; 
				
				yAxis.xPosOldPxl   = yAxis.xPosMeasPxl; // so that 1st y.xDotEstMS == 0
        
				yAxis.ssvEst_k[0]  = yAxis.xPosMeasM;
        yAxis.ssvEst_k[1]  = 0.0;
				
        yAxis.ssvEst_k1[0] = yAxis.xPosMeasM;
        yAxis.ssvEst_k1[1] = 0.0;
				
        xAxis.ssvEst_k[2]  = 0.0;
        yAxis.ssvEst_k[2]  = 0.0;
				
        xAxis.ssvEst_k1[2] = 0.0;
        yAxis.ssvEst_k1[2] = 0.0;
				
				xAxis.ssvEst_k1[3] = 0.0;
        yAxis.ssvEst_k1[3] = 0.0;
#endif
      } // end keep everything static in first N control cycles

		if (kSampleIndex == 1) // covariace and stae initialization at starting for both x&y axis
		{
			
			int i, j;
			
		// X Axis state initialization
				xAxis.ssvEst_k[0] = xAxis.xPosMeasM;
				xAxis.ssvEst_k[1] = 0.0;
				xAxis.ssvEst_k[2] = 0.0;
				xAxis.ssvEst_k[3] = 0.0;

				xAxis.ssvEst_k1[0] = xAxis.xPosMeasM;
				xAxis.ssvEst_k1[1] = 0.0;
				xAxis.ssvEst_k1[2] = 0.0;
				xAxis.ssvEst_k1[3] = 0.0;

		 //Covariance initialization
				xAxis.P_k[0][0] = 1.0;
				xAxis.P_k[1][1] = 200.0;
				xAxis.P_k[2][2] = 50.0;
				xAxis.P_k[3][3] = 10.0;
			
				xAxis.P_k1[0][0] = 1.0;
				xAxis.P_k1[1][1] = 200.0;
				xAxis.P_k1[2][2] = 50.0;
				xAxis.P_k1[3][3] = 10.0;

		 //zero off-diagonal
				for (i=0;i<4;i++){
						for (j=0;j<4;j++){
								if (i != j) 
								{
									xAxis.P_k[i][j] = 0.0;
								}
														}
												}
				for (i=0;i<4;i++){
						for (j=0;j<4;j++){
								if (i != j) 
								{
									xAxis.P_k1[i][j] = 0.0;
							}
												}
															}

			//Y Axis- state initialization
				yAxis.ssvEst_k[0] = yAxis.xPosMeasM;
				yAxis.ssvEst_k[1] = 0.0;
				yAxis.ssvEst_k[2] = 0.0;
				yAxis.ssvEst_k[3] = 0.0;

				yAxis.ssvEst_k1[0] =yAxis.xPosMeasM;
				yAxis.ssvEst_k1[1] = 0.0;
				yAxis.ssvEst_k1[2] = 0.0;
				yAxis.ssvEst_k1[3] = 0.0;

		 //Covariance initialization
				yAxis.P_k[0][0] = 1.0;
				yAxis.P_k[1][1] = 200.0;
				yAxis.P_k[2][2] = 50.0;
				yAxis.P_k[3][3] = 10.0;
				
				yAxis.P_k1[0][0] = 1.0;
				yAxis.P_k1[1][1] = 200.0;
				yAxis.P_k1[2][2] = 50.0;
				yAxis.P_k1[3][3] = 10.0;

		 //zero off-diagonal
				for (i=0;i<4;i++){
						for (j=0;j<4;j++){
								if (i != j) 
									{yAxis.P_k[i][j] = 0.0;
									}
								}
							}
				for (i=0;i<4;i++){
						for (j=0;j<4;j++){
								if (i != j) 
									{yAxis.P_k1[i][j] = 0.0;
									}	
																	}
															}
				// 
		// kalman gain vector initialization
				
				xAxis.Kf[0][0] = 0;
				xAxis.Kf[1][0] = 0;
				xAxis.Kf[2][0] = 0;
				xAxis.Kf[3][0] = 0;
									
				yAxis.Kf[0][0] = 0;
				yAxis.Kf[1][0] = 0;
				yAxis.Kf[2][0] = 0;
				yAxis.Kf[3][0]= 0;						
		}  
			// compute ball speed by simple backward difference of position measurements
    //xAxis.xDotEstMS  = (xAxis.xPosMeasPxl - xAxis.xPosOldPxl) / T;    // scaled in pxl/s ??? MS
    //yAxis.xDotEstMS  = (yAxis.xPosMeasPxl - yAxis.xPosOldPxl) / T;
      xAxis.xDotDifPxl = (xAxis.xPosMeasPxl - xAxis.xPosOldPxl) / T;    // scaled in pxl/s
      yAxis.xDotDifPxl = (yAxis.xPosMeasPxl - yAxis.xPosOldPxl) / T;

      OpMode = OPMODE_CLOSED_LOOP_CONTROL;

      if ( OpMode == OPMODE_CLOSED_LOOP_CONTROL )
      {
      // state estimation & state feedback control -> compute new servo actuator angle
        BSP_TurnOnOffLED( calcFSFAndObs, LED_ON );
				
      //bbSSControlAndObserver( &xAxis, &yAxis, &SSmodel );
        StateObserver  ( &xAxis, &yAxis, &SSmodel );
        StateController( &xAxis, &yAxis, &SSmodel );

        BSP_TurnOnOffLED( calcFSFAndObs, LED_OFF );
      
        // convert servo actuator angleCmd to PWM pulse duration in [us] 
        BSP_TurnOnOffLED( calcPWM, LED_ON );
        TIM_XAXIS_RSTVAL = setPWMSignalAngle( &xAxis, +1 ); // unified version
        TIM_YAXIS_RSTVAL = setPWMSignalAngle( &yAxis, -1 ); // unified version   
        BSP_TurnOnOffLED( calcPWM, LED_OFF );

      } // end if OPMODE_CLOSED_LOOP_CONTROL

      else // unknown OpMode
      {
         print_string("\n\n\rERROR: Illegal OpMode in Control Loop =");
         print_int( OpMode );
         print_string("\n\r");
         _getch();
      }


      // send control related data to PC for logging
      BSP_TurnOnOffLED( serialSend, LED_ON );
      sendLogData( &xAxis, &yAxis );
      BSP_TurnOnOffLED( serialSend, LED_OFF );

      BSP_TurnOnOffLED( oneCompleteLoop, LED_OFF );


#ifdef DEBUG_MSG_ON
      print_TwoFloat(xAxis.xPosRefM,  yAxis.xPosRefM);
      print_TwoFloat(xAxis.xPosMeasM, yAxis.xPosMeasM);
      print_TwoFloat(xAxis.ssvEst_k[0], yAxis.ssvEst_k[0]);
      print_TwoFloat(xAxis.ssvEst_k[1], yAxis.ssvEst_k[1]);

#if ORDER == 3
      //print_TwoFloat( xAxis.ssvEst_k[2], yAxis.ssvEst_k[2]);
#endif
      //print_TwoFloat(xAxis.xPosMeasM - xAxis.posEst, yAxis.xPosMeasM - yAxis.posEst);
      print_TwoFloat(xAxis.servoAngleCmdRad, yAxis.servoAngleCmdRad);
#endif
    } 
    else // checksum verify FAILED, data from PC are NOT OK
    {
      _putch( '!' );       // indicate auxSIO communication error
      _putch( ASC_BELL );  // by '!' and a beep !
      
#ifdef DEBUG_MSG_ON
      print_TwoFloat(xAxis.xPosRefM, yAxis.xPosRefM);
      print_string("   NaN      NaN   ");
      print_TwoFloat(xAxis.ssvEst_k[0], yAxis.ssvEst_k[0]);
      print_TwoFloat(xAxis.ssvEst_k[1], yAxis.ssvEst_k[1]);

#if ORDER == 3
      //print_TwoFloat( xAxis.ssvEst_k[2], yAxis.ssvEst_k[2]);
#endif
      //print_TwoFloat(xAxis.xPosMeasM - xAxis.posEst, yAxis.xPosMeasM - yAxis.posEst);
      print_TwoFloat(xAxis.servoAngleCmdRad, yAxis.servoAngleCmdRad);
#endif
    } // END if( CheckSumOK ...)
  } // END while( TRUE ) // real-time loop - do forever

} // END of controlLoop( )


void StateObserver( SS_VAR *xAxis, SS_VAR *yAxis, SS_PAR *ssModel )
/**
 * @brief Compute new estimated state.
 *
 *  State space model:
 *    x[k+1]   = (F - K`*G)*x[k] + G*V*u[k] + L*(y_meas[k] - y_est[k])
 *  with
 *    y_est[k] = C' * x[k]
 *
 * @param[in] xAxis    Pointer to state variables struct for X-Axis
 * @param[in] yAxis    Pointer to state variables struct for Y-Axis
 * @param[in] ssModel  Pointer to struct with state space model parameters 
 *
 */
{
		int i;
		int j;
  static float xEstError;
  static float yEstError;
	    float xMeas[1] ;
    float yMeas[1] ;
  OpMode = OPMODE_STATE_OBSERVER;	

  // store last predicted state to current state vector
#if ORDER == 2
  xAxis->ssvEst_k[0] = xAxis->ssvEst_k1[0];
  yAxis->ssvEst_k[0] = yAxis->ssvEst_k1[0];

  xAxis->ssvEst_k[1] = xAxis->ssvEst_k1[1];
  yAxis->ssvEst_k[1] = yAxis->ssvEst_k1[1];

#elif ORDER ==3
	
  xAxis->ssvEst_k[0] = xAxis->ssvEst_k1[0];
  xAxis->ssvEst_k[1] = xAxis->ssvEst_k1[1];
  xAxis->ssvEst_k[2] = xAxis->ssvEst_k1[2];

  yAxis->ssvEst_k[0] = yAxis->ssvEst_k1[0];
  yAxis->ssvEst_k[1] = yAxis->ssvEst_k1[1];
  yAxis->ssvEst_k[2] = yAxis->ssvEst_k1[2];
	
#elif ORDER ==4
	/*xAxis->xPosOldPxl   = xAxis->xPosMeasPxl; // so that 1st xDotEstMS == 0
	xAxis->ssvEst_k[0] = xAxis->ssvEst_k1[0];
  yAxis->ssvEst_k[0] = yAxis->ssvEst_k1[0];

  xAxis->ssvEst_k[1] = xAxis->ssvEst_k1[1];
  yAxis->ssvEst_k[1] = yAxis->ssvEst_k1[1];
	
	xAxis->ssvEst_k[2] = xAxis->ssvEst_k1[2];
  yAxis->ssvEst_k[2] = yAxis->ssvEst_k1[2];

  xAxis->ssvEst_k[3] = xAxis->ssvEst_k1[3];
  yAxis->ssvEst_k[3] = yAxis->ssvEst_k1[3];
	*/ //critical bug
	
	
#elif ORDER > 4
  #error "Illegal value for system ORDER ! Valid (as yet):  2, 3 or 4"
#endif
/*************************************************************************\
  Observer / Kalman Filter Calculations
	x[k+1] = F*x[k] + G*u[k] + L*(y_meas[k] - y_est[k])
	u[k] = servoAngleCmdRad
\*************************************************************************/

  xEstError = xAxis->xPosMeasM - xAxis->ssvEst_k[0];
  yEstError = yAxis->xPosMeasM - yAxis->ssvEst_k[0];

#if ORDER == 2
  xAxis->ssvEst_k1[0]  = ssModel->F[0][0] * xAxis->ssvEst_k[0]
                       + ssModel->F[0][1] * xAxis->ssvEst_k[1]
                       + ssModel->G[0]    * xAxis->servoAngleCmdRad
                       + ssModel->L[0]    * xEstError;

  xAxis->ssvEst_k1[1]  = ssModel->F[1][0] * xAxis->ssvEst_k[0]
                       + ssModel->F[1][1] * xAxis->ssvEst_k[1]
                       + ssModel->G[1]    * xAxis->servoAngleCmdRad
                       + ssModel->L[1]    * xEstError;

  yAxis->ssvEst_k1[0]  = ssModel->F[0][0] * yAxis->ssvEst_k[0]
                       + ssModel->F[0][1] * yAxis->ssvEst_k[1]
                       + ssModel->G[0]    * yAxis->servoAngleCmdRad
                       + ssModel->L[0]    * yEstError;

  yAxis->ssvEst_k1[1]  = ssModel->F[1][0] * yAxis->ssvEst_k[0]
                       + ssModel->F[1][1] * yAxis->ssvEst_k[1]
                       + ssModel->G[1]    * yAxis->servoAngleCmdRad
                       + ssModel->L[1]    * yEstError;

#elif ORDER == 3
  xAxis->ssvEst_k1[0] += ssModel->F[0][2] * xAxis->ssvEst_k[2];
  xAxis->ssvEst_k1[1] += ssModel->F[1][2] * xAxis->ssvEst_k[2];

  xAxis->ssvEst_k1[2]  = ssModel->F[2][0] * xAxis->ssvEst_k[0]
                       + ssModel->F[2][1] * xAxis->ssvEst_k[1]
                       + ssModel->F[2][2] * xAxis->ssvEst_k[2]
                       + ssModel->G[2]    * xAxis->servoAngleCmdRad
                       + ssModel->L[2]    * xEstError;

  yAxis->ssvEst_k1[0] += ssModel->F[0][2] * yAxis->ssvEst_k[2];
  yAxis->ssvEst_k1[1] += ssModel->F[1][2] * yAxis->ssvEst_k[2];

  yAxis->ssvEst_k1[2]  = ssModel->F[2][0] * yAxis->ssvEst_k[0]
                       + ssModel->F[2][1] * yAxis->ssvEst_k[1]
                       + ssModel->F[2][2] * yAxis->ssvEst_k[2]
                       + ssModel->G[2]    * yAxis->servoAngleCmdRad
                       + ssModel->L[2]    * yEstError;
#elif ORDER == 4 // ms main calculation loop
    
   


	xAxis->theta_rad		 = (xAxis->ssvEst_k[2])*CDEG2RAD; // servo angle in radians
	yAxis->theta_rad		 = (yAxis->ssvEst_k[2])*CDEG2RAD ;// servo angle in radians
	
	xAxis->delta_theta_rad		 = (xAxis->ssvEst_k[3])*CDEG2RAD ; // bias in radians
	yAxis->delta_theta_rad		 = (yAxis->ssvEst_k[3])*CDEG2RAD ; // bias in radians
	

	
	// variables to calculate F& G matrix
	
	// F matrix params
	xAxis->Fcparams[0] = T;
	xAxis->Fcparams[1] =kga;
	xAxis->Fcparams[2] =xAxis->theta_rad;
	xAxis->Fcparams[3] =xAxis->delta_theta_rad;
	xAxis->Fcparams[4] =as;
	
	yAxis->Fcparams[0] = T;
	yAxis->Fcparams[1] =kga;
	yAxis->Fcparams[2] =yAxis->theta_rad;
	yAxis->Fcparams[3] =yAxis->delta_theta_rad;
	yAxis->Fcparams[4] =as;
	
	// G Matrix params
	xAxis->Gcparams[0] = T;
	xAxis->Gcparams[1] =kga;
	xAxis->Gcparams[2] =xAxis->theta_rad;
	xAxis->Gcparams[3] =xAxis->delta_theta_rad;
	xAxis->Gcparams[4] =as;
	
	yAxis->Gcparams[0] = T;
	yAxis->Gcparams[1] =kga;
	yAxis->Gcparams[2] =yAxis->theta_rad;
	yAxis->Gcparams[3] =yAxis->delta_theta_rad;
	yAxis->Gcparams[4] =as;
	

	// calculating Fmat by calling C function	
	KFcomputeFMat(xAxis->Fnl4, xAxis->Fcparams);
	KFcomputeFMat(yAxis->Fnl4, yAxis->Fcparams);
		
		

	// calculating Gmat by calling C function
	KFcomputeGMat(xAxis->Gnl4, xAxis->Gcparams);
	KFcomputeGMat(yAxis->Gnl4, yAxis->Gcparams);
		  
  //  start from previous corrected state
for (i = 0; i < 4; i++) {
    xAxis->ssvEst_k1[i] = xAxis->ssvEst_k[i];
    yAxis->ssvEst_k1[i] = yAxis->ssvEst_k[i];
}

 
	// State Prediction step
	KFpredictState(xAxis->ssvEst_k1,xAxis->Fnl4,xAxis->Gnl4,xAxis->servoAngleCmdRad);
	KFpredictState(yAxis->ssvEst_k1,yAxis->Fnl4,yAxis->Gnl4,yAxis->servoAngleCmdRad);	
		
       

	
  // covariance prediction
  KF_PTUPD(xAxis->P_k, xAxis->Fnl4, ssModel->Q);
  KF_PTUPD(yAxis->P_k, yAxis->Fnl4, ssModel->Q);
	
	  for (i= 0; i < 4; i++) {
      for (j = 0; j < 4; j++) {
				xAxis->P_k1[i][j] = xAxis->P_k[i][j];
				yAxis->P_k1[i][j] = yAxis->P_k[i][j];
      }
  }
	 
	 // Kalman gain
	 KFcomputeKMat(xAxis->Kf,xAxis->P_k1,ssModel->R);
	 KFcomputeKMat(yAxis->Kf,yAxis->P_k1,ssModel->R);
	
	 // covariance update
	 KF_PMUPD(xAxis->P_k,xAxis->Kf,xAxis->P_k1);
	 KF_PMUPD(yAxis->P_k,yAxis->Kf,yAxis->P_k1);
	
	  // measurement vector
  xMeas[0] = xAxis->xPosMeasM;
  yMeas[0] = yAxis->xPosMeasM;
	
	// state update
	KF_xMUPD(xAxis->ssvEst_k1,xAxis->Kf,xMeas);
	KF_xMUPD(yAxis->ssvEst_k1,yAxis->Kf,yMeas);

  // save corrected state for next cycle
for (i = 0; i < 4; i++) {
    xAxis->ssvEst_k[i] = xAxis->ssvEst_k1[i];
    yAxis->ssvEst_k[i] = yAxis->ssvEst_k1[i];
}
	
	 //
	
#else
  #error "Illegal value for system ORDER ! Valid values: 2 or 3"
#endif

} // end StateObserver()


void StateController( SS_VAR *xAxis, SS_VAR *yAxis, SS_PAR *ssModel )
/**
 * @brief Compute new control variable servoAngleCmdRad == u8.
 *
 *  State space model:
 *    x[k+1]   = (F - K`*G)*x[k] + G*V*u[k] + L*(y_meas[k] - y_est[k])
 *  with
 *    y_est[k] = C' * x[k]
 *
 * @param[in] xAxis    Pointer to state variables struct for X-Axis
 * @param[in] yAxis    Pointer to state variables struct for Y-Axis
 * @param[in] ssModel  Pointer to struct with state space model parameters 
 *
 */
{
  OpMode = OPMODE_STATE_CONTROLLER;

/*************************************************************************\
  Full state feedback controller
	ThetaServoCmd = xPosRefM * V - K' * xEst
\*************************************************************************/
 
  xAxis->servoAngleCmdRad = xAxis->xPosRefM * ssModel->V
                     - ssModel->K[0] * xAxis->ssvEst_k[0] 	// 20260412- ms changed xPosMeasM to xAxis->ssvEst_k[0]
                     - ssModel->K[1] * xAxis->ssvEst_k[1];

  yAxis->servoAngleCmdRad = yAxis->xPosRefM * ssModel->V
                     - ssModel->K[0] * yAxis->ssvEst_k[0] 				// 20260412- ms changed xPosMeasM to xAxis->ssvEst_k[0]
                     - ssModel->K[1] * yAxis->ssvEst_k[1];
										 
//#if ORDER == 3 // using 3rd order controller
  xAxis->servoAngleCmdRad = xAxis->servoAngleCmdRad - ssModel->K[2] * xAxis->ssvEst_k[2];
  yAxis->servoAngleCmdRad = yAxis->servoAngleCmdRad - ssModel->K[2] * yAxis->ssvEst_k[2];
	//#if ORDER == 4
  xAxis->servoAngleCmdRad = xAxis->servoAngleCmdRad - ssModel->K[3] * xAxis->ssvEst_k[3];
  yAxis->servoAngleCmdRad = yAxis->servoAngleCmdRad - ssModel->K[3] * yAxis->ssvEst_k[3];

	
//#endif

#ifdef INVERT_SERVO_X
  xAxis->servoAngleCmdRad = - xAxis->servoAngleCmdRad;
#endif
#ifdef INVERT_SERVO_Y
  yAxis->servoAngleCmdRad = - yAxis->servoAngleCmdRad;
#endif

  // compute ballplate angle in [rad]  !! SIMPLIFIED/LINEARIZED EQN !!
  xAxis->plateAngleRad = (xAxis->servoAngleCmdRad * SERVO_ARM_LENGTH ) / HALF_BOARD_LENGTH ;
  yAxis->plateAngleRad = (yAxis->servoAngleCmdRad * SERVO_ARM_LENGTH ) / HALF_BOARD_LENGTH ;

  // Limit servo angle  |servoAngleCmdRad| <= MAX_SERVO_ANGLE_RAD
  if ( xAxis->servoAngleCmdRad > MAX_SERVO_ANGLE_RAD )
  { 
    xAxis->servoAngleCmdRad =    MAX_SERVO_ANGLE_RAD;
  }

  if( xAxis->servoAngleCmdRad < -MAX_SERVO_ANGLE_RAD )
  {
    xAxis->servoAngleCmdRad =   -MAX_SERVO_ANGLE_RAD;
  }

  if ( yAxis->servoAngleCmdRad > MAX_SERVO_ANGLE_RAD ) 
  {
    yAxis->servoAngleCmdRad =    MAX_SERVO_ANGLE_RAD;
  }
	
  if( yAxis->servoAngleCmdRad < -MAX_SERVO_ANGLE_RAD ) 
  {
    yAxis->servoAngleCmdRad =   -MAX_SERVO_ANGLE_RAD;
  }

} // end StateController()


/************************************************************************\
*    END OF MODULE Controller.c
\************************************************************************/
