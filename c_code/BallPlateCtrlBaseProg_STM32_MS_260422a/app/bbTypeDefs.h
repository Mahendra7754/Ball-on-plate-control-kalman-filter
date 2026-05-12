/**
*  ***********************************************************************
*  @file    bbTypeDefs.h
*  @author  mys (based on WS2014 Master's Project by I. Lienhardt)
*  @version V1.00
*  @brief   State space model and state vector estimation related data
*           structures.
*
*  @date    06/12/2016
**************************************************************************
*/
#ifndef __BB_TYPDEFS__
#define __BB_TYPDEFS__


#include "..\basicIO\mctDefs.h" // useful constants, macros, type shorthands 

#include "bbSSParams.h"

#define PI         ((float) 3.1415926)
#define CRAD2DEG   ((float) 180.0 / PI)
#define CDEG2RAD   (PI / (float) 180.0)

#ifndef MDIM
#define MDIM 1
#endif
/** Struct for state space parameters.
*
* The struct SS_PAR is a container for parameters of a state space model
* of the type:
*
*  x[k+1]   = (A - K^T *B ) * x[k] + B *V *r[k] + L *(y_meas[k] - y_est[k]
*
*  y_est[k] = C' * x[k] 
*/

typedef struct SS_PAR // state space model/observer & controller parameters
{
  const float F[ORDER][ORDER];    //!< Transition Matrix
  const float G[ORDER];           //!< Input Gain Matrix 
  const float C[ORDER];           //!< Measurement Matrix
  const float K[ORDER];           //!< Controller gain vector
  const float V;                  //!< Overall gain
  //const float L[ORDER];           //!< Observer/Kalman gain vector, doesn#t need to declare it here
	 float Q[ORDER];		//!< Process covarience
	 float R[1];									//!< measurement varience
} SS_PAR;


/** Struct for state space variables.
*
* SS_VAR is a container struct for variables of the BallPlate model.
*
* Only variables of the state space model, full state feedback and 
* observer/kalman filter should be members of this struct.
*/

typedef struct SS_VAR    //!< state space variables
{
  UINT16 posMax;         //!< max detectable image coordinate of ball [pxl]
  UINT16 posMin;         //!< min detectable image coordinate of ball [pxl]
  
 /** Current [k] state vector.
   *  index 0: position of ball in [m]
   *  index 1: speed    of ball in [m/s]
   *  index 2: servo arm angle  in [rad]   (ONLY FOR ORDER == 3)
  */
  float  ssvEst_k[ORDER];  

 /** Predicted [k+1] state vector.
   *  index 0: position of ball in [m]
   *  index 1: speed    of ball in [m/s]
   *  index 2: servo arm angle  in [rad]   (ONLY FOR ORDER == 3)
  */  
  float  ssvEst_k1[ORDER];
   
  // ball positions in [pxl]
  short  xPosMeasPxl;             //!< current measured position [pxl]
  short  xPosRefPxl;              //!< reference position        [pxl]
  short  xPosOldPxl;              //!< last measured position    [pxl]
  INT16  xDotDifPxl;              //!< num diff  speed in        [pxl/s]

  // ball positions in [m]
  float  xPosMeasM;               //!< current measured position [m]
  float  xPosRefM;                //!< reference position        [m]
  float  xPosEstM;                //!< estimated position        [m]
  float  xDotEstMS;               //!< estimated speed in        [m/s]
  
	float  servoAngleCmdRad;        //!< servo angle command in [rad]
  INT16  servoAngleCalOffsetPWM;  //!< servo angle horizontal calibration offset [us]  
  UINT16 servoAngleCmdPWM;        //!< servo angle command PWM pulse width in [us]

  float  plateAngleRad;           //!< ball plate angle in [rad]
	
	float P_k[ORDER][ORDER]; 				// initial estimation error covarience matrix
	float P_k1[ORDER][ORDER]; 			// extrapolated error covarience matrix
	float Kf[ORDER][MDIM];								// Kalman gain vector
	
	float theta_rad;										// servo angle in rad
	float delta_theta_rad;									// bias in rad
	
	float cos_term; 
	

	
	float Fcparams[5];
	float Gcparams[5];
	float Fnl4[ORDER][ORDER];   // or [4][4]
	float Gnl4[ORDER];          // or [4]
	
	float yMeas[MDIM];
	
	//float L[3]; // kalman gain vector
	
} SS_VAR;

#endif
