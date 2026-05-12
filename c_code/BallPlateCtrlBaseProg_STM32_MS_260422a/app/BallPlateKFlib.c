/**
*  ***********************************************************************
*  @file    BallPlateKFlib.c
*  @author  B. Mysliwetz
*  @version V1.10
*  @brief   C function templates for XXX model simulation & Kalman Filter
*           estimation, usable via MATLAB Shared Library Calling (CALLLIB).
*
*  IMPORT C-functions from 'BallPlateKFlib.mexw64' into MATLAB program via :
*
*           if not(libisloaded('BallPlateKFlib'))
*            %addpath( fullfile(matlabroot,'extern','examples','shrlib') );
*             addpath( 'C:\myMatLab\BallPlatSim\' );
*             loadlibrary('BallPlateKFlib');
*           end
*           libfunctions BallPlateKFlib -full    % print lib function properties
*
*
*  USAGE of C-functions in MATLAB program via 'calllib( )' e.g.:
*
*           FParams = single( [ LLTest,  ThetaElKF_k, OmegaElKF_k ] );
*           FMat32 = ( calllib( 'BallPlateKFlib', 'KFcomputeFMat', FMat32', FParams ) )';
*           calllib( 'BallPlateKFlib', 'print44Mat', FMat32' );
* 
*
*  Changes:
*  09.10.2013 mys     Initial version
*  14.12.2016 mys     Adapted for BallPlate
*  12.12.2019 mys     Adapted for R2019
*  07.12.2020 mys     FLOAT type/precision configurable via #define
*                      
*  @date    07/12/2020
**************************************************************************
*/

#define  MATLAB_USE

#include <math.h>

#ifdef MATLAB_USE
//#ifndef  STM32_VERSION
#define  EXPORT_FCNS
#include "shrhelp.h"
#include <mex.h>  /* only needed because of mexFunction below and mexPrintf */
#endif

#include "BallPlateKFlib.h"



/**
*  ***********************************************************************
*  @brief  Computes time varying elements of motor model transition matrix.
*
*  MATLAB Code % FMat = I + A*T = eye(4) + dfdx*T
*
*  FMat = [ (1-K*T),    0,  LL*T*sin(Theta_k), LL*Omega_k*T*cos(Theta_k)
*                0, (1-K*T), -LL*T*cos(Theta_k), LL*Omega_k*T*sin(Theta_k)
*                0,       0,                  1,                       0
*                0,       0,                  T,                       1 ]
*
*  @param  FMat   : 4x4 system transition matrix / electrical motor model 
*  @param  Params : 3x1 vector [ LL*T, ThetaEst_k, OmegaEst_k ]
*  @retval none
**************************************************************************
*/


/**
 * @brief Computes the Jacobian F matrix (4x4)
 * MATLAB: Fnl4 = [ 1 T 0 0; 0 1 (kga*c*T) (kga*c*T); 0 0 (1-as*T) 0; 0 0 0 1 ]
 */
EXPORTED_FUNCTION void KFcomputeFMat(FLOAT FMat[][NDIM], FLOAT FParams[]) {
    FLOAT T = FParams[0]; // Sampling time
    FLOAT kga = FParams[1]*(3.14/180); // Ball-plate gain
    FLOAT theta = FParams[2]; // xState(3) in radians
    FLOAT bias = FParams[3]; // xState(4) in radians
    FLOAT as = FParams[4]; // Servo pole

  
        FLOAT c = cos(theta + bias);

    // Initialize all to 0 first (or identity)
    for (int i = 0; i < NDIM; i++)
        for (int j = 0; j < NDIM; j++) FMat[i][j] = 0.0;

    FMat[0][0] = 1.0; FMat[0][1] = T;
    FMat[1][1] = 1.0; FMat[1][2] = kga * c * T; FMat[1][3] = kga * c * T;
    FMat[2][2] = 1.0 - (as * T);
    FMat[3][3] = 1.0;
}

/**
 * @brief Computes the G matrix (4x1)
 * MATLAB: Gnl4 = [ 0; kga*c*T22*as; as*T - as^2*T22; 0 ]
 */
EXPORTED_FUNCTION void KFcomputeGMat(FLOAT GMat[NDIM], FLOAT GParams[]) {
    FLOAT T = GParams[0];
    FLOAT kga = GParams[1]*(3.14/180);
    FLOAT theta = GParams[2];
    FLOAT bias = GParams[3];
    FLOAT as = GParams[4];

    FLOAT T22 = (T * T) / 2.0;
    FLOAT c = cos(theta + bias);

    GMat[0] = 0.0;
    GMat[1] = kga * c * T22 * as;
    GMat[2] = (as * T) - (as * as * T22);
    GMat[3] = 0.0;
}



/**
*  ***********************************************************************
*  @brief  Computes time extrapolated state xk1 = FMat*xk + BMat*uk.
*  @param  xk   : 4x1 state vector in at [k] -> out predicted at [k+1]
*  @param  FMat : 4x4 system transition matrix / electrical motor model 
*  @param  b    : non-zero elements of BMat11 == BMat22 == b 
*  @param  uk   : 2x1 control input vector at [k], ie voltages Ua Ub
*  @retval none
**************************************************************************
*/
/**
 * @brief Prediction: x = F*x + G*u
 */

EXPORTED_FUNCTION void KFpredictState(FLOAT xk[], FLOAT Phi[][NDIM], FLOAT G[], FLOAT uk) {
    FLOAT xpred[NDIM];
    for (int i = 0; i < NDIM; i++) {
        FLOAT sum = 0.0;
        for (int j = 0; j < NDIM; j++) {
            sum += Phi[i][j] * xk[j];
        }
        // Use the G vector computed earlier
        xpred[i] = sum + G[i] * uk;
    }
    for (int i = 0; i < NDIM; i++) xk[i] = xpred[i];
}

/**
*  ***********************************************************************
*  @brief  Computes time updated covariance matrix P_k1 = Phi*P_k*Phi' + Q.
*  @param  PMat : 4x4 estim. error cov matrix in at sample [k], out at [k+1] 
*  @param  FMat : 4x4 system transition matrix / electrical motor model 
*  @param  QMat : 4x1 process noise cov matrix diagonal elements
*  @retval none
**************************************************************************
*/

EXPORTED_FUNCTION void KF_PTUPD( FLOAT P[][NDIM], FLOAT Phi[][NDIM], FLOAT Q[] )
{
    // P = Phi*P*Phi' + Q (Q diagonal)
  FLOAT T1[NDIM][NDIM];
  for(int i=0;i<NDIM;++i){
    for(int j=0;j<NDIM;++j){
      FLOAT s=0.0;
      for(int k=0;k<NDIM;++k) s+= Phi[i][k]*P[k][j];
      T1[i][j]=s;
    }
  }
  for(int i=0;i<NDIM;++i){
    for(int j=0;j<NDIM;++j){
      FLOAT s=0.0;
      for(int k=0;k<NDIM;++k) s+= T1[i][k]*Phi[j][k]; // Phi'
      P[i][j]=s;
    }
    P[i][i]+=Q[i];
  }
  // ...
} // end function KF_PTUPD( )





/**
*  ***********************************************************************
*  @brief  Computes KF gain matrix     KKF_k = PMat_k1*CMat' * CPCRinv.
*  @param  KMat  : out 4x2 KF gain matrix
*  @param  PMat  : in  4x4 estim. error cov matrix, extrapolated for [k+1] 
*  @param  RMat  : in  2x1 measurement noise cov matrix diagonal elements
*  @retval none
**************************************************************************
*/

EXPORTED_FUNCTION void KFcomputeKMat( FLOAT KMat[][MDIM], FLOAT PMat[][NDIM], FLOAT Rdiag[] )
{
    // H = [1 0 0 0], S = P(0,0)+R
  FLOAT S = PMat[0][0] + Rdiag[0];
  FLOAT invS = 1.0 / S;
  for(int i=0;i<NDIM;++i) KMat[i][0] = PMat[i][0] * invS;
    // ...
} // end function KFcomputeKMat( )



/**
*  ***********************************************************************
*  @brief  Computes inverse of C*P_k1*C' + R).
*  @param  CPCRinv: out 2x2 matrix, inverse of (C*P_k1*C' + R) 
*  @param  PMat   : in  4x4 estim. error cov matrix, extrapolated for [k+1] 
*  @param  RMat   : in  2x1 measurement noise cov matrix diagonal elements
*  @retval none
**************************************************************************
*/

EXPORTED_FUNCTION void KFcomputeCPCRinv( FLOAT CPCRinv[][MDIM], FLOAT PMat[][NDIM], FLOAT Rdiag[] )
{
  FLOAT S = PMat[0][0] + Rdiag[0];
  CPCRinv[0][0] = 1.0 / S;  // ...
} // end function KFcomputeCPCRinv( )


/**
*  ***********************************************************************
*  @brief  Computes KF gain matrix     KKF_k = PMat_k1*CMat' * CPCRinv.
*          needs intermediate result from KFcomputeCPCRinv( )
*  @param  KMat   : out 4x2 KF gain matrix
*  @param  PMat   : in  4x4 estim. error cov matrix, extrapolated for [k+1] 
*  @param  CPCRinv: 2x2 input matrix, inverse of (C*P*C' + R) 
*  @retval none
**************************************************************************
*/

EXPORTED_FUNCTION void KFcomputeKMat0( FLOAT KMat[][MDIM], FLOAT PMat[][NDIM], FLOAT CPCRinv[][MDIM] )
{
  for(int i=0;i<NDIM;++i) KMat[i][0] = PMat[i][0] * CPCRinv[0][0]; // ...
} // end function KFcomputeKMat0( )



/**
*  ***********************************************************************
*  @brief  Computes PMat measurement update P_k = P_k1 - KMat*CMat*P_k1.
*  @param  Pkout  : out 4x4 estim. error cov matrix, updated 
*  @param  KMat   : in  4x2 KF gain matrix
*  @param  Pk1in  : in  4x4 estim. error cov matrix, extrapolated for [k+1] 
*  @retval none
**************************************************************************
*/

EXPORTED_FUNCTION void KF_PMUPD( FLOAT Pkout[][NDIM], FLOAT KMat[][MDIM], FLOAT Pk1in[][NDIM] )
{
   // P = Pk1 - K*H*Pk1 ; H selects row 0
  for(int i=0;i<NDIM;++i){
    for(int j=0;j<NDIM;++j){
      Pkout[i][j] = Pk1in[i][j] - KMat[i][0]*Pk1in[0][j];
    }
  }// ...
} // end function KF_PMUPD( )



/**
*  ***********************************************************************
*  @brief  Computes xKF measurement update x_k = x_k1 + KMat*(ymeas-ypred).
*  @param  xKF    : in/out 4x1 state vector; in predicted, out updated 
*  @param  KMat   : in  4x2 KF gain matrix
*  @param  yMeas  : in  2x1 Measurement vector 
*  @retval none
**************************************************************************
*/

EXPORTED_FUNCTION void KF_xMUPD( FLOAT xKF[NDIM], FLOAT KMat[][MDIM], FLOAT yMeas[MDIM] )
{
  // yhat = xKF[0]
  FLOAT innov = yMeas[0] - xKF[0];
  for(int i=0;i<NDIM;++i) xKF[i] += KMat[i][0]*innov;

} // end function KF_xMUPD( )



 

#ifdef MATLAB_USE

EXPORTED_FUNCTION void print44Mat( FLOAT mat44[][NDIM] )
{
   int i,j;
   mexPrintf("\n");
   for ( i=0; i< NDIM; ++i ) 
   {
       for ( j=0; j< NDIM; ++j )
       {
           mexPrintf("%10.6f", mat44[i][j]);
       }
       mexPrintf("\n");
   }
   mexPrintf("\n");
}


EXPORTED_FUNCTION void myPrintMat( char matName[], int nRows, int nCols, FLOAT mat[] )
{
   int i,j,k;
   mexPrintf("%s =\n", matName);
   for ( i = 0; i < nRows; ++i ) 
   {
      for ( j = 0; j < nCols; ++j )
      {
          k = i*nCols + j;
        //k = j*nRows + i;
          mexPrintf("%10.6f", mat[k]);
      }
      mexPrintf("\n");
   }
   mexPrintf("\n");
}


/* this function exists so that mex may be used to compile the library
   it is not otherwise needed */

void mexFunction( int nlhs, mxArray *plhs[], 
          int nrhs, const mxArray*prhs[] )
{
}

#endif

/**
* ************************************************************************
*  End of module BallPlateKFlib_Template.c
**************************************************************************
*/
