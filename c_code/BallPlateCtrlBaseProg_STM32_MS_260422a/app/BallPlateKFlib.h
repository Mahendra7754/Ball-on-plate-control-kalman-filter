/**
*  ***********************************************************************
*  @file    BallPlateKFlib.h
*  @author  B. Mysliwetz
*  @version V1.10
*  @brief   C function prototypes and defines for BallPlate simulation & 
*           Kalman Filter based estimation, usable via MATLAB 
*           Shared Library Calling (CALLLIB).
*
*  IMPORT C-functions from 'BallPlateKFlib.mexw64 into MATLAB program via :
*
*  if not(libisloaded('BallPlateKFlib'))
*   %addpath( fullfile(matlabroot,'extern','examples','shrlib') );
*    addpath( 'C:\myMatLab\BallPlatSim\' );
*    loadlibrary('BallPlateKFlib');
*  end
*  libfunctions BallPlateKFlib -full    % print lib function properties
*
*
*  USAGE of C-functions in MATLAB program via 'calllib( )' e.g.:
*
*  FParams = single( [ LLTest,  ThetaElKF_k, OmegaElKF_k ] );
*  FMat32 = ( calllib( 'BallPlateKFlib', 'KFcomputeFMat', FMat32', FParams ) )';
*  calllib( 'BallPlateKFlib', 'print44Mat', FMat32' );
*
*
*  Changes:
*  09.10.2013 mys     Initial version
*  14.12.2016 mys     Adapted for BallPlate                     
*  12.12.2019 mys     Adapted for R2019
*  07.12.2020 mys     FLOAT type/precision configurable via #define
*                      
*  @date    07/12/2020
*********************************************************************
*/

#ifndef BALLPLATE_KFLIB_H            // avoid multiple inclusion
#define BALLPLATE_KFLIB_H

#define EXPORT_FCNS

// configurable floatingpoint precision
#define  FLOAT    float // changed from double

// configurable system order
#define  NDIM     4

// configurable number of measurements
#define  MDIM     1


// time update related functions

 void KFcomputeFMat( FLOAT FMat[][NDIM], FLOAT FParams[] );
 void KFcomputeGMat(FLOAT GMat[NDIM], FLOAT GParams[]);

 void KFpredictState( FLOAT xKF[], FLOAT FMat[][NDIM], FLOAT G[], FLOAT uk );

 void KF_PTUPD( FLOAT PMat[][NDIM], FLOAT FMat[][NDIM], FLOAT Qdiag[] );



// measurement update related functions
 void KFcomputeKMat( FLOAT KMat[][MDIM], FLOAT PMat[][NDIM], FLOAT Rdiag[] );



 void KF_PMUPD( FLOAT Pkout[][NDIM], FLOAT KMat[][MDIM], FLOAT Pk1in[][NDIM] );

 void KF_xMUPD( FLOAT xKF[NDIM], FLOAT KMat[][MDIM], FLOAT yMeas[MDIM] );



// for test & debugging

 void KFcomputeCPCRinv( FLOAT CPCRinv[][MDIM], FLOAT PMat[][NDIM], FLOAT Rdiag[] );

 void KFcomputeKMat0( FLOAT KMat[][MDIM], FLOAT PMat[][NDIM], FLOAT CPCRinv[][MDIM] );


 void myPrintMat( char matName[], int nRows, int nCols, FLOAT mat[] );

 void print44Mat( FLOAT mat44[][NDIM] );

#endif  // #ifndef BALLPLATE_KFLIB_H

/**
* ************************************************************************
*  End of module BallPlateKFlib.h
**************************************************************************
*/