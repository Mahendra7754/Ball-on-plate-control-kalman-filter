%*************************************************************************
% BallPlateSim0.m
%
% BallPlate (x-AXIS ONLY) simulation base program for xPos- & xDot-
% estimation via a 2nd- or 3rd-ORDER MODEL & OBSERVER
%
% 2022/11/06 mys
% 2026/02/21  Mahendra Singh
%
% NOTES  
%  Logfile ball position (XXXpos) measurements/estimates are scaled in [pxl]  
%  Logfile ball speed    (XXXdot) values/estimates       are scaled in [pxl/s]
%  They can be re-scaled into [m] and [m/s] via constant CM2PXL (see code).
%
% Format of logfile record:
%  1st column:   Abs. log time stamp          t_abs        in [s]
%  9th column:   Measured ball position       xPosMeas     in [pxl]
% 10th column:   Measured ball position       yPosMeas     in [pxl]
% 11th column:   Reference ball position      xPosRef      in [pxl]
% 12th column:   Reference ball position      yPosRef      in [pxl]
% 13th column:   Ball speed via num. dif.     xDotDifLog   in [pxl/s]
% 14th column:   Ball speed via num. dif.     yDotDifLog   in [pxl/s]
% 15th column:   Platform angle x             xAlfaPlatLog in [o]
% 16th column:   Platform angle y             yAlfaPlatLog in [o]
% 17th column:   Servo arm angle x            xThetServLog in [o]
% 18th column:   Servo arm angle y            yThetServLog in [o]
% 19th column:   Servo angle x PWM command    xThetPWMcmd  in [us]
% 20th column:   Servo angle y PWM command    yThetPWMcmd  in [us]
% 21st column:   Estimated ball x position    xPosEstLog   in [pxl]
% 22nd column:   Estimated ball x speed       xDotEstLog   in [pxl/s]
% Extended record option:
% +23rd column:  Estimated ball y position    yPosEstLog   in [pxl]
% +24th column:  Estimated ball y speed       yDotEstLog   in [pxl/s]
%
%   MatLab decimal number format uses '.' vs orig logfile uses ','  !! 
%   Original *.csv was converted to MatLab readable decimal format.
%
% Variable naming & suffix convention:
%  -Meas   real measured value
%  -Ref    real controller reference/setpoint value
%  -Dif    num. differentiation via backwards difference
%  -Pos    Position
%  -Dot    Position derivative == speed
%  -Est    Estimate (computed via Observer or KF)
%  -Obs    Estimate computed via Observer
%  -KF     Estimate computed via Kalman Filter
%
%  -Log    from log file == recorded from real system
%  -Sim    computed via this MATLAB simulation, to be compared with -Log values
%
% Changes
% 161025 Initial version, read 'BallPlateLog.cvs' & plot selected columns
% 171101 Detailed inline documentation of logfile record, added comments
% 181129 Line 206 changed sign of xThetServSim_v = - THETA_SERVO_MAX * ...
% 191025 Line 111 set PWM_X_OFFSET = 0 to compensate calibration offset
% 221101 Legends added to plots
%*************************************************************************

clear all;
close all;
clc;

%*************************************************************************
% PROGRAM CONFIGURATION & SIMULATION CONTROL SECTION
%*************************************************************************

LogFile   = 'BallPlateLogData.csv';
kMax      =  300;         % # of records to be read from *.csv logfile
kSub     =  15;         % # of records to be read from *.csv logfile

% if not (libisloaded('BallPlateKFlib'))
%     addpath("C:\Users\mahen_r913oco\Desktop\Study Material\KALMAN\project\untitled.m");
%     loadlibrary("BallPlateKFlib");
% end
% libfunctions BallPlateKFlib -full

% Common/general simulation parameters
T         = 50.0e-3;      % DEFAULT sample period @ 20Hz frame rate = 50ms
kSample_v = 0:kMax-1;     % simulation sample index vector
SimTime_v = T*kSample_v;  % simulation time vector

sObsPole0 = -10.0;       % observer pole assignment in S-domain
sObsPole1 = -20.0;
sObsPole2 = -25.0;       % only used in 3rd order system


% Various flags/switches to control program behaviour 
bLogFileFormat24Columns = 1; % if 1: extended logfile record +yEst +yDotEst in cols 23&24
bUseTrueSineTheta       = 0; % if 1: compute num. precise AlfaPlatSim with sin(ThetaServo)
bPlotBasicLogFileData   = 1; % if 1: plot real system / basic logfile data
bPlotSimObs2 = 1; % if 1: compute&plot OBS2 simulation vs real system
bPlotSimObs3 = 1; % if 1: compute&plot OBS3 simulation vs real system
bPlotKF3 = 1; % if 1: compute&plot KF3 estimates vs OBS3 & real system
bPlotEKF4 = 1; % if 1: compute&plot EKF4 estimates vs KF3 & real system
bPlotcEKF4=1; % if 1: compute EKF4 and EKF4c

%*************************************************************************
% INITIALIZATION SECTION
%*************************************************************************
% Elementary numeric values / constants / scaling factors

T22       = T*T/2;        % term used in time discrete model equations

CSQRT2    = sqrt(2.0);
CSQRT3    = sqrt(3.0);
C2PI      = 2.0 * pi;
CRAD2DEG  = 180.0 / pi;   % converts [rad] -> [o]
CDEG2RAD  = pi / 180.0;   % converts [o] -> [rad]

CGEARTH   = 9.81;         % gravity constant in [m/s2]
CM2PXL    = 1380;         % camera & lens scaling factor in [pxl] per [m]
                          % for VRMagic camera (1350..1390) with 8.5mm lens
                          
% Plant model parameters
THETA_SERVO_MAX = 50;     % servo arm angle max amplitude        in [o]
PWM_CENTER   =  1500;     % PWM for servo neutral position       in [us]
PWM_AMPLTD   =   500;     % PWM amplitude for servo +/-max pos   in [us]
PWM_X_OFFSET =    60;     % PWM offset for plate horizontal pos. in [us]  -35
PWM_Y_OFFSET =     0;     % PWM offset for plate horizontal pos. in [us]  -50
PWM_XOFFS_CAL   =  60;   % PWM adjustment in [us] for PLATE HORIZONTAL CALIBRATION. DEFAULT +60

PWM_XOFFS_BIAS  =   -100;   % PWM adjustment in [us] to  SIMULATE a BIAS in ThetaSCmdSim for EKF4 estimation

PWM_XOFFSET  = PWM_XOFFS_CAL + PWM_XOFFS_BIAS;  % PWM adjustment in [us] for ThetaSCmdSim 


mBall     = 0.003;        % ball mass        in [kg]   NOT USED/NEEDED HERE !
rBall     = 0.002;        % ball radius      in [m]    NOT USED/NEEDED HERE !
dServo    = 0.030;        % servo arm length in [m]
dPlate    = 0.200;        % plate arm attachment radius in [m]

kbb = -0.6*CGEARTH*dServo/dPlate; % model/accel. constant in SI units
kga = kbb * CM2PXL * CDEG2RAD;    % ZiSo's model/acceleration constant:
                                  % requires input (servo-) angle in [o]
                                  % returns output scaled in [pxl/s2] 
 
                                  %symmetry variable
 symErrPred=0;
 symErrUpdt=0;

kbb =kbb*CM2PXL; %% 
%% Second order observer Plant model

 if(bPlotSimObs2)   % Time CONTINUOUS 2nd order plant model:   xdot(t) = A2*x(t) + b2*u(t)

A2= [0 1; 0 0]
b2=[0;kga];
ct= [1 0]
Qb2=[ct;
    ct*A2]
%%syc= ss(A2,b2,ct,[]) 

  
det( Qb2 )               % observability check for CONTINUOUS system
if (det( Qb2 ) == 0)
   warning( 'det(Q_b2) = 0: 2nd ORDER CONTINUOUS SYSTEM NOT OBSERVABLE !' ); % message/break
end


% Time DISCRETE 2nd order plant model:   x[k+1] = F2*x[k] + g2*u[k]

I= eye(2);
F2_exp = expm(A2*T); %% matrix exponential

F2_Antl= [1 T;  %%Analytical
          0 1]
F2 = F2_exp; 
syms tau
E = expm(A2*tau);
g2= double(int(E*b2,tau,0,T))

z0 = exp(sObsPole0*T); %%continuous to discreet poles
z1 = exp(sObsPole1*T);

a0= -(z0+z1);
a1= (z1*z0);

Qz_F = F2_exp*F2_exp + a0*F2_exp+ a1* eye(2);
Qz_F_Antl = F2_Antl*F2_Antl + a0*F2_Antl + a1* eye(2);

Qb2= [ct;
    ct*F2_exp] ;%% observability matrix

O_Antl= [ct;
    ct*F2_Antl] ,%% observability matrix using F2 analytical

det( Qb2 )               % observability check for TIME DISCRETE system
if (det( Qb2 ) == 0)
   warning( 'det(Q_b2) = 0: 2nd ORDER DISCRETE SYSTEM NOT OBSERVABLE !' ); % message/break
end

% Ackermann 1
disp('2 nd order observer gain Using Ackermann 1')
v= [0;1];
h_obs = Qz_F*(Qb2\v); %% Observer gain Ackermann 1

h_obs_Antl2 = Qz_F_Antl*(Qb2\v) %% Observer gain 

% Ackermann 2
disp('using Ackermann 2')
    syms z;
    Qz = (z-z0)*(z-z1);
    AA_coeffs = coeffs(Qz,z);
    q0 = double(AA_coeffs(1));
    q1 = double(AA_coeffs(2));
    q2 = double(AA_coeffs(3));
   
    
    l3 = inv(Qb2)*[0;1]; %% last column of inverted observability matrix
    
    hObs2 = (q0*eye(2) + q1*F2_exp + q2*F2_exp*F2_exp) *l3 %% ackermann 2
    
    h_obs2_Antl= (q0*eye(2) + q1*F2_Antl + q2*F2_Antl*F2_Antl) *l3

 end




%% 3rd order observer Plant Model

 if (bPlotSimObs3==1)  % Time CONTINUOUS 3nd order plant model:   xdot(t) = A2*x(t) + b2*u(t)    taos = 0.2;[]
    
    taos = 0.2;
     as = 1/taos;
    
    A3 = [0 1 0;
        0 0 kga;
        0 0 -as];
    b3 = [0;0;as];
    c3 = [1 0 0];
    d3 = [0];
    
    F3_exp= expm(A3*T)
    syms tau;
    E = expm(A3*tau);
    G3_1= double(int(E*b3,tau,0,T)) %
    
    %Discretization
    %create a state space system
    %sys3 = ss(A3,b3,c3,d3);
    %discretize it with matlab function
    %sys_d3 = c2d(sys3,T);
    %F3 = sys_d3.A;
    %G3 = sys_d3.B;

   %  Observability matrix (discrete) 
    Qb3 = [c3; 
        c3*F3_exp;
        c3*F3_exp^2];
    
% observability check
if abs(det(Qb3)) < 1e-12
    warning('det(Qb3) ~ 0: 3rd ORDER DISCRETE SYSTEM NOT OBSERVABLE!');
end

 z31 = exp(sObsPole0*T);
 z32 = exp(sObsPole1*T);
 z33 = exp(sObsPole2*T);

% Characteristic polynomial coefficients 
% Desired polynomial: (z - z31)(z - z32)(z - z33)

a2 = -(z31 + z32 + z33);
a1 =  (z31*z32 + z31*z33 + z32*z33);
a0 = -(z31*z32*z33);

% Evaluate polynomial 
Qz_F3 = (F3_exp^3) + a2*(F3_exp^2) + a1*(F3_exp) + a0*eye(3);

%Ackermann 1 (Observer gain)
disp('3rd order observer gain using Ackermann 1')
v3 = [0; 0; 1];                
h_obs3 = Qz_F3 * (Qb3 \ v3)     % (3x1) observer gain vector
    
 
    syms z;
    Qz = (z-z31)*(z-z32)*(z-z33);
    AA_coeffs = coeffs(Qz,z);
    q30 = double(AA_coeffs(1));
    q31 = double(AA_coeffs(2));
    q32 = double(AA_coeffs(3));
    q33 = double(AA_coeffs(4));
    
    l3 = inv(Qb3)*[0;0;1]; %% last column of inverted observability matrix
    
    disp('3rd order observer gain using Ackermann 2')

    hObs3 = (q30*eye(3) + q31*F3_exp + q32*F3_exp*F3_exp+q33*F3_exp*F3_exp*F3_exp) *l3 %% ackermann 2

   %% 3rd order linear Kalman filter
  
        taos = 0.2;
     as = 1/taos;
    
    A3k = [0 1 0;
        0 0 kga;
        0 0 -as];
    b3k = [0;0;as];
    c3k = [1 0 0];
    d3k = [0];
    
    disp("3rd order KF")
    F3k_exp= expm(A3k*T)
    syms tauk;
    Ek = expm(A3k*tauk);
    G3k_1= double(int(Ek*b3k,tauk,0,T)) %

   Hk            = c3k; %% transformation matrix
   sigmaXposPxl = 0.5; %% measurement noise standard deviation, scaled in
                       % 0.1 / 1.0 / 5.0
   rVar3        = sigmaXposPxl * sigmaXposPxl; % measurement noise variance 
   
   qVarXpos     = 5.0; % process noise covariance matrix diag. elements
qVarXdot        = 200; % try out 10.0 / 100.0 / 1000.0 <‐> effect ? 
qVarTheta       = 5;
q11             = qVarXpos;
q22             = qVarXdot;
q33             = qVarTheta;

Q3k             = diag( [ q11 q22 q33 ] ); % process noise covariance matrix

% initialize/dimension est. error covariance matrix P & Kalman gain vector kKF

P3k             = diag( [ 100 100 100] );  % Initial est. error cov. matrix P3(0)
P3k1            = diag( [ 3.31 3.31 3.31] ); % Extrapolated est. error cov. matrix
kKF3            = [ 0 0 0 ]'; % Kalman filter gain vector

% 3rd ORDER KALMAN FILTER system simulation

xPosSim3KF_v    = zeros( kMax,1 ); % xPos via KF simulation ‐ plot vector
xDotSim3KF_v    = zeros( kMax,1 ); % xDot via KF simulation ‐ plot vector
xThetSim3KF_v   = zeros( kMax,1 ); % xThet via KF simulation ‐ plot vector

% kSub is a subsection of kMax to plot some values w HIGH timing resolution
xk1KF3_v        = zeros( kSub,1 ); % kKF3(1) ‐ plot vector hires
xk2KF3_v        = zeros( kSub,1 ); % kKF3(2) ‐ plot vector hires
xk3KF3_v        = zeros( kSub,1 ); % kKF3(3) ‐ plot vector hires

xP11KF3_v       = zeros( kSub,1 ); % P3Mat(1,1) ‐ plot vector hires
xP22KF3_v       = zeros( kSub,1 ); % P3Mat(2,2) ‐ plot vector hires
xP33KF3_v       = zeros( kSub,1 ); % P3Mat(3,3) ‐ plot vector hires
    
 end

 %% Time DISCRETE NONLINEAR 4th order plant model:   x(k+1) = Phi4o*x(k) + g4o*u(k)

 if (bPlotEKF4==1||bPlotcEKF4==1)
    taos = 0.2;
     as = 1/taos; %% servo time constant

     

     A4o = [ 0  1    0     0;
        0  0   kga   kga;
        0  0  -as    0;
        0  0    0    0 ];

     b4o = [0; 0; as; 0];
     I4 = eye(4);

    
F4olin = I4 + A4o*T

T2  = T*T;
F4o2 = I4 + A4o*T + 0.5*(A4o*A4o)*T2

% exact expm 
F4mexp = expm(A4o*T);


Phi4o = F4o2;     
%Phi4o = F4mexp;  
    
c4 = [1;0;0;0];     % so y = c4' * x
H4 = c4.'; %% measurement matrix
   
% observability check for the DISCRETE linearized model:
Qb4d = [ H4;
         H4*Phi4o;
         H4*(Phi4o^2);
         H4*(Phi4o^3) ];

detQb4d = det( Qb4d );               % observability check TIME DISCRETE system  
RankQb4 = rank( Qb4d );
if (RankQb4 < 4)
    warning( 'Rank of Q_b4d < 4: DISCRETE 4th ORDER ThetOffset-Param SYSTEM NOT OBSERVABLE !' ); % message/break
end


  sigmaXposPxl = 1; %% measurement noise standard deviation, scaled in
                       % 0.1 / 1.0 / 5.0
   rVar4k        = sigmaXposPxl * sigmaXposPxl; % measurement noise variance 
   
qVarXpos     = 5.0; % process noise covariance matrix diag. elements
qVarXdot        = 100; % try out 10.0 / 100.0 / 1000.0 <‐> effect ? 
qVarTheta       = 5;
qVarThetOffs    = 0.5;
q11             = qVarXpos;
q22             = qVarXdot;
q33             = qVarTheta;
q44             = qVarThetOffs;

Q4o             = diag( [ q11 q22 q33 q44] ); % process noise covariance matrix
Q4oc             = [ q11; q22; q33; q44]; % process noise covariance matrix fo c code

% initialize/dimension est. error covariance matrix P & Kalman gain vector kKF

P4ok             = diag( [ 1.0 10.0  1.0  2.0] );  % Initial est. error cov. matrix P3(0)
P4ok1            = diag( [ 0 0 0 0] ); % Extrapolated est. error cov. matrix
kEKF4o            = [ 0 0 0 0]'; % Kalman filter gain vector

% C variable
Pc4ok             = diag( [ 1.0 10.0  1.0  2.0] );  % Initial est. error cov. matrix P3(0)
Pc4ok1            = diag( [ 0 0 0 0] ); % Extrapolated est. error cov. matrix
kcEKF4o            = [ 0 0 0 0]'; % Kalman filter gain vector

% 4th ORDER KALMAN FILTER system simulation

xPosSimEKF4o_v    = zeros( kMax,1 ); % xPos via KF simulation ‐ plot vector
xDotSimEKF4o_v    = zeros( kMax,1 ); % xDot via KF simulation ‐ plot vector
xThetSimEKF4o_v   = zeros( kMax,1 ); % xThet via KF simulation ‐ plot vector
xThetOffsSimEKF4o_v   = zeros( kMax,1 ); % xThet via KF simulation ‐ plot vector

%C-variable
xcPosSimEKF4o_v    = zeros( kMax,1 ); % xPos via KF simulation ‐ plot vector
xcDotSimEKF4o_v    = zeros( kMax,1 ); % xDot via KF simulation ‐ plot vector
xcThetSimEKF4o_v   = zeros( kMax,1 ); % xThet via KF simulation ‐ plot vector
xcThetOffsSimEKF4o_v   = zeros( kMax,1 ); % xThet via KF simulation ‐ plot vector

kSub= 2*kMax;

% kSub is a subsection of kMax to plot some values w HIGH timing resolution
xk1KF4o_v        = zeros( kSub,1 ); % kKF3(1) ‐Kalman Gain plot vector hires
xk2KF4o_v        = zeros( kSub,1 ); % kKF3(2) ‐ plot vector hires
xk3KF4o_v        = zeros( kSub,1 ); % kKF3(3) ‐ plot vector hires
xk4KF4o_v        = zeros( kSub,1 ); % kKF4(4) ‐ plot vector hires

% C-variable
xck1KF4o_v        = zeros( kSub,1 ); % kKF3(1) ‐ plot vector hires
xck2KF4o_v        = zeros( kSub,1 ); % kKF3(2) ‐ plot vector hires
xck3KF4o_v        = zeros( kSub,1 ); % kKF3(3) ‐ plot vector hires
xck4KF4o_v        = zeros( kSub,1 ); % kKF4(4) ‐ plot vector hires

xP11KF4o_v       = zeros( kSub,1 ); % P3Mat(1,1) ‐ Covarience plot vector hires
xP22KF4o_v       = zeros( kSub,1 ); % P3Mat(2,2) ‐ plot vector hires
xP33KF4o_v       = zeros( kSub,1 ); % P3Mat(3,3) ‐ plot vector hires
xP44KF4o_v       = zeros( kSub,1 ); % P3Mat(3,3) ‐ plot vector hires

% C-C-variable
xcP11KF4o_v       = zeros( kSub,1 ); % P3Mat(1,1) ‐ plot vector hires
xcP22KF4o_v       = zeros( kSub,1 ); % P3Mat(2,2) ‐ plot vector hires
xcP33KF4o_v       = zeros( kSub,1 ); % P3Mat(3,3) ‐ plot vector hires
xcP44KF4o_v       = zeros( kSub,1 ); % P3Mat(3,3) ‐ plot vector hires
 end

%% *************************************************************************
% DATA INPUT / LOGFILE READ SECTION
%*************************************************************************

iLine = 2;    % line # to start reading (skip header & only LONG! lines) 
jCol0 = 0;    % column # to start reading

if ( bLogFileFormat24Columns == 1 ), % use extended 24 column data record
  jCol1 = 23; % last column index, extended (NEW) logfile has 24 data columns
else                                 % use 22 column data record
  jCol1 = 21; % OLDER ballplate logfile may have only 22 data columns
end

% Read 1st data record/line from logfile (i.e. skip header text line !)
LogData1_t = dlmread( LogFile , ';', iLine, jCol0, [iLine jCol0 iLine jCol1] );

% Read desired range of kMax data records into one table/matrix LogData1_t
for k = 1:kMax-1
  iLine = iLine + 2;    % current line#/record to be read from logfile
  LogRec1_t = dlmread( LogFile , ';', iLine, jCol0, [iLine jCol0 iLine jCol1] );
  %LogRec1_t            % test output of current file record 
  LogData1_t = [ LogData1_t
                 LogRec1_t  ]; % append new record to total table
end % for 

% Split up logfile table/matrix into individual variables -> column vectors XXX_v
AbsTime_v      = LogData1_t(:,1);  %  1st column:  absolute log time in [s]

xPosMeas_v     = LogData1_t(:,9);  %  9th column:  xPosMeas   in [pxl]
yPosMeas_v     = LogData1_t(:,10); % 10th column:  yPosMeas   in [pxl]

xPosRef_v      = LogData1_t(:,11); % 11th column:  xPosRef    in [pxl]
yPosRef_v      = LogData1_t(:,12); % 12th column:  yPosRef    in [pxl]

xDotDifLog_v   = LogData1_t(:,13); % 13th column:  xDotDifLog in [pxl/s]
yDotDifLog_v   = LogData1_t(:,14); % 14th column:  yDotDifLog in [pxl/s]

xAlfaPlatLog_v = LogData1_t(:,15); % 15th column:  platform angle x  in [o]
yAlfaPlatLog_v = LogData1_t(:,16); % 16th column:  platform angle y  in [o]

xThetServLog_v = LogData1_t(:,17); % 17th column:  servo arm angle x in [o]
yThetServLog_v = LogData1_t(:,18); % 18th column:  servo arm angle y in [o]

xThetPWMcmd_v = LogData1_t(:,19)-PWM_CENTER+PWM_X_OFFSET; % 19th col: xThetPWMcmd in [us]
yThetPWMcmd_v = LogData1_t(:,20)-PWM_CENTER+PWM_Y_OFFSET; % 20th col: yThetPWMcmd in [us]

xPosEstLog_v = LogData1_t(:,21);   % 21st column:  xPosEstLog    in [pxl]
xDotEstLog_v = LogData1_t(:,22);   % 22nd column:  xDotEstLog    in [pxl/s]

if ( bLogFileFormat24Columns == 1 ),
  yPosEstLog_v = LogData1_t(:,23); % +23rd column: +yPosEstLog   in [pxl]
end

if ( bLogFileFormat24Columns == 1 ),
  yDotEstLog_v = LogData1_t(:,24); % +24th column: +yDotEstLog   in [pxl/s]
end


% compute alfa platform simulation values (+/- numerically precise/true sin(Theta) )
xThetServSim_v = - THETA_SERVO_MAX * xThetPWMcmd_v / PWM_AMPLTD;      % in [o]

if ( bUseTrueSineTheta == 1 ) % use true sin(ThetaServo)
  xAlfaPlatSim_v = sin(xThetServSim_v*CDEG2RAD) * dServo / dPlate;
else                          % use linearization for sin(ThetaServo)
  xAlfaPlatSim_v = xThetServSim_v * dServo / dPlate;            % in [o]
end


if ( 40 )
% Plot xAlfaPlatLog vs xAlfaPlatSim
  figure(30); clf;
  plot(SimTime_v, [ xThetServLog_v  xThetServSim_v ] );
  grid on;
  xlabel('SimTime [s]')
  ylabel('Theta  [o] [o]')
  legend('ThetServLog', 'ThetServSim');
  title('ThetaServLog  ThetaServSim');

  figure(41); clf;
  plot(SimTime_v, [ xAlfaPlatLog_v  xThetServSim_v  xAlfaPlatSim_v ] );
  grid on;
  xlabel('SimTime [s]')
  ylabel('Alfa/Theta  [o] [o] [o]')
  legend('AlfaPlatLog', 'ThetServSim', 'AlfaPlatSim');
  title('AlfaPlatLog  ThetaServSim AlfaPlatSim');
end

%*************************************************************************
% DATA PROCESSING & SIMULATION SECTION
%*************************************************************************

% dimension & init simulation vars / plot vectors
xDotSimLPF_v   = zeros( kMax,1 );

%% 2nd order system simulation
xPosSim2Obs_v  = zeros( kMax,1 ); % xPos2 via observer simulation
xDotSim2Obs_v  = zeros( kMax,1 ); % xDot2 via observer simulation

% 2nd order
xPosSim        =zeros(kMax,1); %% xpos via open loop response
xDotSim        =zeros(kMax,1); %% xDot via open loop response

% dimension & init 2nd order state vector before starting sim loop
xState2Obsk  = [ xPosMeas_v(1)    % initialize xObsk(1) w measured value
                 0 ];
xState2Obsk1 = [ xPosMeas_v(1)    % initialize xObsk1(1) w measured value
                 0 ];

%% 3rd order system simulation
xPosSim3Obs_v  = zeros( kMax,1 ); % xPos2 via observer simulation
xDotSim3Obs_v  = zeros( kMax,1 ); % xDot2 via observer simulation
xThetaSim3Obs_v =zeros(kMax,1);   % xTheta via observer simulation

% 3rd order state variables

xPosSim3OL= zeros(kMax,1); %% xpos via open loop response
xdotSim3OL= zeros(kMax,1); %% xDot via open loop response

         
xState3Obsk  = [ xPosMeas_v(1)    % initialize xObsk(1) w measured value
                 0 
                 0];
xState3Obsk1 = [ xPosMeas_v(1)    % initialize xObsk1(1) w measured value
                 0 
                 0 ];  
%% 3rd and 4th ORDER KALMAN FILTER

xState3KFk =[xPosMeas_v(1);0;0];    % initializig with measured value
xState3KFk1 =[xPosMeas_v(1);0;0];

xStateEKF4ok =[xPosMeas_v(1);0;0;0];    % initializig states with measured value
xState4EKF4ok1 =[xPosMeas_v(1);0;0;0];

 DEG2RAD = pi/180; % Degree to radian conversion
%% open loop response of system
xState = [0; 0]; % Initialize state vector [position; velocity]



uk     = 0; % scalar control input variable (here: servo arm angle Theta in [o])
input =zeros(kMax,1);


%% second order simulation open loop

if(bPlotSimObs2) 

% MAIN SIMULATION LOOP
for k = 1:kMax % sample index 

 input(k)= uk;
 if k>=20 && k<40
 uk=30; %% input servo angle
 
 elseif k>= 40 && k<60
 uk=-30;
 elseif k>= 60
 uk= 00;
 end
 % Store current state
 xPosSim(k) = xState(1); % Position
 xDotSim(k) = xState(2); % Velocity
 
 % State update using state-space equations
 
 xState = F2 * xState + g2 * uk;
end
%% 2 nd order closed loop respose
yk =[0];
 
 for k=1:kMax
 yk = xPosMeas_v(k);
 uk= -xThetPWMcmd_v(k)/10; %% conversion factor
 xState2Obsk1 = F2*xState2Obsk + g2*uk + h_obs*(yk-ct*xState2Obsk);  % x[k+1] = F2*x[k] + g2*u[k] + h2*( y[k] - c2'x[k] ) 
 xPosSim2Obs_v(k)= xState2Obsk(1);
 xDotSim2Obs_v(k)= xState2Obsk1(2);
 xState2Obsk = xState2Obsk1;
 
 end
end

%% 3rd order simulations

uk=0; %% input
xState = [0; 0; 0]; % Initialize state vector [position; velocity]

if(bPlotSimObs3==1) %%open loop respose
 
for k=1:kMax;

input(k)= uk;

if k>=20 && k<40 %% From 1s to < 2s
 uk=25;          %% input ThetaServoSim
 
elseif k>= 40 && k<60 %% from 2s to <3s
 uk=-25;

elseif k>= 60        %% at 3 s
 uk= 00;

end

 % Store current state
 xPosSim3OL(k) = xState(1); % Position
 xDotSim3OL(k) = xState(2); % Velocity
 
 % State update using state-space equations
 
 xState = F3_exp * xState + G3_1 * uk;

end

%%3rd order Closed Loop
 
 for k=1:kMax
         
 yk = xPosMeas_v(k);
 uk= -xThetPWMcmd_v(k)/10; %% conversion factor
 xState3Obsk1 = F3_exp*xState3Obsk + G3_1*uk + hObs3*(yk-c3*xState3Obsk);

 xPosSim3Obs_v(k)= xState3Obsk1(1);
 xDotSim3Obs_v(k)= xState3Obsk1(2);
 xThetaSim3Obs_v(k)=xState3Obsk1(3);
 xState3Obsk = xState3Obsk1;

end
    %%disp("done with 3 order")

end
%%disp("done with 3 order outside")

%% 3rd order Kalman Filter 
if bPlotKF3
%disp("inside kf3")
    for k=1:kMax
         yk = xPosMeas_v(k);
         uk= -xThetPWMcmd_v(k)/10; %% conversion factor
         
         % Prediction
         % a priori state estimate
         xState3KFk1 = F3k_exp*xState3KFk+G3k_1*uk;

         % covarience estimation
         P3k1= F3k_exp* P3k*F3k_exp.'+Q3k;

         % Measurement upadte

         % Upadte kalman gain
         kKF3= P3k1*Hk.'*inv(Hk*P3k1*Hk.'+rVar3);

         % posteriori state estimate // correction step
         xState3KFk = xState3KFk1+kKF3*(yk-Hk*xState3KFk1);

         %Covarience update
         P3k = (eye(3)-kKF3*Hk)*P3k1;

         %checking symmetry
         symErrPred = max(abs(P3k1 - P3k1.'));
         symErrUpdt = max(abs(P3k  - P3k.'));

if any(symErrPred > 1e-10) || any(symErrUpdt > 1e-10)
    fprintf('k=%d  symErrPred=%.3e  symErrUpdt=%.3e\n', k, symErrPred, symErrUpdt);
end

       
         %storing estimated position , velocity and theta
         xPosSim3KF_v(k)=xState3KFk(1); 
         xDotSim3KF_v(k)=xState3KFk(2);
         xThetSim3KF_v(k)=xState3KFk(3);

        if(kSub>=k)
         % storing covarience over time
         xP11KF3_v(k) =P3k(1,1);
         xP22KF3_v(k) =P3k(2,2);
         xP33KF3_v(k) =P3k(3,3);

         % storing Kalman Gain over time
        xk1KF3_v(k)    =kKF3(1);
        xk2KF3_v(k)    =kKF3(2);
        xk3KF3_v(k)    =kKF3(3);
        end
        
    end

disp("KKF3")
disp(kKF3)
end

%% 4rd order Kalman Filter 

xStateEKF4ok =[xPosMeas_v(1);0;0;0];    % initializig with measured value for Matlab calculation
xStateEKF4ok1 =[xPosMeas_v(1);0;0;0];

xcStateEKF4ok =[xPosMeas_v(1);0;0;0];    % initializig with measured value for C-function
xcStateEKF4ok1 =[xPosMeas_v(1);0;0;0];

 DEG2RAD = pi/180;

if (bPlotEKF4==1||bPlotcEKF4==1)
%disp("inside kf4")
    for k=1:1:kMax
         yk = xPosMeas_v(k);
         uk= - (xThetPWMcmd_v-PWM_XOFFSET) / 10; %input angle in Degrees with Bias
 %uk= - (xThetPWMcmd_v) / 10;

         theta_rad       = xStateEKF4ok(3) * CDEG2RAD;  % Servo angle in rad
        delta_theta_rad = xStateEKF4ok(4) * CDEG2RAD; % bias  in rad, changed CDEGto Rad, it was already in radian

        c = cos(theta_rad + delta_theta_rad);

        % F = df/dx  (linearization of nonlinear accel term) 
        % Jaccobian matrix
        
        Fnl4 = [ 1  T   0              0;
                 0  1   (kga*c*T)      (kga*c*T);
                 0  0   (1-as*T)       0;
                 0  0   0              1 ];

               Gnl4 = [ 0;
                 kga*c*T22*as;
                 as*T - as^2*T22;
                 0 ];

         % Prediction using Fx + Gu
        xStateEKF4ok1 = Fnl4 * xStateEKF4ok + Gnl4 * uk(k);

         % covarience prediction
        P4ok1 = Fnl4*P4ok*Fnl4' + Q4o;

                %%Measurement upadte
         % Upadte kalman gain
         S      = H4*P4ok1*H4' + rVar4k;      % scalar
         kEKF4o  = (P4ok1*H4')/S;             % 4x1

        innov  = yk - H4*xStateEKF4ok1;        % scalar

        xStateEKF4ok = xStateEKF4ok1 + kEKF4o*innov;

         %Covarience update
         % Joseph stabilized P update 
         I4  = eye(4);
    KH   = kEKF4o*H4;                    % 4x4
    P4ok = (I4-KH)*P4ok1*(I4-KH)' + kEKF4o*rVar4k*kEKF4o';


         %checking symmetry
         symErrPred = max(abs(P4ok1 - P4ok1.'));
         symErrUpdt = max(abs(P4ok  - P4ok.'));

if any(symErrPred > 1e-10) || any(symErrUpdt > 1e-10)
    fprintf('k=  symErrPred=%.3e  symErrUpdt=%.3e\n', k, symErrPred, symErrUpdt);
end

       
    % storing estimated position , velocity ,theta and bias
        xPosSimEKF4o_v(k)         = xStateEKF4ok(1); 
        xDotSimEKF4o_v(k)        = xStateEKF4ok(2);
        xThetSimEKF4o_v(k)       = xStateEKF4ok(3);
        xThetOffsSimEKF4o_v(k)   = xStateEKF4ok(4);
       
    % predicted covariance (after extrapolation)
        xP11KF4o_v(2*k-1) = P4ok1(1,1);
        xP22KF4o_v(2*k-1) = P4ok1(2,2);
        xP33KF4o_v(2*k-1) = P4ok1(3,3);
        xP44KF4o_v(2*k-1) = P4ok1(4,4);

    % updated covariance (after innovation)
        xP11KF4o_v(2*k) = P4ok(1,1);
        xP22KF4o_v(2*k) = P4ok(2,2);
        xP33KF4o_v(2*k) = P4ok(3,3);
        xP44KF4o_v(2*k) =  P4ok(4,4);

    % storing Kalman Gain over time
        xk1KF4o_v(k)    =kEKF4o(1);
        xk2KF4o_v(k)    =kEKF4o(2);
        xk3KF4o_v(k)    =kEKF4o(3);
        xk4KF4o_v(k)    =kEKF4o(4);
       
%Calculations using C-Functions

         ykc = xPosMeas_v(k);
         ukc = - (xThetPWMcmd_v(k)-PWM_XOFFSET)/10; %input angle in Degrees woith bias
  %ukc = - (xThetPWMcmd_v(k)/10); 
     theta_radc       = xcStateEKF4ok(3) * CDEG2RAD;   % Servo angle in rad
     delta_theta_radc = xcStateEKF4ok(4) * CDEG2RAD; 

    %variable for calculation of F and G matrix
    Fcparams = [T, kga, theta_radc, delta_theta_radc, as];
    Gcparams = [T, kga, theta_radc, delta_theta_radc, as ];

    % calling C-Fuctions to calculate F and G matrix
    % Fcnl4 = zeros(4,4,'double');
    % Fcnl4 = calllib("BallPlateKFlib","KFcomputeFMat", Fcnl4, Fcparams);
    % Fcnl4 = Fcnl4.';
    % Fcnl4 = Fcnl4.';
    % Gcnl4 = zeros(4,1,'double');
    % Gcnl4 = calllib("BallPlateKFlib", "KFcomputeGMat",Gcnl4,Gcparams);
    % 
    % 
    % 
    % 
    % %prediction
    % xcStateEKF4ok1 = calllib("BallPlateKFlib", "KFpredictState",xcStateEKF4ok(:),Fcnl4,Gcnl4(:),ukc);
    % Pc4ok1  = calllib("BallPlateKFlib", "KF_PTUPD", Pc4ok,Fcnl4,Q4oc);
    % 
    % %Measurement upadate
    % kcEKF4o = calllib("BallPlateKFlib", "KFcomputeKMat", kcEKF4o,Pc4ok1,rVar4k);
    % Pc4ok  = calllib("BallPlateKFlib","KF_PMUPD",Pc4ok,kcEKF4o,Pc4ok1);
    % xcStateEKF4ok = calllib("BallPlateKFlib","KF_xMUPD",xcStateEKF4ok1(:),kcEKF4o,xPosMeas_v(k));
    % 
    % 
    % %storing state varibles for plotting
    % 
    % xcPosSimEKF4o_v(k)    = xcStateEKF4ok(1); 
    % xcDotSimEKF4o_v(k)    = xcStateEKF4ok(2); 
    % xcThetSimEKF4o_v(k)   = xcStateEKF4ok(3); 
    % xcThetOffsSimEKF4o_v(k)   = xcStateEKF4ok(4); 
    % 
    %  %Updated covarience 
    %  xcP11KF4o_v(2*k) = Pc4ok(1,1);
    %  xcP22KF4o_v(2*k) = Pc4ok(2,2);
    %  xcP33KF4o_v(2*k) = Pc4ok(3,3);
    %  xcP44KF4o_v(2*k) = Pc4ok(4,4);
    % 
    % % predicted covariance (after extrapolation)
    %   xcP11KF4o_v(2*k-1) = Pc4ok1(1,1);
    %   xcP22KF4o_v(2*k-1) = Pc4ok1(2,2);
    %   xcP33KF4o_v(2*k-1) = Pc4ok1(3,3);
    %   xcP44KF4o_v(2*k-1) = Pc4ok1(4,4);
    % 
    % % storing Kalman Gain over time
    %   xck1KF4o_v(k)    =kcEKF4o(1);
    %   xck2KF4o_v(k)    =kcEKF4o(2);
    %   xck3KF4o_v(k)    =kcEKF4o(3);
    %   xck4KF4o_v(k)    =kcEKF4o(4);

    % 
    end
    % disp('Kalman gain from Matlab kEKF4o')
    % disp(kEKF4o)
    % 
    % disp('Kalman gain from C fn kcEKF4o')
    % disp(kcEKF4o)
   

end



%*************************************************************************
% DATA OUTPUT / PLOTTING SECTION
%*************************************************************************

%% second order plots

if(bPlotSimObs2)

figure(1);
 subplot(3, 1, 1);
 plot(SimTime_v, xPosSim, 'b-', 'LineWidth', 1.5);
 grid on;
 xlabel('Time [s]');
 ylabel('Ball Position [pxl]');
 title('Open Loop Ball Position (xPosSim)');
 
 subplot(3, 1, 2);
 plot(SimTime_v, xDotSim, 'r-', 'LineWidth', 1.5);
 grid on;
 xlabel('Time [s]');
 ylabel('Ball Speed [pxl/s]');
 title('Open Loop Ball Speed (xDotSim)');
 
 
 subplot(3, 1, 3);
 plot(SimTime_v, input, 'g-', 'LineWidth', 1.5);
 grid on;
 xlabel('Time [s]');
 ylabel('input angle in degrees');
 title('input to the system');
 
 figure(3);
 hold on;
 plot(SimTime_v,xPosMeas_v)
 hold on;
 plot(SimTime_v,xPosEstLog_v)
 hold on;
 plot(SimTime_v,xPosSim2Obs_v)
 hold on;
 legend("xPosMeas","xPosEstLog","xPos2Obs")
 title("Second order with observer, observed vs estimated position")
 xlabel("Time, s")
 ylabel("Ball position [pixels]")
 
 figure(4);
 hold on;
 plot(SimTime_v,xDotDifLog_v)
 hold on;
 plot(SimTime_v,xDotEstLog_v)
 hold on;
 plot(SimTime_v,xDotSim2Obs_v)
 hold on;
 legend("xDotDifLog","xDotEstLog","xDot2Obs")
 title("Second order with observer, observed vs estimated velocity")
 xlabel("Time, s")
 ylabel("Ball velocity [pixels/second]")

end

%% 3rd order plots

if(bPlotSimObs3==1)

 figure(5);

 subplot(3, 1, 1);
 plot(SimTime_v, xPosSim3OL, 'b-', 'LineWidth', 1.5);
 grid on;
 xlabel('Time [s]');
 ylabel('Ball Position [pxl]');
 title('Open Loop Ball Position (xPosSim)');
 
 subplot(3, 1, 2);
 plot(SimTime_v, xDotSim3OL, 'r-', 'LineWidth', 1.5);
 grid on;
 xlabel('Time [s]');
 ylabel('Ball Speed [pxl/s]');
 title('Open Loop Ball Speed (xDotSim)');
 
 
 subplot(3, 1, 3);
 plot(SimTime_v, input, 'g-', 'LineWidth', 1.5);
 grid on;
 xlabel('Time [s]');
 ylabel('input angle in degrees');
 title('input to the system');
 
 
 figure(6);
 hold on;
 plot(SimTime_v,xPosMeas_v)
 hold on;
 plot(SimTime_v,xPosEstLog_v)
 hold on;
 
 plot(SimTime_v,xPosSim3Obs_v)
 grid on;
 plot(SimTime_v,xPosRef_v)
 grid on;
 legend("xPosMeas","xPosEstLog","xPos3Obs","xPosref")
 title("3rd order with observer, observed vs estimated position")
 xlabel("Time, s")
 ylabel("Ball Position [pxl]")
 
 figure(7);

 hold on;
 plot(SimTime_v,xDotDifLog_v)
 hold on;
 plot(SimTime_v,xDotEstLog_v)
 hold on;
  
 plot(SimTime_v,xDotSim2Obs_v)
 hold on;
 plot(SimTime_v,xDotSim3Obs_v)
 hold on;
 plot(SimTime_v,-xThetPWMcmd_v)
 hold on;
 legend("xDotDifLog","xDotEstLog","xDotSim2Obs","xDot3Obs","xThetaPWMcmd")
 title("3rd order and 2 Order, observed vs estimated velocity")
 xlabel("Time, s")
 ylabel("Ball velocity [pixels/second], Theta_commanded in PWM")
 hold off;
 
 
end

if(bPlotKF3==1)


 figure(9);

 hold on;
 plot(SimTime_v,xPosMeas_v)
 hold on;
 plot(SimTime_v,xPosEstLog_v)
 hold on;
 plot(SimTime_v,xPosSim3Obs_v)
 hold on;
 plot(SimTime_v,xPosSim3KF_v)
 grid on;
 plot(SimTime_v,xPosRef_v)
 grid on;
 legend("xPosMeas","xPosEstLog","xPos3Obs","xPos3KF","xPosref")
 title("Estimated position Measured, Estimated,3rd Order observer, Kalman Filter and reference ")
 xlabel("Time, s")
 ylabel("Ball Position [pxl]")
 hold off;

 figure(10);

 hold on;
 plot(SimTime_v,xDotDifLog_v)
 hold on;
 plot(SimTime_v,xDotEstLog_v,'LineWidth', 1.5)
 hold on;
 plot(SimTime_v,xDotSim3Obs_v)
 hold on;
 plot(SimTime_v,xDotSim3KF_v)
 hold on;
 plot(SimTime_v,-xThetPWMcmd_v)
 hold on;
 legend("xDotDifLog","xDotEstLog","xDot3Obs","xDotSim3KF","xThetaPWMcmd")
 title("3rd order with observer,Kalman Filter, observed vs estimated velocity")
 xlabel("Time, s")
 ylabel("Ball velocity [pixels/second], Theta_commanded in PWM")
 hold off;

 figure(11);

 subplot(3,1,1)
 plot( xP11KF3_v,'b-')
 ylabel('P(1,1)')
 subplot(3,1,2)
 plot( xP22KF3_v,'r-')
 ylabel('P(2,2)')
 subplot(3,1,3)
 plot( xP33KF3_v,'m-')
 ylabel('P(3,3)')
 xlabel('k')
 title('Covariance over time')
 hold off;

 figure(12);

 subplot(3,1,1)
 plot(  xk1KF3_v,'b-')
 ylabel('xK1F3')
 subplot(3,1,2)
 plot( xk2KF3_v,'r-')
 ylabel('xK2F3')
 subplot(3,1,3)
 plot(  xk3KF3_v,'m-')
 ylabel('xK3F3')
 xlabel('k')
 title('(KKF3)Kalman Gain over time')
 hold off;
end
% 
% 
if(bPlotEKF4==1)


 figure(13);

 hold on;
 plot(SimTime_v,xPosMeas_v)
 hold on;
 plot(SimTime_v,xPosEstLog_v)
 hold on;
 plot(SimTime_v,xPosSim3Obs_v)
 hold on;
 plot(SimTime_v,xPosSim3KF_v)
 plot(SimTime_v, xPosSimEKF4o_v)
 grid on;
 plot(SimTime_v,xPosRef_v)
 grid on;
 legend("xPosMeas","xPosEstLog","xPosSim3Obs","xPosSim3KF"," xPosSimEKF4","xPosref")
 title("Estimated position Measured,Estimated,3rd Order Observer, 3rd Order linear KF,  4rth order EKF and reference ")
 xlabel("Time, s")
 ylabel("Ball Positin [pxl] ")
 hold off;

 figure(14);

 hold on;
 plot(SimTime_v,xDotDifLog_v)
 hold on;
 plot(SimTime_v,xDotEstLog_v,'LineWidth', 1.5)
 hold on;
 plot(SimTime_v,xDotSim3Obs_v)
 hold on;
 plot(SimTime_v,xDotSim3KF_v)
 hold on;
 plot(SimTime_v, xDotSimEKF4o_v)
  hold off;

 legend("xDotDifLog","xDotEstLog","xDotSim3Obs","xDotSim3KF"," xDotSimEKF4o")
 title("Velocity Comparison between Estimated velocity Vs 3rd Order Observer Vs 3rd Order Kalman Filter Vs Extended Kalman Filter ")
 xlabel("Time, s")
 ylabel("Ball velocity [pixels/second]")

 figure(15);

hold on;
 plot( xP11KF4o_v(1:200),'b-')
hold on;
 plot(  xP22KF4o_v(1:200)/100,'r-')
hold on;
 plot(  xP33KF4o_v(1:200),'m-')
hold on;

 plot( xP44KF4o_v(1:200),'g-')
 title('Covariance over time')
 ylabel('P(1,1),P(2,2),P(3,3),P(4,4)')
 legend("P(1,1)","P(2,2)/100","P(3,3)","P(4,4)")
 xlabel('k')


  figure(16);

 subplot(4,1,1)
 plot(xk1KF4o_v(1:200) ,'b-')
 title('(KKF4)Kalman Gain over time')
 ylabel('xK1F4')
 subplot(4,1,2)
 plot(xk2KF4o_v(1:200) ,'r-')
 ylabel('xK2F4')
 subplot(4,1,3)
 plot(xk3KF4o_v(1:200) ,'m-')
 ylabel('xK3F4')
 subplot(4,1,4)
 plot(xk4KF4o_v(1:200) ,'g-')
 ylabel('xK4F4')
 xlabel('k')

 figure(17)
hold on;
plot(SimTime_v, xThetPWMcmd_v, 'r', 'LineWidth', 1.5);
hold on;
plot(SimTime_v, xThetOffsSimEKF4o_v*10, 'm');
hold on;
plot(SimTime_v, xThetaSim3Obs_v*10, 'k');
hold on;
plot(SimTime_v, xThetSimEKF4o_v*10, 'b');
grid on;
legend('xThetPWMcmd','xThetOffsSimEKF4o','xThetaSim3Obs', 'xThetSimEKF4o');
title('Comparison of xPWMCmd(us), Estimated Bias, XThetSimobs3, XThetSimEKF4');
xlabel('Time [s]');
ylabel('us, Angle in [deg]*10');
hold off;
end

if(bPlotcEKF4==1)


 figure(18);

 hold on;
 plot(SimTime_v,xPosMeas_v)
 hold on;
 plot(SimTime_v,xPosEstLog_v)
 hold on;
 plot(SimTime_v, xPosSimEKF4o_v)
 grid on;
 plot(SimTime_v, xcPosSimEKF4o_v)
 grid on;
 plot(SimTime_v,xPosRef_v)
 grid on;
 legend("xPosMeas","xPosEstLog","xPosSimEKF4","xcPosSimEKF4o","xPosref")
 title("Comparison of position Measured,Estimated,4rth order EKF, EKFc and reference ")
 xlabel("Time, s")
 ylabel("Ball Positin [pxl] ")
 hold off;

 figure(19);

 hold on;
 plot(SimTime_v,xDotDifLog_v)
 hold on;
 plot(SimTime_v,xDotEstLog_v,'LineWidth', 1.5)
 hold on;
 plot(SimTime_v, xDotSimEKF4o_v)
 hold on;
 plot(SimTime_v, xcDotSimEKF4o_v)
 hold off;

 legend("xDotDifLog","xDotEstLog"," xDotSimEKF4o","xcDotSimEKF4o")
 title("velocity  Extended Kalman Filter vs EKFc vs estimated velocity ")
 xlabel("Time, s")
 ylabel("Ball velocity [pixels/second]")

 figure(20);

hold on;
plot( xcP11KF4o_v(1:200),'b-')
hold on;
plot(  xcP22KF4o_v(1:200)/100,'r-')
hold on;
plot(  xcP33KF4o_v(1:200),'m-')
hold on;
plot( xcP44KF4o_v(1:200),'g-')
title('Covariance over time from C fn')
ylabel('P(1,1),P(2,2),P(3,3),P(4,4)')
legend("P(1,1)","P(2,2)/100","P(3,3)","P(4,4)")
xlabel('k')


  figure(21);

 subplot(4,1,1)
 plot(xck1KF4o_v(1:200) ,'b-')
 title('(cKKF4)Kalman Gain over time from c fn')
 ylabel('xcK1F4')
 subplot(4,1,2)
 plot(xck2KF4o_v(1:200) ,'r-')
 ylabel('xcK2F4')
 subplot(4,1,3)
 plot(xck3KF4o_v(1:200) ,'m-')
 ylabel('xcK3F4')
 subplot(4,1,4)
 plot(xck4KF4o_v(1:200) ,'g-')
 ylabel('xcK4F4')
 xlabel('k')


end
% *************************************************************************
% End of BallPlateSim0.m
% **********************************************************************