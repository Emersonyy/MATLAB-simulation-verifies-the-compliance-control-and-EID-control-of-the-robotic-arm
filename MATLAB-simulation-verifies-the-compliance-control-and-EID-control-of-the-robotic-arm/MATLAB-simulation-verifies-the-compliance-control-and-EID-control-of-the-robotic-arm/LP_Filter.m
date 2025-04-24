function [sys,x0,str,ts,simStateCompliance] = LP_Filter(t,x,u,flag,PM)

switch flag,

  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;

  case 1,
    sys=mdlDerivatives(t,x,u,PM);

  case 2,
    sys=mdlUpdate(t,x,u);

  case 3,
    sys=mdlOutputs(t,x,u,PM);

  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  case 9,
    sys=mdlTerminate(t,x,u);

  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes

sizes = simsizes;

sizes.NumContStates  = 2;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   

sys = simsizes(sizes);

% initialize the initial conditions
x0  = [0 0];

% str is always an empty matrix
str = [];

% initialize the array of sample times
ts  = [0 0];

simStateCompliance = 'UnknownSimState';


function sys=mdlDerivatives(t,x,u,PM)

dt_hat = u(1:2);

AF  = PM.AF;
BF  = PM.BF;

dot_x = AF*x+BF*dt_hat;

sys = dot_x;


function sys=mdlUpdate(t,x,u)

sys = [];


function sys=mdlOutputs(t,x,u,PM)

CF = PM.CF;

dt_tilde = CF*x;
sys = dt_tilde;


function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    
sys = t + sampleTime;


function sys=mdlTerminate(t,x,u)

sys = [];


