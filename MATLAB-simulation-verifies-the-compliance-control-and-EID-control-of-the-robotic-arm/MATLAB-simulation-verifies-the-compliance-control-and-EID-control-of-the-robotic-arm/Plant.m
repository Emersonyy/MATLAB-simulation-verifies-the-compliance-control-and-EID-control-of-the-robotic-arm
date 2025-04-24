function [sys,x0,str,ts,simStateCompliance] = Plant(t,x,u,flag,PM)

switch flag

  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;

  case 1
    sys=mdlDerivatives(t,x,u,PM);

  case 2
    sys=mdlUpdate(t,x,u);

  case 3
    sys=mdlOutputs(t,x,u,PM);

  case 4
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  case 9
    sys=mdlTerminate(t,x,u);

  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes

global p 
sizes = simsizes;

sizes.NumContStates  = 4;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 6;
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   

sys = simsizes(sizes);

% initialize the initial conditions
x0  = [0 0 0 0];

% str is always an empty matrix
str = [];

% initialize the array of sample times
ts  = [0 0];

p=[2.9 0.76 0.87 3.04 0.87];
simStateCompliance = 'UnknownSimState';


function sys=mdlDerivatives(t,x,u,PM)

global p 
ut = u(1:2);
dt = u(3:4);

D0=[p(1)+p(2)+2*p(3)*cos(x(3)) p(2)+p(3)*cos(x(3));
    p(2)+p(3)*cos(x(3)) p(2)];
C0=[-p(3)*x(4)*sin(x(3)) -p(3)*(x(2)+x(4))*sin(x(3));
     p(3)*x(2)*sin(x(3))  0];
s1= (-C0)/D0;s2=inv(D0); 
A=[0,1,0,0;0,s1(1,1),0,s1(1,2);0,0,0,1;0,s1(2,1),0,s1(2,2)];
B=[0,0;s2(1,1),s2(1,2);0,0;s2(2,1),s2(2,2)];

dot_x = A*x+B*(ut+dt);

sys = dot_x;


function sys=mdlUpdate(t,x,u)

sys = [];


function sys=mdlOutputs(t,x,u,PM)

C = PM.C;
y = C*x;
sys = [x;y];


function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    
sys = t + sampleTime;


function sys=mdlTerminate(t,x,u)

sys = [];


