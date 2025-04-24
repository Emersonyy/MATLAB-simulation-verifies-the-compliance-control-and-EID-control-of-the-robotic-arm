function [sys,x0,str,ts,simStateCompliance] = SO(t,x,u,flag,PM)

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

sizes.NumContStates  = 4;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 4;
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   

sys = simsizes(sizes);

% initialize the initial conditions
x0  = [0 0 0 0];

% str is always an empty matrix
str = [];

% initialize the array of sample times
ts  = [0 0];

simStateCompliance = 'UnknownSimState';


function sys=mdlDerivatives(t,x,u,PM)

global p 
y  = u(1:2);
uf = u(3:4);
x_hat = [x(1);x(2);x(3);x(4)];

D0=[p(1)+p(2)+2*p(3)*cos(x(3)) p(2)+p(3)*cos(x(3));
    p(2)+p(3)*cos(x(3)) p(2)];
C0=[-p(3)*x(4)*sin(x(3)) -p(3)*(x(2)+x(4))*sin(x(3));
     p(3)*x(2)*sin(x(3))  0];
s1=(-C0)/D0; s2=inv(D0); 
A=[0,1,0,0;0,s1(1,1),0,s1(1,2);0,0,0,1;0,s1(2,1),0,s1(2,2)];
B=[0,0;s2(1,1),s2(1,2);0,0;s2(2,1),s2(2,2)];

C  = PM.C;
L = PM.L;

dot_x_hat = A*x_hat+B*uf+L*(y-C*x_hat);

sys = dot_x_hat;


function sys=mdlUpdate(t,x,u)

sys = [];


function sys=mdlOutputs(t,x,u,PM)

L = PM.L;
C = PM.C;
x_hat = [x(1);x(2);x(3);x(4)];
y = u(1:2);

w = L*(y-C*x_hat);

sys = w;


function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    
sys = t + sampleTime;


function sys=mdlTerminate(t,x,u)

sys = [];


