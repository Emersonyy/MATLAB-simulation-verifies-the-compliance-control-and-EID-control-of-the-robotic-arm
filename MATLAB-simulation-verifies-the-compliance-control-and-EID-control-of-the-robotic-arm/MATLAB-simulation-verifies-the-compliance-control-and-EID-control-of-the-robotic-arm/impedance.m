function [sys,x0,str,ts,simStateCompliance] = impedance(t,x,u,flag,PM)

switch flag,
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
  case 3,
    sys=mdlOutputs(t,x,u,PM);
  case {1, 2, 4, 9 }
    sys = [];
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes

global delta_x delta_x_dot delta_x_ddot 
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 4;
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [0 0];
delta_x_ddot=[0;0];delta_x=[0;0];delta_x_dot=[0;0];
simStateCompliance = 'UnknownSimState';

function sys=mdlOutputs(t,x,u,PM)
global delta_x delta_x_dot delta_x_ddot 
F_ext=u(1:2);

Md=[0.5,0;0,1];Bd=[0.2,0;0,5];Kd=[5,0;0,10];dt=0.01;

delta_x_ddot = inv(Md)*(F_ext - Bd*delta_x_dot - Kd*delta_x) ;
delta_x_dot = delta_x_dot + delta_x_ddot*dt ;
delta_x = delta_x + delta_x_dot*dt ;
x_d =  delta_x;
x_dot_d=delta_x_dot;

sys(1) = x_d(1);
sys(2) = x_d(2);
sys(3) = x_dot_d(1);
sys(4) = x_dot_d(2);

