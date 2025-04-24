function [sys,x0,str,ts,simStateCompliance] = PID(t,x,u,flag,PM)

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

global integral
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 8;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [0 0];
integral=0;
simStateCompliance = 'UnknownSimState';

function sys=mdlOutputs(t,x,u,PM)
global integral
x_hat = u(1:4);
X=u(5);Y=u(6);X_dot=u(7);Y_dot=u(8);

Kp = PM.Kp;ki=[7,0;0,8];dt=0.01;

Xe=X-x_hat(1);Ye=Y-x_hat(3);
Xe_dot=X_dot-x_hat(2);Ye_dot=Y_dot-x_hat(4);

err=[Xe;Xe_dot;Ye;Ye_dot];
err1=[Xe;Ye];

integral=integral+err1*dt;
if abs(integral) > 10
    integral = sign(integral)*10;
end

uf = Kp*err+ki*integral;

sys = uf;