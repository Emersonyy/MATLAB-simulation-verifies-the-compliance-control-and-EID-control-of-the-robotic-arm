close all;

figure(1);
subplot(411);
plot(out.t,out.dt(:,1),'r',out.t,out.EID(:,1),'b');
xlabel('time(s)');ylabel('EID估计');
legend('External torque','EID estimate');

subplot(412);
plot(out.t,out.x_y(:,1),'b');
xlabel('time(s)');ylabel('导纳控制');
legend('位置改变量');

subplot(413);
plot(out.t,out.x_y(:,1),'b',out.t,out.y(:,1),'r');
xlabel('time(s)');ylabel('output');
legend('Desire','actual');


