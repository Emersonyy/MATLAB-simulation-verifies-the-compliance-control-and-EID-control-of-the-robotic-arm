% 清除工作区和命令窗口
clear;
clc;

% 参数设置
L1 = 1; % 第一根连杆长度 (m)
L2 = 1; % 第二根连杆长度 (m)
m1 = 1; % 第一根连杆质量 (kg)
m2 = 1; % 第二根连杆质量 (kg)
g = 9.81; % 重力加速度 (m/s^2)

% 初始条件
theta1_0 = pi/4; % 第一根连杆初始角度 (rad)
theta2_0 = pi/4; % 第二根连杆初始角度 (rad)
dtheta1_0 = 0; % 第一根连杆初始角速度 (rad/s)
dtheta2_0 = 0; % 第二根连杆初始角速度 (rad/s)

% 导纳控制器参数
Kp = diag([10, 10]); % 比例增益矩阵
Dv = diag([5, 5]); % 阻尼增益矩阵

% 时间设置
tspan = [0 10]; % 模拟时间范围 (s)
dt = 0.01; % 时间步长 (s)
t = tspan(1):dt:tspan(end); % 时间向量

% 存储结果
theta1_hist = zeros(size(t));
theta2_hist = zeros(size(t));
x_hist = zeros(size(t));
y_hist = zeros(size(t));

% 初始化状态变量
theta1 = theta1_0;
theta2 = theta2_0;
dtheta1 = dtheta1_0;
dtheta2 = dtheta2_0;

% 模拟主循环
for k = 1:length(t)
    % 计算雅可比矩阵 J
    J = [-L1*sin(theta1) - L2*sin(theta1 + theta2);
          L1*cos(theta1) + L2*cos(theta1 + theta2)];
    
    % 计算末端执行器位置
    x = L1*cos(theta1) + L2*cos(theta1 + theta2);
    y = L1*sin(theta1) + L2*sin(theta1 + theta2);
    
    % 计算末端执行器速度
    dx_dt = J * [dtheta1; dtheta2];
    
    % 假设期望的末端执行器位置和速度为零
    x_des = 0;
    y_des = 0;
    dx_des_dt = [0; 0];
    
    % 计算误差
    error_pos = [x - x_des; y - y_des];
    error_vel = dx_dt - dx_des_dt;
    
    % 导纳控制律
    F_ext = Kp * error_pos + Dv * error_vel;
    
    % 计算关节空间中的力矩
    tau = J' * F_ext;
    
    % 计算动力学方程
    M = [m1*L1^2 + m2*(L1^2 + 2*L1*L2*cos(theta2)) + m2*L2^2, m2*(L1*L2*cos(theta2) + L2^2);
         m2*(L1*L2*cos(theta2) + L2^2), m2*L2^2];
    C = [-(m2*L1*L2*sin(theta2)*dtheta2), -m2*L1*L2*sin(theta2)*(dtheta1 + dtheta2);
         m2*L1*L2*sin(theta2)*dtheta1, 0];
    G = [(m1*L1 + m2*L1)*g*cos(theta1) + m2*g*L2*cos(theta1 + theta2);
         m2*g*L2*cos(theta1 + theta2)];
    
    % 解微分方程
    ddtheta = inv(M) * (tau - C*[dtheta1; dtheta2] - G);
    
    % 更新状态变量
    dtheta1 = dtheta1 + ddtheta(1) * dt;
    dtheta2 = dtheta2 + ddtheta(2) * dt;
    theta1 = theta1 + dtheta1 * dt;
    theta2 = theta2 + dtheta2 * dt;
    
    % 存储结果
    theta1_hist(k) = theta1;
    theta2_hist(k) = theta2;
    x_hist(k) = x;
    y_hist(k) = y;
end

% 绘制结果
figure;
subplot(3,1,1);
plot(t, rad2deg(theta1_hist), 'b');
hold on;
plot(t, rad2deg(theta2_hist), 'r');
xlabel('Time (s)');
ylabel('Joint Angles (\circ)');
legend('\theta_1', '\theta_2');
grid on;

subplot(3,1,2);
plot(x_hist, y_hist, 'g');
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('End-Effector Trajectory');
grid on;

subplot(3,1,3);
plot(t, sqrt(dx_hist.^2 + dy_hist.^2), 'm');
xlabel('Time (s)');
ylabel('End-Effector Speed (m/s)');
grid on;




