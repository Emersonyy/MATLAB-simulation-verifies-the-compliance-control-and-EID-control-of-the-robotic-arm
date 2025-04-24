% 导纳参数定义
M = 2;   % 质量 (kg)
B = 15;  % 阻尼 (Ns/m)
K = 30;  % 刚度 (N/m)

% 初始状态
x = 0;      % 初始位置
x_dot = 0;  % 初始速度
x_d = 0;    % 期望位置

% 仿真参数
dt = 0.001; % 时间步长
t_total = 5;% 总时长
steps = t_total/dt;

% 数据记录
pos_actual = zeros(1, steps);
pos_desired = zeros(1, steps);
force_ext = zeros(1, steps);

% 导纳控制主循环
for i = 1:steps
    t = i*dt;
    
    % 生成期望轨迹（示例为正弦运动）
    x_d = 0.1 * sin(2*pi*0.5*t);
    
    % 模拟外部力（示例为阶跃响应）
    if t > 1 && t < 3
        F_ext = 10; 
    else
        F_ext = 0;
    end
    
    % 导纳控制方程计算加速度
    x_ddot = (F_ext - B*(x_dot - 0) - K*(x - x_d)) / M;
    
    % 数值积分更新状态
    x_dot = x_dot + x_ddot * dt;
    x = x + x_dot * dt;
    
    % 记录数据
    pos_actual(i) = x;
    pos_desired(i) = x_d;
    force_ext(i) = F_ext;
end

% 可视化结果
figure;
subplot(2,1,1);
plot(dt:dt:t_total, pos_actual, 'b', dt:dt:t_total, pos_desired, 'r--');
xlabel('Time (s)');
ylabel('Position (m)');
legend('Actual','Desired');
title('导纳控制位置跟踪');

subplot(2,1,2);
plot(dt:dt:t_total, force_ext);
xlabel('Time (s)');
ylabel('External Force (N)');
title('外部作用力');

