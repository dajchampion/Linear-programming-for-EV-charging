clc; clear; close all;
rng(0);

%% 参数设定
numEV = 20; T = 24; Pmax = 7; Pmin = -7;
baseLoad = [150, 140, 130, 125, 120, 130, 180, 250, 350, 400, ...
            420, 450, 470, 460, 440, 430, 450, 500, 550, 520, ...
            480, 400, 300, 200];
lambda = 0.1 + 0.002 * baseLoad;
% 扩展 lambda 为 20x24 的矩阵
lambda = repmat(lambda, numEV, 1);  % 将 1x24 的 lambda 向量重复 20 次

SOC_max = 50 * ones(numEV, 1); 
SOC_max_expanded=repmat(SOC_max, 1, T);

SOC_min = 10 * ones(numEV, 1);
SOC_min_expanded=repmat(SOC_min, 1, T);

SOC_init = randi([15, 40], numEV, 1);
SOC_init_expanded = repmat(SOC_init, 1, T);  % 将 SOC_init 重复 24 次，形成 20x24 矩阵

SOC_target = randi([30, 45], numEV, 1);
EV_arrival = randi([1 12], numEV, 1);
EV_departure = EV_arrival + randi([6 12], numEV, 1);
EV_status = zeros(numEV, T);
for i = 1:numEV
    EV_status(i, EV_arrival(i):min(EV_departure(i), T)) = 1;
end

n=T;
A=ones(n);
A=triu(A);

%% 线性规划求解
cvx_begin
    variables P_ev(numEV, T)
    variable SOC(numEV, T)
    
    % 目标函数：最小化充电成本和负荷波动
    minimize(sum(sum(lambda .* P_ev)) + sum(sum((sum(P_ev, 1) + baseLoad - mean(baseLoad)).^2)))
    
    % 约束条件
    subject to
        Pmin * EV_status <= P_ev <= Pmax * EV_status;
        % SOC(:,1) == SOC_init + P_ev(:,1);
        % for t = 2:T
        %     SOC(:,t) == SOC(:,t-1) + P_ev(:,t);
        % end
        SOC == SOC_init_expanded + P_ev * A;
        SOC_min_expanded <= SOC <= SOC_max_expanded;
        for ev = 1:numEV
            SOC(ev, EV_departure(ev)) >= SOC_target(ev);
        end
cvx_end

%% 绘图
figure;
subplot(3,1,1);
plot(1:T, sum(P_ev,1) + baseLoad, 'b', 'LineWidth', 2); hold on;
plot(1:T, baseLoad, 'r--', 'LineWidth', 2);
xlabel('时间 (小时)'); ylabel('负荷 (kW)');
title('优化后总负荷'); legend('优化后', '基准负荷');

subplot(3,1,2);
imagesc(P_ev); colormap(jet); colorbar;
xlabel('时间 (小时)'); ylabel('EV编号');
title('EV 充放电策略');

subplot(3,1,3);
plot(1:T, SOC', 'k', 'LineWidth', 2);
xlabel('时间（小时）'); ylabel('电池能量');
title('EV能量变化');
