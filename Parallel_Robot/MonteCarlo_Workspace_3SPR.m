%% Monte Carlo 工作空间搜索（3SPR）
% 随机采样主动关节变量 (d1,d2,d3)，调用 FK_3SPR 求平台位姿，收集点云绘制工作空间。
%
% 先定义几何约束： d_min / d_max
close all;

N = 20000;                 % 采样点数：FK_3SPR 每次要 vpasolve，太大可能会很慢
rng(0);                   % 固定随机种子便于复现实验

% 体素收敛提前停止（按成功样本数检查）
% 当工作空间在给定分辨率 h 下已收敛时，不必强行跑满 N
% 现在关闭，保证跑满固定的N
use_early_stop = false;
h_early = 0.5;                  % 提前停止用的体素分辨率
check_every_success = 1000;      % 每累计成功阈值检查一次
no_new_patience = 2;             % 检查两次得到新增体素数为0则停止



%%
% 定义杆长约束

d_min = [-1; -1; -1];        % (d1,d2,d3) 下界
d_max = [2; 2; 2];        % (d1,d2,d3) 上界

%% Monte Carlo 采样与求解
P = nan(N, 3);            % 平台位置点云
D = nan(N, 3);            % 对应的 (d1,d2,d3)
T_all = nan(4, 4, N);      % 平台齐次位姿（每个样本一个 4x4）

success = false(N, 1);
fail_reason = strings(N, 1);

ok_idx = zeros(N, 1);      % 成功样本对应的索引（预分配）
ok_cnt = 0;

last_covered = 0;
no_new_cnt = 0;

x0 = zeros(1, 12);        % vpasolve 的初值（使用上一组解滚动更新会更稳）

for i = 1:N
    d = d_min + (d_max - d_min) .* rand(3, 1);

    try
        [vars_sol_double, T_limb1_sol] = FK_3SPR(S1, S2, S3, d(1), d(2), d(3), T_01, x0);

        % 三条支链闭环后平台位姿一致，这里取 limb1 的位姿作为平台位姿
        p_platform = T_limb1_sol(1:3, 4);

        P(i, :) = double(p_platform(:)).';
        D(i, :) = double(d(:)).';
        T_all(:, :, i) = double(T_limb1_sol);
        success(i) = all(isfinite(P(i, :)));

        if success(i)
            ok_cnt = ok_cnt + 1;
            ok_idx(ok_cnt) = i;
        end

        % 用本次解作为下一次初值（显著提升收敛率/速度）
        x0 = vars_sol_double(:).';

    catch ME
        success(i) = false;
        fail_reason(i) = string(ME.identifier);
    end

    % 可选：按成功样本的体素覆盖收敛提前停止
    if use_early_stop && ok_cnt > 0 && mod(ok_cnt, check_every_success) == 0
        P_tmp = P(ok_idx(1:ok_cnt), :); %#ok<UNRCH>
        origin_tmp = min(P_tmp, [], 1);
        vox_tmp = floor((P_tmp - origin_tmp) ./ h_early);
        covered_now = size(unique(vox_tmp, 'rows'), 1);

        if covered_now == last_covered
            no_new_cnt = no_new_cnt + 1;
        else
            no_new_cnt = 0;
        end
        last_covered = covered_now;

        fprintf('[early-stop] success=%d, covered=%d (h=%.3g), noNew=%d/%d\n', ...
            ok_cnt, covered_now, h_early, no_new_cnt, no_new_patience);

        if no_new_cnt >= no_new_patience
            fprintf('[early-stop] 触发：连续 %d 次检查新增体素为 0，提前停止（success=%d / try=%d）。\n', ...
                no_new_patience, ok_cnt, i);
            break;
        end
    end
end

N = i;
success = success(1:i);
fail_reason = fail_reason(1:i);
P = P(1:i, :);
D = D(1:i, :);
T_all = T_all(:, :, 1:i);

P_ok = P(success, :);
D_ok = D(success, :);
T_ok = T_all(:, :, success);

fprintf('Monte Carlo 完成：成功 %d / %d (%.2f%%)\n', nnz(success), N, 100*nnz(success)/N);

%% 3.5) 体素覆盖率收敛检查（用来判断点数是否“够”）
% 分辨率 h 越小，越需要更多点；这里按你给的 h=0.5
h = 0.5;

% 也可以一次性检查多个分辨率
h_list = unique([h, 0.25, 1.0]);

if ~isempty(P_ok)
    % 主分辨率：画收敛曲线
    origin = min(P_ok, [], 1);
    vox = floor((P_ok - origin) ./ h);     % 体素坐标（整数）

    % 统计随采样数量增长的已覆盖体素数（分段计算，避免每点都 unique 太慢）
    step = max(50, round(size(P_ok,1) / 20));
    ks = step:step:size(P_ok,1);
    if ks(end) ~= size(P_ok,1)
        ks(end+1) = size(P_ok,1);
    end

    covered = zeros(size(ks));
    for ii = 1:numel(ks)
        covered(ii) = size(unique(vox(1:ks(ii),:), 'rows'), 1);
    end

    delta = [covered(1), diff(covered)];
    delta_ratio = delta ./ covered;

    fprintf('体素收敛检查 (h=%.3g): 最终覆盖体素数=%d\n', h, covered(end));
    fprintf('最后一段新增体素占比=%.3g\n', delta_ratio(end));

    % 多分辨率报告（不画图，只打印）
    for hh = h_list
        vox_h = floor((P_ok - origin) ./ hh);
        covered_h = size(unique(vox_h, 'rows'), 1);
        fprintf('  - h=%.3g: 覆盖体素数=%d\n', hh, covered_h);
    end

    figure('Name','Voxel Coverage Convergence');
    plot(ks, covered, '-o', 'LineWidth', 1.2);
    grid on; xlabel('成功样本数'); ylabel('已覆盖体素数');
    title(sprintf('Voxel Coverage (h=%.3g)', h));
else
    warning('没有成功点 (P_ok 为空)，无法做体素收敛检查。');
end

%% 4) 可视化
figure('Name','3SPR Workspace (Monte Carlo)');
scatter3(P_ok(:,1), P_ok(:,2), P_ok(:,3), 8, P_ok(:,3), 'filled');
axis equal; grid on; view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
title(sprintf('3SPR Workspace (N=%d, success=%d)', N, nnz(success)));
colorbar;

% 可选：用 alphaShape 估计包络（点数较多时会更慢）
shp = alphaShape(P_ok(:,1), P_ok(:,2), P_ok(:,3));
figure('Name','Workspace AlphaShape');
plot(shp);
axis equal; grid on; view(3);

%% 5) 保存结果
save('workspace_montecarlo_3SPR_20000.mat', 'P_ok', 'D_ok', 'T_ok', 'P', 'D', 'T_all', 'success', 'fail_reason', 'd_min', 'd_max', 'N');
