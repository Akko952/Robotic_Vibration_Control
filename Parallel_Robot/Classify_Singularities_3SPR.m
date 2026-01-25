%% Classify singularities for MonteCarlo workspace points (3SPR)
% 对每个 (d1,d2,d3) 样本：
% 1) 计算闭环正运动学 FK_3SPR
% 2) 更新支链几何（不绘图）reflash_Kinematic(..., doPlot=false)
% 3) 计算 H_p / H_a，并按秩判断奇异性
%    - 若 rank([H_a, H_p]) < rows([H_a, H_p]) -> configuration space singularity
%    - 否则若 rank(H_p) < rows(H_p) -> actuator singularity
%
% 输出：
%   config_sing, actuator_sing 两个结构体数组（含杆长与秩信息）
%   regular_mask 标记非奇异样本

% ---- 0) 初始化机构参数（若需要） ----
% ---- 1) 载入 MonteCarlo 数据（只需要杆长 D_ok） ----
matFile = 'workspace_montecarlo_3SPR_20000.mat';
if ~exist(matFile, 'file')
    error('找不到数据文件：%s', matFile);
end
S = load(matFile, 'D_ok');
D_ok = S.D_ok;

n = size(D_ok, 1);

% ---- 2) 参数设置 ----
rank_tol = 1e-9;   % rank() 容差（可按数值尺度调整）
progress_every = 200;

% 固定初值（按你的要求：每个点都用全零初值）
x0 = zeros(1, 12);

% ---- 3) 结果容器 ----
config_sing = struct('idx', {}, 'd', {}, 'R_H', {}, 'rowsH', {}, 'R_Hp', {}, 'rowsHp', {});
actuator_sing = struct('idx', {}, 'd', {}, 'R_H', {}, 'rowsH', {}, 'R_Hp', {}, 'rowsHp', {});
regular_mask = false(n, 1);
failed = struct('idx', {}, 'd', {}, 'reason', {});

% ---- 4) 主循环：逐点分类 ----
for i = 1:n
    d = D_ok(i, :);

    try
        [vars_sol_double, T_limb1_sol, Ad_all_1_sol, ~, Ad_all_2_sol, ~, Ad_all_3_sol] = ...
            FK_3SPR(S1, S2, S3, d(1), d(2), d(3), T_01, x0);

        % 更新支链几何：批处理时不要绘图
        [S1_new, S2_new, S3_new] = reflash_Kinematic(T_limb1_sol, p, T_12, T_13, false);

        [H_p_sol, H_a_sol] = cal_limb_jacobian(S1_new, S2_new, S3_new, Ad_all_1_sol, Ad_all_2_sol, Ad_all_3_sol);

        H = [H_a_sol, H_p_sol];
        rowsH = size(H, 1);
        R_H = rank(H, rank_tol);

        rowsHp = size(H_p_sol, 1);
        R_Hp = rank(H_p_sol, rank_tol);

        if R_H < rowsH
            config_sing(end+1) = struct('idx', i, 'd', d, 'R_H', R_H, 'rowsH', rowsH, 'R_Hp', R_Hp, 'rowsHp', rowsHp); %#ok<SAGROW>
            continue;
        end

        if R_Hp < rowsHp
            actuator_sing(end+1) = struct('idx', i, 'd', d, 'R_H', R_H, 'rowsH', rowsH, 'R_Hp', R_Hp, 'rowsHp', rowsHp); %#ok<SAGROW>
        else
            regular_mask(i) = true;
        end

        % 不滚动更新初值：保持 x0=zeros(1,12)

    catch ME
        failed(end+1) = struct('idx', i, 'd', d, 'reason', string(ME.identifier)); %#ok<SAGROW>
    end

    if mod(i, progress_every) == 0
        fprintf('progress: %d/%d | config=%d | actuator=%d | regular=%d | failed=%d\n', ...
            i, n, numel(config_sing), numel(actuator_sing), nnz(regular_mask), numel(failed));
    end
end

fprintf('\nDone. total=%d | config=%d | actuator=%d | regular=%d | failed=%d\n', ...
    n, numel(config_sing), numel(actuator_sing), nnz(regular_mask), numel(failed));

save('singularity_classification_3SPR_20000.mat', 'config_sing', 'actuator_sing', 'regular_mask', 'failed', 'rank_tol', 'matFile');
