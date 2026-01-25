# Parallel_Robot（3SPR 并联机构）

本目录包含一个 3SPR 并联机构的运动学/雅可比/工作空间分析与可视化工具链：
- 运动学参数初始化与支链 POE 建模
- 平台/支链雅可比计算（含力雅可比相关计算）
- 闭环正运动学（数值求解）
- 蒙特卡洛工作空间采样 + alphaShape 包络 + 体素收敛检查
- 基于位姿序列的机构动画绘制与 GIF 导出
- 杆长（d1,d2,d3）梯形速度规划（含正负位移）

## 依赖与环境

- MATLAB（建议 R2023b 及以上）
- 需要 Symbolic Math Toolbox（`vpasolve` 用于闭环 FK 数值求解）

## 快速开始

### 1) 初始化并显示一个姿态

直接运行 [MAIN.m](MAIN.m)：

1. `Kinematic` 初始化机构几何与螺旋轴参数
2. 计算初始状态下 POE、雅可比
3. 调用 `FK_3SPR` 求解给定 (d1,d2,d3) 的平台位姿
4. 调用 `reflash_Kinematic` 绘制机构

### 2) 蒙特卡洛工作空间

运行 [MonteCarlo_Workspace_3SPR.m](MonteCarlo_Workspace_3SPR.m)：

- 随机采样主动关节变量 `(d1,d2,d3)`（由 `d_min/d_max` 控制）
- 调用 `FK_3SPR` 得到平台位姿并收集点云
- 绘制 3D 点云（颜色按 z）
- 用 `alphaShape` 绘制近似包络
- 输出体素覆盖收敛检查（默认 `h=0.5`，并打印多分辨率覆盖统计）
- 保存数据到 `workspace_montecarlo_3SPR_20000.mat`

常用输出变量（保存在 `.mat` 中）：
- `P_ok`：成功样本的平台位置点云（N×3）
- `D_ok`：成功样本对应的 (d1,d2,d3)（N×3）
- `T_ok`：成功样本的平台位姿（4×4×N）

### 3) 杆长轨迹与 GIF 导出

工作流：
1. 使用 [trap_traj_disp_signed.m](trap_traj_disp_signed.m) 为 d1/d2/d3 生成梯形速度位移轨迹
2. 逐帧调用 `FK_3SPR(..., x0)` 求解平台位姿序列 `T_seq(4×4×N)`
3. 调用 [make_robot_gif.m](make_robot_gif.m) 导出 GIF

要点：
- `make_robot_gif` 的输入 **必须** 是 `4x4xN` 齐次矩阵序列。
- `FK_3SPR` 支持可选初值 `x0`：建议用“上一帧解滚动更新”为下一帧初值，显著提升连续求解的收敛率与速度。

## 主要文件说明

### 入口/演示

- [MAIN.m](MAIN.m)：示例流程入口（初始化 → 雅可比 → FK → 绘图 → 工作空间 → 轨迹/GIF）。
- [Kinematic.m](Kinematic.m)：机构几何参数与螺旋轴参数初始化。

### 运动学与雅可比

- [branch_forward_kinematics.m](branch_forward_kinematics.m)：单支链 POE 正运动学（输出支链末端位姿与伴随矩阵序列）。
- [cal_POE_and_AdjointT.m](cal_POE_and_AdjointT.m)：计算初始时刻三支链 POE 与伴随矩阵。
- [cal_limb_jacobian.m](cal_limb_jacobian.m)：计算开环支链空间雅可比，并组装相关矩阵。
- [cal_platform_jacobian_by_forceScrew.m](cal_platform_jacobian_by_forceScrew.m)：平台力雅可比相关计算。
- [Force_Jacobian.m](Force_Jacobian.m)：力雅可比（或相关处理）脚本/函数。

### 闭环正运动学（数值求解）

- [FK_3SPR.m](FK_3SPR.m)：闭环正运动学求解（使用 `vpasolve`）。
  - 输入：`S1,S2,S3,d1,d2,d3,T_01`，以及可选 `x0`（12 个未知量的初值）。
  - 输出：`vars_sol_double`（12×1）与三条支链闭环后的位姿 `T_limb*_sol`。

### 可视化与导出

- [reflash_Kinematic.m](reflash_Kinematic.m)：根据平台位姿更新三条支链的几何并绘制；内部使用专用图窗，避免污染其他 figure。
- [make_robot_gif.m](make_robot_gif.m)：对 `T_seq(4×4×N)` 逐帧绘制并写入 GIF。

### 轨迹规划

- [trap_traj_disp.m](trap_traj_disp.m)：位移型梯形/三角速度规划（原始版本，通常假设位移为非负）。
- [trap_traj_disp_signed.m](trap_traj_disp_signed.m)：对 `trap_traj_disp` 的封装，支持位移为正/负。

## 常见问题（FAQ）

### 1) 为什么连续求解时突然不收敛？

`vpasolve` 对初值敏感；连续轨迹建议用上一帧 `vars_sol_double` 作为下一帧 `x0`，而不是每帧都用全零初值。

### 2) 为什么 GIF/序列赋值时报 “T_seq 必须是 4x4xN”？

`make_robot_gif` 会检查输入维度；请确保你存入的是 `double` 类型的 4×4 齐次矩阵，并在求解失败时跳过该帧或给出替代帧。

## 变更记录（基于提交历史的简要整理）

- b6484e1：新增工作空间蒙特卡洛脚本、体素收敛检查、GIF 导出、带符号位移的梯形规划；`FK_3SPR` 增加 `x0` 初值输入；`reflash_Kinematic` 重构并隔离绘图图窗。
- 7b2ab5b：实现正运动学求解。
- b6078d3：实现力雅可比/约束相关雅可比计算，存在个别行差异待进一步核对。
- 8056a4e / 287bc35 / 305f50f / 067bdaa：重构与清理，提升可读性，整理文件结构。
