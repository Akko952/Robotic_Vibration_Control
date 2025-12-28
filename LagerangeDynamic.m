solve_kinematics;
CalculateKineticEnergy;

% 查看对 dQ1 求导后的结果
dT_dQ1 = simplify(diff(T_all, dQ1));
disp('对 dQ1 求导后的结果：');
dT_dQ1

dT_dQ1 = collect(dT_dQ1, dQ1)