x = out.simout_x(end, :)'; 

L1=x(1);L2=x(2);
L3=x(3);L4=x(4);
Q1=x(5);dQ1=x(6);ddQ1=x(7);
Q2=x(8);dQ2=x(9);ddQ2=x(10);
Q3=x(11);dQ3=x(12);ddQ3=x(13);
s=x(14);ds=x(15);dds=x(16);
dL2=x(17);ddL2=x(18);
D=x(19);
check_val_1 = L1*sin(Q1) - L2*sin(Q2);
check_val_2 = D+L1*cos(Q1) - L2*cos(Q2);
check_val_3= L4*cos(Q3) - L3*sin(Q2)-s;
check_val_4 = L4*sin(Q3) - L3*cos(Q2);
fprintf('验证 f1 (应接近0): %.4e\n', check_val_1);
fprintf('验证 f2 (应接近0): %.4e\n', check_val_2);
fprintf('验证 f3 (应接近0): %.4e\n', check_val_3);
fprintf('验证 f4 (应接近0): %.4e\n', check_val_4);

GenerateSimplifiedMechanism(x);