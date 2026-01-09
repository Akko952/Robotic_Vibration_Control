function xi = vee_se3(X)
%VEE_SE3 将 4x4 se(3) 矩阵转换为 6x1 twist (in Space form)
%
%   [ w_hat  v ]
%   [   0    0 ]
%
% xi = [w; v]

    w_hat = X(1:3,1:3);
    v     = X(1:3,4);

    w = [ w_hat(3,2);
          w_hat(1,3);
          w_hat(2,1) ];

    xi = [w; v];
end