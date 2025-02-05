function Phi = cal_LIEKF_F(w,a, Rbn,dt)
    state_dim = 24;
    %{
    F.block(0, 0, 3, 3) = exp_so3(-w_hat * dt);
        F.block(0, 9, 3, 3).noalias() = -exp_so3(-w_hat * dt) * Jr_so3(-w_hat * dt) * dt;
        F.block(9, 9, 3, 3).setIdentity();
        F.block(6, 0, 3, 3).noalias() = -R_Gtoi.transpose() * skew_x(a_hat * dt);
        F.block(6, 6, 3, 3).setIdentity();
        F.block(6, 12, 3, 3) = -R_Gtoi.transpose() * dt;
        F.block(12, 12, 3, 3).setIdentity();
        F.block(3, 0, 3, 3).noalias() = -0.5 * R_Gtoi.transpose() * skew_x(a_hat * dt * dt);
        F.block(3, 6, 3, 3) = Eigen::Matrix3d::Identity() * dt;
        F.block(3, 12, 3, 3) = -0.5 * R_Gtoi.transpose() * dt * dt;
        F.block(3, 3, 3, 3).setIdentity();
        Eigen::Matrix<double, 15, 12> G = Eigen::Matrix<double, 15, 12>::Zero();
    %}
    %
    %
    %F = [0_33         0_33  0_33   -Rbn          0_33]
    %    [skew(gn)     0_33  0_33  -skew(vn)Rbn   -Rbn]
    %    [0.5*skew(gn) I_33  0_33  -skew(pn)Rbn   -Rbn]
    %    [         0_69                  I66          ]
    %
    %0611 yxy
        %F = [0_33         0_33  0_33   -Rbn          0_33]
    %    [skew(gn)     0_33  0_33  -skew(vn)Rbn   Rbn]
    %    [0.5*skew(gn) I_33  0_33  -skew(pn)Rbn   Rbn]
    %    [         0_69                  I66          ]
    %06820 yxy
        %F = [-skew(w)       0_33      0_33         -I_33        0_33]
    %        [-skew(a)      -skew(w)   0_33          0_33   -I_33]
    %        [0              I_33      -skew(w)      0_33     Rbn]
    %        [         0_69                  I66          ]
    Phi = zeros(state_dim,state_dim);
    g0 = 9.79;
    gn = [0;0;-g0];
    tao_wind = 50;
    F  = zeros(state_dim,state_dim);
    F(1:3,1:3) = -skewMat(w);
    F(1:3,10:12) = -eye(3);
    F(4:6,1:3) = - skewMat(a);
    F(4:6,4:6) = -skewMat(w);
    F(4:6,13:15) = -eye(3)*(-1);
    F(7:9,4:6) = eye(3)*(-1);
    F(7:9,7:9) = -skewMat(w);
    % F(7:9,13:15) = Rbn;
    % 
    % F(1:3,10:12) = -Rbn;
    % F(4:6,1:3)   = skewMat(gn);
    % F(4:6,10:12) = -skewMat(vn)*Rbn;
    % F(4:6,13:15) = -Rbn*(-1);
    % F(7:9,4:6)   = eye(3);
    % F(7:9,1:3)   = 0.5*skewMat(gn)*dt;
    % F(7:9,10:12) = -skewMat(pn)*Rbn;
    % F(7:9,13:15) = -Rbn*(-1);
    %F(10:15,10:15) = eye(6);
    F(22:24,22:24) = -1/tao_wind;
    Phi = double(eye(state_dim) - F * dt);

end




