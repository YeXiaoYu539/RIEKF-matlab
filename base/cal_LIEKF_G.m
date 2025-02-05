function G = cal_LIEKF_G(Rbn, dt)
    % G.block(0, 0, 3, 3) = -exp_so3(-w_hat * dt) * Jr_so3(-w_hat * dt) * dt;
    %    G.block(6, 3, 3, 3) = -R_Gtoi.transpose() * dt;
    %    G.block(3, 3, 3, 3) = -0.5 * R_Gtoi.transpose() * dt * dt;
    %    G.block(9, 6, 3, 3).setIdentity();
    %    G.block(12, 9, 3, 3).setIdentity();
    state_dim = 24;
    cov_dim = 21;
    G = zeros(state_dim,cov_dim);
    % G(1:3,1:3) = -  Rbn * dt;
    % G(4:6,1:3) = -skewMat(vn)*Rbn* dt;
    % G(4:6,4:6) =  - Rbn;
    % G(7:9,1:3) = -skewMat(pn)*Rbn * dt;
    % G(7:9,4:6) =  - Rbn;
    % G(10:12,7:9) = eye(3)*dt;
    % G(13:15,10:12) = eye(3)*dt;


    % G(1:3,1:3) =  - Rbn * dt;
    % G(4:6,1:3) = -skewMat(vn)*Rbn * dt*(-1);
    % G(4:6,4:6) =  - Rbn * dt;
    % G(7:9,1:3) = -skewMat(pn)*Rbn * dt*(-1);
    % G(7:9,4:6) =  - Rbn * dt;
    % G(10:12,7:9) = eye(3)* dt;
    % G(13:15,10:12) = eye(3);
    % G(16:18,13:15) = eye(3);
    % G(19:21,16:18) = eye(3)* dt;
    % G(22:24,19:21) = eye(3)* dt;

    G(1:3,1:3) = - dt;
    % G(4:6,4:6) = - Rbn'* dt;
    % G(7:9,4:6) = - Rbn'* dt;
    G(4:6,4:6) = - dt;
    % G(7:9,4:6) = - dt;
    G(10:12,7:9) = eye(3);
    G(13:15,10:12) = eye(3);
    G(16:18,13:15) = eye(3);
    G(19:21,16:18) = eye(3)* dt;
    G(22:24,19:21) = eye(3)* dt;

    %G = G*double(dt);
end