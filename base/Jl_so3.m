function J = Jl_so3(w)
% theta = w;
theta = norm(w);
if theta < 1e-6
    J = eye(3);
else
    a = w / theta;
    J = double(sin(theta) / theta * eye(3) + (1 - sin(theta) / theta) * a * a' + ...
    ((1 - cos(theta)) / theta) * skewMat(a));
    % J = double(eye(3) + (theta - sin(theta) / theta) * a * a' + ...
    % ((1 - cos(theta)) / theta) * skewMat(a));
end
end
