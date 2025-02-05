function R = exp_so3(w)

w_x  = [0, -w(3), w(2);
        w(3), 0, -w(1);
        -w(2), w(1), 0];
theta = norm(w);
if theta < 1e-7
    A = 1;
    B = 0.5;
else
    A = double(sin(theta) / theta);
    B = double((1 - cos(theta)) / (theta * theta));
end

if theta == 0
    R = double(eye(3));
else
    R = double(eye(3) + A * w_x + B * w_x * w_x);
end
end