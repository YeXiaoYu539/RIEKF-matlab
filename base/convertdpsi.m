function dpsi = convertdpsi(dpsi)
num = size(dpsi,1);
for i = 1:num
    if dpsi(i) > 180
        dpsi(i) = dpsi(i) -360;
    elseif dpsi(i) < -180
        dpsi(i) = dpsi(i) + 360;
    else 
        dpsi(i) = dpsi(i);
    end
end
end