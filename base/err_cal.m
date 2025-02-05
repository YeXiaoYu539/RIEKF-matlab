function [std,rmse] = err_cal(data,true_data,flag)

err = abs(data - true_data);
if strcmp(flag,'att')
  err(:,3) = convertdpsi(err(:,3));
end
num = size(true_data,1);
std = mean(err);
rmse = sqrt(sum(err.^2)/num);
end