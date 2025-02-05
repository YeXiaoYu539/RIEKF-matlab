function [std,rmse] = err_cal(data)

num = size(data,1);
std = mean(data);
rmse = sqrt(sum(data.^2)/num);
end