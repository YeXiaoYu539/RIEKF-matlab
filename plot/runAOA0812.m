loadflightdata = 1;
if loadflightdata == 0
    drawfigure_newRIEKF;
    loadflightdata = 1;
end
%% set param
Alldata = cell(60,1);
% %% 参数拉偏设置
% phi0_rad = 0/57.3;
% gam0_rad = 0/57.3;
% psi0_rad = 0/57.3;
% 
% vn0 = 0;
% ve0 = 0;
% vd0 = 0;
% 
% pn0 = 0;
% pe0 = 0;
% pd0 = 0;

%% att Monte Carlo
for i = 1:60
    phi0_rad = (i - 30)/57.3;
    gam0_rad = (i - 30)/57.3;
    Alldata{i} = sim("GNSS_INS230612_AOA.slx")
end

%% vn Monte Carlo
Alldata_vn_deviation = cell(60,1);
for i = 1:60
    vn0 = (i - 30)/30;
    ve0 = (i - 30)/30;
    vd0 = (i - 30)/30;

    Alldata_vn_deviation{i} = sim("GNSS_INS230612_AOA.slx");
end
