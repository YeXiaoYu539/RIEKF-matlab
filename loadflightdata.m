 clc;
 clear all;
 close all;

 data = xlsread('trainflightrawdata\data_156_20210709_ID5.csv',1);
%data = xlsread('testflightrawdata\datadata_180_飞行数据20210427_穿越门框_yrw15_5.csv',1);
%data = xlsread('trainflightrawdata\data_125_20210714.csv',1);

% load("NNModel\lstmnet.mat");
startNum = 3;
endNum = length(data(:,1));
% endNum = 3000;
dataRange = startNum:endNum;
%% time
All.t = (startNum:endNum)*0.01;All.vt = data(dataRange,3);
% All.t = All.t';
%% value
All.altitude = data(dataRange,4);   %altitude
All.vn = data(dataRange,5);         %velocity
All.ve = data(dataRange,6);
All.mhdot = data(dataRange,7);

All.psd = data(dataRange,8);
All.yrw = data(dataRange,9);
All.xrw = data(dataRange,10);
All.drange = data(dataRange,13);
All.mh = data(dataRange,14);
All.phiL = data(dataRange,15);
All.thetaL = data(dataRange,16);
All.psiL = data(dataRange,17);
All.p = data(dataRange,18);
All.q = data(dataRange,19);
All.r = data(dataRange,20);
All.Nx = data(dataRange,21);
All.Ny = data(dataRange,22);
All.Nz = data(dataRange,23);
All.Qbar = data(dataRange,24);
All.Vc = data(dataRange,25);
All.href = data(dataRange,27);
%% 三维位置
All.Xnorth = data(dataRange,30);
All.Yeast = data(dataRange,31);
All.hrefAL = data(dataRange,32);
All.apthL = data(dataRange,37);
All.aplonL = data(dataRange,38);
All.aplatL = data(dataRange,39);
All.apyawL = data(dataRange,40);
All.apnwsL = data(dataRange,41);
All.apbrkL = data(dataRange,42);
All.thrustcmd = data(dataRange,43);
All.vtcmd = data(dataRange,44);
All.nzcmd = data(dataRange,45);
All.nycmd = data(dataRange,46);
All.phicmd = data(dataRange,47);
All.gamacmd = data(dataRange,48);
All.hcmd = data(dataRange,49);
All.psdcmd = data(dataRange,50);
All.yrwcmd = data(dataRange,51);
All.k = data(dataRange,54);
All.auto = data(dataRange,61);
All.RClost = data(dataRange,68);
All.LockCH = data(dataRange,69);
All.TakeoffCH = data(dataRange,70);
All.ModeselectCH = data(dataRange,71);
All.ManualCH = data(dataRange,72);
All.LockUL = data(dataRange,73);
All.TakeoffUL = data(dataRange,74);
All.ModeSelect = data(dataRange,75);
All.Manual = data(dataRange,76);
All.thrust = data(dataRange,77);
All.decl = data(dataRange,78);
All.dacl = data(dataRange,79);
All.drcl = data(dataRange,80);
All.nwscl = data(dataRange,81);
All.appc = data(dataRange,82);
All.aprc = data(dataRange,83);
All.apyc = data(dataRange,84);
All.fmphase = data(dataRange,86);
All.door_incamx = data(dataRange,87);
All.door_incamy = data(dataRange,88);
All.door_incamz = data(dataRange,89);
All.can_indoorx = data(dataRange,93);
All.can_indoory = data(dataRange,94);
All.can_indoorz = data(dataRange,95);
%% 经纬度
All.lon = data(dataRange,103);
All.lat = data(dataRange,104);

All.D_confidenceFG  = data(dataRange,105);
All.CVmode = data(dataRange,107);
All.INS_phi = data(dataRange,108);
All.INS_theta = data(dataRange,109);
All.INS_psi = data(dataRange,110);
All.INS_vn = data(dataRange,111);
All.INS_ve = data(dataRange,112);
All.INS_vd = data(dataRange,113);
All.INS_xn = data(dataRange,114);
All.INS_xe = data(dataRange,115);
All.INS_xd = data(dataRange,116);
All.GPSMode = data(dataRange,117);
All.Alt_Ps = data(dataRange,118);
All.baro = data(dataRange,120);
All.mx = data(dataRange,124);
All.my = data(dataRange,125);
All.mz = data(dataRange,126);
