function [ Lon,Lat,Alt ] = TransformBDXTLLA( pned,LLA0 )
pn = pned(1);
pe = pned(2);
pd = pned(3);

Lon0 = LLA0(1);
Lat0 = LLA0(2);
Alt0 = LLA0(3);

%TransformBDXTLLA 此处显示有关此函数的摘要
%   将北东下坐标系转换到经纬高
% //1.将北东下标系下x,y,z坐标转换为在北天东坐标系下x,y,z坐标
[ xn,xh,xe ] = TransformBDXTBTD(pn,pe,pd);

% 	//2.将北天东标系下x,y,z坐标转换为在地心坐标系下x,y,z坐标
[DX,DY,DZ]=TransformBTDTDX(xn,xh,xe, Lon0,Lat0,Alt0);

% 	//3.将地心坐标系下x,y,z坐标转换为经纬高
[Lon,Lat,Alt]=TransformDXTLLA(DX,DY,DZ);

end

