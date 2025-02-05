function [ pn,pe,pd ] = TransformLLATBDX( Lon,Lat,Alt, Lon0,Lat0,Alt0)
%TransformLLATBDX 此处显示有关此函数的摘要
%   将经纬高转换到北东下坐标系
% //1.将经纬高转换为在地心坐标系下x,y,z坐标
[DX,DY,DZ]=TransformLLATDX(Lon,Lat,Alt);

% 	//2.将地心坐标系下x,y,z坐标转换为在北天东标系下x,y,z坐标
[xn,xh,xe]=TransformDXTBTD( DX,DY,DZ, Lon0,Lat0,Alt0);

% 	//3.将北天东坐标系下x,y,z坐标转换为在北东下标系下x,y,z坐标
[ pn,pe,pd ] = TransformBTDTBDX(xn,xh,xe);

end

