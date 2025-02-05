function [ DX,DY,DZ ] = TransformLLATDX( Lon,Lat,Alt )
%TransformLLATDX 此处显示有关此函数的摘要
%   将经纬高转换到地心坐标系
C_EARTH = 6378137;
dr = Alt + C_EARTH;
DX = dr*cos(Lat)*cos(Lon);
DY = dr*cos(Lat)*sin(Lon);
DZ = dr*sin(Lat);


end

