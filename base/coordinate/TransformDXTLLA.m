function [ Lon,Lat,Alt ] = TransformDXTLLA( DX,DY,DZ )
%TransformDXTLLA 此处显示有关此函数的摘要
%   将地心坐标系转换到经纬高
C_EARTH = 6378137;

x = DX;
y = DY;
z = DZ;

dr = sqrt(x*x + y*y + z*z);

Alt= dr - C_EARTH;
Lat = asin(z / dr);

if y>=0.0
        Lon = acos(x / sqrt(x*x + y*y));
else
        Lon = -acos(x / sqrt(x*x + y*y));
end
    
end

