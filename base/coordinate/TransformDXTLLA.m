function [ Lon,Lat,Alt ] = TransformDXTLLA( DX,DY,DZ )
%TransformDXTLLA �˴���ʾ�йش˺�����ժҪ
%   ����������ϵת������γ��
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

