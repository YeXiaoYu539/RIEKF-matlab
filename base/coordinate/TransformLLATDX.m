function [ DX,DY,DZ ] = TransformLLATDX( Lon,Lat,Alt )
%TransformLLATDX �˴���ʾ�йش˺�����ժҪ
%   ����γ��ת������������ϵ
C_EARTH = 6378137;
dr = Alt + C_EARTH;
DX = dr*cos(Lat)*cos(Lon);
DY = dr*cos(Lat)*sin(Lon);
DZ = dr*sin(Lat);


end

