function [ Lon,Lat,Alt ] = TransformBDXTLLA( pned,LLA0 )
pn = pned(1);
pe = pned(2);
pd = pned(3);

Lon0 = LLA0(1);
Lat0 = LLA0(2);
Alt0 = LLA0(3);

%TransformBDXTLLA �˴���ʾ�йش˺�����ժҪ
%   ������������ϵת������γ��
% //1.�������±�ϵ��x,y,z����ת��Ϊ�ڱ��춫����ϵ��x,y,z����
[ xn,xh,xe ] = TransformBDXTBTD(pn,pe,pd);

% 	//2.�����춫��ϵ��x,y,z����ת��Ϊ�ڵ�������ϵ��x,y,z����
[DX,DY,DZ]=TransformBTDTDX(xn,xh,xe, Lon0,Lat0,Alt0);

% 	//3.����������ϵ��x,y,z����ת��Ϊ��γ��
[Lon,Lat,Alt]=TransformDXTLLA(DX,DY,DZ);

end

