function [ pn,pe,pd ] = TransformLLATBDX( Lon,Lat,Alt, Lon0,Lat0,Alt0)
%TransformLLATBDX �˴���ʾ�йش˺�����ժҪ
%   ����γ��ת��������������ϵ
% //1.����γ��ת��Ϊ�ڵ�������ϵ��x,y,z����
[DX,DY,DZ]=TransformLLATDX(Lon,Lat,Alt);

% 	//2.����������ϵ��x,y,z����ת��Ϊ�ڱ��춫��ϵ��x,y,z����
[xn,xh,xe]=TransformDXTBTD( DX,DY,DZ, Lon0,Lat0,Alt0);

% 	//3.�����춫����ϵ��x,y,z����ת��Ϊ�ڱ����±�ϵ��x,y,z����
[ pn,pe,pd ] = TransformBTDTBDX(xn,xh,xe);

end

