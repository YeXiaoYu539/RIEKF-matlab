function [ pn,ph,pe ] = TransformDXTBTD( DX,DY,DZ,Lon0,Lat0,Alt0 )
%UNTITLED4 �˴���ʾ�йش˺�����ժҪ
%   ����������ϵת�������춫ϵ

m_dE_LN = zeros(3,3);%����ϵ�����춫ϵת������

[ DX0,DY0,DZ0 ] = TransformLLATDX(Lon0,Lat0,Alt0);

d_x_d = DX - DX0;
d_y_d = DY - DY0;
d_z_d = DZ - DZ0;

Fai = Lat0;
Lamda = Lon0;

m_dE_LN(1,1) = -sin(Fai)*cos(Lamda);
m_dE_LN(1,2) = -sin(Fai)*sin(Lamda);
m_dE_LN(1,3) = cos(Fai);
m_dE_LN(2,1) = cos(Fai)*cos(Lamda);
m_dE_LN(2,2) = cos(Fai)*sin(Lamda);
m_dE_LN(2,3) = sin(Fai);
m_dE_LN(3,1) = -sin(Lamda);
m_dE_LN(3,2) = cos(Lamda);
m_dE_LN(3,3) = 0.0;

A = m_dE_LN*[d_x_d;d_y_d;d_z_d];
pn = A(1);
ph = A(2);
pe = A(3);


end

