function [ pn,ph,pe ] = TransformDXTBTD( DX,DY,DZ,Lon0,Lat0,Alt0 )
%UNTITLED4 此处显示有关此函数的摘要
%   将地心坐标系转换到北天东系

m_dE_LN = zeros(3,3);%地心系到北天东系转换矩阵

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

