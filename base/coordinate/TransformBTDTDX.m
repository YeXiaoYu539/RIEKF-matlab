function [ DX,DY,DZ ] = TransformBTDTDX( pn,ph,pe,Lon0,Lat0,Alt0 )
%TransformBTDTDX 此处显示有关此函数的摘要
%   将北天东系转换到地心坐标系
m_dN_LE = zeros(3,3);

[ DX0,DY0,DZ0 ] = TransformLLATDX(Lon0,Lat0,Alt0);

Fai = Lat0;
Lamda = Lon0;

x_bt = pn;
y_bt = ph;
z_bt = pe;

m_dN_LE(1,1) = -sin(Fai)*cos(Lamda);
m_dN_LE(1,2) = cos(Fai)*cos(Lamda);
m_dN_LE(1,3) = -sin(Lamda);
m_dN_LE(2,1) = -sin(Fai)*sin(Lamda);
m_dN_LE(2,2) = cos(Fai)*sin(Lamda);
m_dN_LE(2,3) = cos(Lamda);
m_dN_LE(3,1) = cos(Fai);
m_dN_LE(3,2) = sin(Fai);
m_dN_LE(3,3) = 0.0;

A = m_dN_LE*[x_bt;y_bt;z_bt];
x_dx = A(1);
y_dx = A(2);
z_dx = A(3);

DX = DX0 + x_dx;
DY = DY0 + y_dx;
DZ = DZ0 + z_dx;

end

