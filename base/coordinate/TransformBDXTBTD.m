function [ xn,xh,xe ] = TransformBDXTBTD(pn,pe,pd )
%TransformBDXTBTD 此处显示有关此函数的摘要
%   将北东下系转换到北东天坐标系
xn = pn;
xh = -pd;
xe = pe;

end

