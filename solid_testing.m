clear;
clc;
addpath('functions');
format long;


a = v3(1,1,1);
b = v3(1,1,-1);
c = v3(1,-1,1);
d = v3(1,-1,-1);
e = v3(-1,1,1);
f = v3(-1,1,-1);
g = v3(-1,-1,1);
h = v3(-1,-1,-1);
i = v3(0,0,0);
j = v3(0,0,1);
k = v3(0,0,-1);

s = solid([a,b,c,d,e,f,g,h,i,j,k]);
s.print();

fprintf("------------------------------\n");
s.setDirection( 10, 11, 2, 4, v3(1,0,0) );
s.print();