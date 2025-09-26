clear;
clc;
addpath('functions');
format long

a = v3( 1, 1, 1);
b = v3( 1,-1,-1);
c = v3(-1, 1,-1);
d = v3(-1,-1, 1);

a = v3(0,sqrt(2),0);
b = v3(1,0,0);
c = v3(0.5,1,0);
d = v3(0.5,-1,0);

a = v3( 0, 0, 0);
b = v3( 1, 1, 1);
c = v3( 1, 1,-1);
d = v3( 1,-1, 1);
e = v3( 1,-1,-1);
f = v3(-1, 1, 1);
g = v3(-1, 1,-1);
h = v3(-1,-1, 1);
i = v3(-1,-1,-1);
j = v3( 0, 1, 1);
k = v3( 0,-1, 1);
l = v3( 1, 0, 0);
m = -l;


s = solid([a b c d e f g h i j k l m]);

s.print()
s.fixed_free_move(1, 2, v3(10,2,-2));
s.print()