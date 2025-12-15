clear all
clc
close all

car = struct();%Definiciones del coche

g=9.81; %m/s^2

car.l=1.6; %longitud cotxe
car.a = 0.8;%Distancia ruedas del a cg
car.b = car.l-car.a;%Distancia ruedas tras a cg
car.Tf = 1.25;%Trackwith front
car.Tr = 1.25;%Trackwith rear
car.weight = 205+70; %Pes
car.Zrf=0.07; %Roll center front
car.Zrr=0.08; %Roll center rear
car.H=0.225;
car.Kf= 18370.86641/57.2958; %roll stiffnes front
car.Kr= 18370.86641/57.2958; %roll stiffnes rear
car.h= 0.3; 
car.copx=0.8; %centre de pressions


%% Inputs

