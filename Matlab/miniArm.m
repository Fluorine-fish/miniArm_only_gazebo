%% 使用D-H建模，逆解
clear;
clc;
close all

L(1) = Link("revolute","alpha",-pi/2,"d",0.09955);
L(2) = Link('revolute','a',0.14,'alpha',pi,'offset',-pi/2);
L(3) = Link('revolute','alpha',-pi/2,'offset',-pi/2);
L(4) = Link('revolute','alpha',pi/2,'d',0.153);
L(5) = Link('revolute','alpha',-pi/2);
L(6) = Link('revolute','d',0.0875);

Arm = SerialLink(L,'name','miniArm')

% Arm.teach
% view(45,15);

qz = zeros(1,6);
q1 = [0 -pi/4 pi/4 0 pi/4 0];

Arm.plot3d(q1,'path','miniArmSTL','view', [45,15]);
