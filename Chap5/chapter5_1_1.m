%% 此程序功能：根据简化动力学模型(考虑小角度假设下)计算雅克比矩阵
% 版本：V1.0，编写时间2013.12.11
% 计算出的雅克比矩阵是与车辆固有参数紧密相关的，参数变化了，雅克比矩阵也就相应变化了
% 滑移率只是一个估计值，后续如有必要可以做出调整
clc
clear all;
%% 以下为程序
%车辆参数定义 
syms x_dot y_dot phi phi_dot Y X;%车辆状态量
syms delta_f  %前轮偏角,控制量
%syms sf sr;%分别为前后车轮的滑移率,需要提供
Sf=0.2; Sr=0.2;
%syms a b;%前后车轮距离车辆质心的距离，车辆固有参数
a=1.232;b=1.468;
%syms C_cf C_cr C_lf C_lr;%分别为前后车轮的纵横向侧偏刚度，车辆固有参数
Ccf=66900;Ccr=62700;Clf=66900;Clr=62700;
%syms m g I;%m为车辆质量，g为重力加速度，I为车辆绕Z轴的转动惯量，车辆固有参数
m=1723;g=9.8;I=4175;

% 车辆动力学模型
dy_dot=-x_dot*phi_dot+2*(Ccf*(delta_f-(y_dot+a*phi_dot)/x_dot)+Ccr*(b*phi_dot-y_dot)/x_dot)/m;
dx_dot=y_dot*phi_dot+2*(Clf*Sf+Clr*Sr+Ccf*delta_f*(delta_f-(y_dot+phi_dot*a)/x_dot))/m;
%dphi_dot=dphi_dot;
dphi_dot=(2*a*Ccf*(delta_f-(y_dot+a*phi_dot)/x_dot)-2*b*Ccr*(b*phi_dot-y_dot)/x_dot)/I;
Y_dot=x_dot*sin(phi)+y_dot*cos(phi);
X_dot=x_dot*cos(phi)-y_dot*sin(phi);

% 雅克比矩阵求解
f=[dy_dot;dx_dot;phi_dot;dphi_dot;Y_dot;X_dot];%动力学模型
kesi=[y_dot,x_dot,phi,phi_dot,Y,X];%系统状态量
v=delta_f;
R=jacobian(f,kesi);%矩阵A(t)-连续
R2=jacobian(f,v);%矩阵B(t)-连续

% 控制矩阵求解(将连续矩阵转换为离散矩阵，采用近似算法  A=I+T*A(t),B=T*B(t))
I=eye(6);
syms T;
A=I+T*R;
B=T*R2;
A1=vpa(A,3);
B1=vpa(B,3);


