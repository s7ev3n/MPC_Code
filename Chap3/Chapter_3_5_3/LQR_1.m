function LQR_1()
%这里先从简单开始，给定一个直线车道和车辆位置偏差。
%参考轨迹的生成方法有两种：
%1.车辆在Path上投影，然后在PATH上选取一系列的点作参考点
%*现在遇到的问题是Q R的参数怎么设置。而且通用性怎么办？*%

clear all;
close all;
clc;
%% 给定参数：

vel = 6; % 纵向车速,单位：m/s
L=2.85;%轴距
T=0.05;% sample time， control period
% 给定圆形参考轨迹
 CEN=[0,0];       % 圆心
 Radius=20;       % 半径

%% 设置参数
Hp =10;%predictive horizion, control horizon 
N_l=200;% 设置迭代次数

Nx=3;%状态变量参数的个数
Nu=1;%控制变量参数的个数

FWA=zeros(N_l,1);%前轮偏角
FWA(1,1)= 0; %初始状态的前轮偏角

x_real=zeros(Nx,N_l);%实际状态
x_real(:,1)= [22 0 pi/2]; %x0=车辆初始状态X_init初始状态
% x_piao=zeros(N_l,Nx);%实际状态与参考轨迹的误差
% 
% u_real=zeros(N_l,Nu);%实际的控制量
% u_piao=zeros(N_l,Nu);%实际控制量与参考控制量的误差

% X_PIAO=zeros(N_l,3*Hp);%通过DR估计的状态
% 
% XXX=zeros(N_l,3*Hp);%用于保持每个时刻预测的所有状态值

RefTraj=zeros(3,1);
Delta_x = zeros(3,1);

Q=[10 0 0; 0 10 0; 0 0 100];
R=[10];%r是对控制量误差的weighting matrice

Pk=[1 0 0; 0 1 0; 0 0 1]; %人为给定,相当于QN
Vk=[0 0 0]'; %人为给定,相当于QN

%%  算法实现
 u_feedBackward=0;
 u_feedForward=0;
 
 %*首先生成参考轨迹，画出图来作参考*%
 [RefTraj_x,RefTraj_y,RefTraj_theta,RefTraj_delta]=Func_CircularReferenceTrajGenerate(x_real(1,1),x_real(1,2),CEN(1),CEN(2),Radius,250,vel,T,L);

figure(1) %绘制参考路径
plot(RefTraj_x,RefTraj_y,'k')
xlabel('x','fontsize',14)
ylabel('y','fontsize',14)
title('Plot of x vs y - Ref. Trajectory');
legend('reference traj');
axis equal 
grid on
hold on


for i=1:1:N_l

    G_Test = 3;
    %先确定参考点和确定矩阵A,B.这里姑且认为A和B是不变的
    [RefTraj_x,RefTraj_y,RefTraj_theta,RefTraj_delta]=Func_CircularReferenceTrajGenerate(x_real(1,i),x_real(2,i),CEN(1),CEN(2),Radius,Hp,vel,T,L);
    u_feedForward = RefTraj_delta(G_Test);%前馈控制量
%     u_feedForward=0;
    Delta_x(1,1) = x_real(1,i) - RefTraj_x(G_Test);
    Delta_x(2,1) = x_real(2,i) - RefTraj_y(G_Test);
    Delta_x(3,1) = x_real(3,i) - RefTraj_theta(G_Test);
    if  Delta_x(3,1) > pi
         Delta_x(3,1) = Delta_x(3,1)-2*pi;
    else if Delta_x(3,1) < -1*pi
            Delta_x(3,1) = Delta_x(3,1) +2*pi;
        else
            Delta_x(3,1) = Delta_x(3,1);
        end            
    end
    
     % 通过Backward recursion 求K    
    for  j=Hp:-1:2   
        Pk_1 = Pk;
        Vk_1 = Vk;     
        A=[1    0   -vel*sin(RefTraj_theta(j-1))*T; 0    1   vel*cos(RefTraj_theta(j-1))*T; 0    0   1;];
%         B=[cos(RefTraj_theta(j-1))*T   0; sin(RefTraj_theta(j-1))*T   0; 0            vel*T/L;]; 
        COS2 = cos(RefTraj_delta(j-1))^2;
        B=[ 0 0  vel*T/(L*COS2)]'; 

        K = (B'*Pk_1*A)/(B'*Pk_1*B+R);
        Ku = R/(B'*Pk_1*B+R);
        Kv = B'/(B'*Pk_1*B+R);

        Pk=A'*Pk_1*(A-B*K)+Q;   
        Vk=(A-B*K)'*Vk_1 - K'*R*RefTraj_delta(j-1); 
    end
    
     u_feedBackward = -K*(Delta_x)-Ku*u_feedForward-Kv*Vk_1;  
    
    FWA(i+1,1)=u_feedForward+u_feedBackward;
    
     [x_real(1,i+1),x_real(2,i+1),x_real(3,i+1)]=Func_VehicleKineticModule_Euler(x_real(1,i),x_real(2,i),x_real(3,i),vel,FWA(i,1),FWA(i+1,1),T,L);  
     
    
end

%%   绘图
%        figure(1);
%     plot(RefTraj_x,RefTraj_y,'b')
%     hold on;
    plot(x_real(1,:),x_real(2,:),'r*');
    title('跟踪结果对比');
    xlabel('横向位置X');
    % axis([-1 5 -1 3]);
    ylabel('纵向位置Y');  


end