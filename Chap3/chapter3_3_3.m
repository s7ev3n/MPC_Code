clc;
clear all;
%% 参考轨迹生成
N=100;
T=0.05;
% Xout=zeros(2*N,3);
% Tout=zeros(2*N,1);
Xout=zeros(N,3);
Tout=zeros(N,1);
for k=1:1:N
    Xout(k,1)=k*T;
    Xout(k,2)=2;
    Xout(k,3)=0;
    Tout(k,1)=(k-1)*T;
end

%% Tracking a constant reference trajectory
Nx=3; %状态量个数
Nu =2; %控制量个数
Tsim =20; %仿真时间
X0 = [0 0 pi/3]; % 初始位置
[Nr,Nc] = size(Xout); % Nr is the number of rows of Xout
% Mobile Robot Parameters
c = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]; 
L = 1; %车辆轴距
Rr = 1; %轮胎直径
w = 1; %车轮转速
% Mobile Robot variable Model
vd1 = Rr*w; % 参考系统的纵向速度
vd2 = 0; %参考系统前轮偏角

%矩阵定义
x_real=zeros(Nr,Nc);
x_piao=zeros(Nr,Nc);
u_real=zeros(Nr,2);
u_piao=zeros(Nr,2);
x_real(1,:)=X0;
x_piao(1,:)=x_real(1,:)-Xout(1,:);
X_PIAO=zeros(Nr,Nx*Tsim);
XXX=zeros(Nr,Nx*Tsim);%用于保持每个时刻预测的所有状态值
q=[1 0 0;0 1 0;0 0 0.5];
Q_cell=cell(Tsim,Tsim);
for i=1:1:Tsim
    for j=1:1:Tsim
        if i==j
            Q_cell{i,j}=q;
        else 
            Q_cell{i,j}=zeros(Nx,Nx);
        end 
    end
end
Q=cell2mat(Q_cell);
R=0.1*eye(Nu*Tsim,Nu*Tsim); %Nu是控制量个数

%MPC主体
for i=1:1:Nr
    t_d =Xout(i,3);
    a=[1    0   -vd1*sin(t_d)*T;
       0    1   vd1*cos(t_d)*T;
       0    0   1;];
    b=[cos(t_d)*T   0;
       sin(t_d)*T   0;
       0            T;];     
    A_cell=cell(Tsim,1);
    B_cell=cell(Tsim,Tsim);
     for j=1:1:Tsim
        A_cell{j,1}=a^j;
        for k=1:1:Tsim
           if k<=j
                B_cell{j,k}=(a^(j-k))*b;
           else
                B_cell{j,k}=zeros(Nx,Nu);
           end
        end
    end
    A=cell2mat(A_cell);
    B=cell2mat(B_cell);
    
    H=2*(B'*Q*B+R);
    f=2*B'*Q*A*x_piao(i,:)';
    A_cons=[];
    b_cons=[];
    lb=[-1;-1];
    ub=[1;1];
    tic
    [X,fval(i,1),exitflag(i,1),output(i,1)]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub);
    toc
    X_PIAO(i,:)=(A*x_piao(i,:)'+B*X)';
    if i+j<Nr
         for j=1:1:Tsim
             XXX(i,1+3*(j-1))=X_PIAO(i,1+3*(j-1))+Xout(i+j,1);
             XXX(i,2+3*(j-1))=X_PIAO(i,2+3*(j-1))+Xout(i+j,2);
             XXX(i,3+3*(j-1))=X_PIAO(i,3+3*(j-1))+Xout(i+j,3);
         end
    else
         for j=1:1:Tsim
             XXX(i,1+3*(j-1))=X_PIAO(i,1+3*(j-1))+Xout(Nr,1);
             XXX(i,2+3*(j-1))=X_PIAO(i,2+3*(j-1))+Xout(Nr,2);
             XXX(i,3+3*(j-1))=X_PIAO(i,3+3*(j-1))+Xout(Nr,3);
         end
    end
    u_piao(i,1)=X(1,1);
    u_piao(i,2)=X(2,1);
    Tvec=[0:0.05:4];
    X00=x_real(i,:);
    vd11=vd1+u_piao(i,1);
    vd22=vd2+u_piao(i,2);
    XOUT=dsolve('Dx-vd11*cos(z)=0','Dy-vd11*sin(z)=0','Dz-vd22=0','x(0)=X00(1)','y(0)=X00(2)','z(0)=X00(3)');
     t=T; 
     x_real(i+1,1)=eval(XOUT.x);
     x_real(i+1,2)=eval(XOUT.y);
     x_real(i+1,3)=eval(XOUT.z);
     if(i<Nr)
         x_piao(i+1,:)=x_real(i+1,:)-Xout(i+1,:);
     end
    u_real(i,1)=vd1+u_piao(i,1);
    u_real(i,2)=vd2+u_piao(i,2);
    
    figure(1);
    plot(Xout(1:Nr,1),Xout(1:Nr,2));
    hold on;
    plot(x_real(i,1),x_real(i,2),'r*');
    title('跟踪结果对比');
    xlabel('横向位置X');
    axis([-1 5 -1 3]);
    ylabel('纵向位置Y');
    hold on;
    for k=1:1:Tsim
         X(i,k+1)=XXX(i,1+3*(k-1));
         Y(i,k+1)=XXX(i,2+3*(k-1));
    end
    X(i,1)=x_real(i,1);
    Y(i,1)=x_real(i,2);
    plot(X(i,:),Y(i,:),'y')
    hold on;
    
end

%% 以下为绘图部分
figure(2)
subplot(3,1,1);
plot(Tout(1:Nr),Xout(1:Nr,1),'k--');
hold on;
plot(Tout(1:Nr),x_real(1:Nr,1),'k');
%grid on;
%title('状态量-横向坐标X对比');
xlabel('采样时间T');
ylabel('横向位置X')
subplot(3,1,2);
plot(Tout(1:Nr),Xout(1:Nr,2),'k--');
hold on;
plot(Tout(1:Nr),x_real(1:Nr,2),'k');
%grid on;
%title('状态量-横向坐标Y对比');
xlabel('采样时间T');
ylabel('纵向位置Y')
subplot(3,1,3);
plot(Tout(1:Nr),Xout(1:Nr,3),'k--');
hold on;
plot(Tout(1:Nr),x_real(1:Nr,3),'k');
%grid on;
hold on;
%title('状态量-\theta对比');
xlabel('采样时间T');
ylabel('\theta')

figure(3)
subplot(2,1,1);
plot(Tout(1:Nr),u_real(1:Nr,1),'k');
%grid on;
%title('控制量-纵向速度v对比');
xlabel('采样时间T');
ylabel('纵向速度')
subplot(2,1,2)
plot(Tout(1:Nr),u_real(1:Nr,2),'k');
%grid on;
%title('控制量-角加速度对比');
xlabel('采样时间T');
ylabel('角加速度')

figure(4)
subplot(3,1,1);
plot(Tout(1:Nr),x_piao(1:Nr,1),'k');
%grid on;
xlabel('采样时间T');
ylabel('e(x)');
subplot(3,1,2);
plot(Tout(1:Nr),x_piao(1:Nr,2),'k');
%grid on;
xlabel('采样时间T');
ylabel('e(y)');
subplot(3,1,3);
plot(Tout(1:Nr),x_piao(1:Nr,3),'k');
%grid on;
xlabel('采样时间T');
ylabel('e(\theta)');


