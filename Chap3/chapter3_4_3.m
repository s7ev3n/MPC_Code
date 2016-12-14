
clc;
clear all;
%% 参考轨迹生成
tic %tic用来保存当前时间，而后使用toc来记录程序完成时间
Nx=3;%状态量个数
Np=25;%预测时域
Nc=2;%控制时域
l=1;
N=100;%参考轨迹点数量
T=0.05;%采样周期
Xref=zeros(Np,1);
Yref=zeros(Np,1);
PHIref=zeros(Np,1);

%% 对性能函数各参数的初始化
State_Initial=zeros(Nx,1);%state=[y_dot,x_dot,phi,Y,X],此处为给定初始值
State_Initial(1,1)=0;%x 
State_Initial(2,1)=0;%y
State_Initial(3,1)=pi/6;%phi

Q=100*eye(Np+1,Np+1);
R=100*eye(Np+1,Np+1);

%% 开始求解
for j=1:1:N
    lb=[0.8;-0.44;0.8;-0.44];
    ub=[1.2;0.44;1.2;0.44];
    A=[];
    b=[];
    Aeq=[];
    beq=[];
    for Nref=1:1:Np
        Xref(Nref,1)=(j+Nref-1)*T;
        Yref(Nref,1)=2;
        PHIref(Nref,1)=0;
    end
    options = optimset('Algorithm','active-set');
    [A,fval,exitflag]=fmincon(@(x)MY_costfunction(x,State_Initial,Np,Nc,T,Xref,Yref,PHIref,Q,R),[0;0;0;0;],A,b,Aeq,beq,lb,ub,[],options);%有约束求解，但速度慢
    %[A,fval,exitflag]=fminbnd(@(x)MY_costfunction(x,State_Initial,Np,Nc,T,Yref,Q,R,S),lb,ub);%只有上下界约束，但容易陷入局部最小
    %[A,fval,exitflag]=fminsearch(@(x)MY_costfunction(x,State_Initial,Np,Nc,T,Xref,Yref,PHIref,Q,R),[0;0;0;0]);%无约束求解，速度最快
    v_actual=A(1);
    deltaf_actual=A(2);
    fval
    exitflag
    X00(1)=State_Initial(1,1);
    X00(2)=State_Initial(2,1);
    X00(3)=State_Initial(3,1);
    XOUT=dsolve('Dx-v_actual*cos(z)=0','Dy-v_actual*sin(z)=0','Dz-v_actual*tan(deltaf_actual)=0','x(0)=X00(1)','y(0)=X00(2)','z(0)=X00(3)');
    t=T;
    State_Initial(1,1)=eval(XOUT.x);
    State_Initial(2,1)=eval(XOUT.y);
    State_Initial(3,1)=eval(XOUT.z);
    
    figure(1)
    plot(State_Initial(1,1),State_Initial(2,1),'b*');
    axis([0 5 0 3]);
    hold on;
    plot([0,5],[2,2],'r--');
    hold on;
    
end 
 toc
