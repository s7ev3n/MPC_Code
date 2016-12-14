function [sys,x0,str,ts] = MPC_TrajPlanner(t,x,u,flag)
% 该程序功能：用点质量模型设计规划期，能够规避障碍物
% 程序版本 V1.0，MATLAB版本：R2011a,采用S函数的标准形式，
% 程序编写日期 2013.12.17
% 最近一次改写 2014.02.24
% 状态量=[y_dot,x_dot,phi,,Y,X]，控制量为前轮偏角ay


switch flag,
 case 0  %flag=0表示处于初始化状态，此时用函数mdlInitializeSizes进行初始化
  [sys,x0,str,ts] = mdlInitializeSizes; % Initialization
  
 case 2  %flag=2表示此时要计算下一个离散状态
  sys = mdlUpdates(t,x,u); % Update discrete states
  
 case 3 %flag=3表示此时要计算输出
  sys = mdlOutputs(t,x,u); % Calculate outputs
 
%  case 4
%   sys = mdlGetTimeOfNextVarHit(t,x,u); % Get next sample time 

 case {1,4,9} % Unused flags
    % flag=1表示此时要计算连续状态的微分
    %flag=4表示此时要计算下一次采样的时间，只在离散采样系统中有用，主要用于变步长的设置
    %flag=9表示此时系统要结束，一般来说写上在mdlTerminate函数中写上sys=[]就可
  sys = [];
  
 otherwise
  error(['unhandled flag = ',num2str(flag)]); % Error handling
end
% End of dsfunc.

%==============================================================
% Initialization
%==============================================================

function [sys,x0,str,ts] = mdlInitializeSizes

% Call simsizes for a sizes structure, fill it in, and convert it 
% to a sizes array.

sizes = simsizes; %用于设置模块参数的结构体用simsizes来生成
sizes.NumContStates  = 0;%模块连续状态变量的个数
sizes.NumDiscStates  = 5;%模块离散状态变量的个数
sizes.NumOutputs     = 10;%输出期望的Y和phi,给出的是拟合曲线的系数如果有需要可以考虑dphi,dphi=ay/x_dot;
sizes.NumInputs      = 6;%模块输入变量的个数
sizes.DirFeedthrough = 1; %模块是否存在直接贯通
sizes.NumSampleTimes = 1;%模块的采样次数，至少是一个
sys = simsizes(sizes); %设置完后赋给sys输出

x0 =[0.001;0.0001;0.0001;0.00001;0.00001;];    %状态变量设置
%global U;
%U=[0];%控制量初始化,U为一维的
% global x;
% x = zeros(md.ne + md.pye + md.me + md.Hu*md.me,1);   
% Initialize the discrete states.
str = [];             %保留参数， Set str to an empty matrix.
ts  = [0.1 0];       % 采样周期: [period, offset],这里轨迹规划的周期设为100ms
%End of mdlInitializeSizes
		      
%==============================================================
% Update the discrete states
%==============================================================
function sys = mdlUpdates(t,x,u)
  
sys = x; %更新状态变量
%End of mdlUpdate.

%==============================================================
% Calculate outputs
%==============================================================
function sys = mdlOutputs(t,x,u)
    %u是CarSim的输出
    %x是状态量，x=[y_dot,x_dot,phi,,Y,X]
    %t是时间变量
    tic
    Nx=5;%状态量的个数， 
    %Nu=1;%控制量的个数，控制量为前轮偏角ay
    %Ny=2;%输出量的个数，这边falcone在LTV里面验证了两个输出量没有三个输出量的控制效果好，这边可以先用两个输出量
    Np =15;%预测步长
    Nc=2;%控制步长
    Nobs=6;%障碍物个数
    T=0.1;%Sample Time
   
    %输入接口转换,x_dot后面加一个非常小的数，是防止出现分母为零的情况
    % y_dot=u(1)/3.6-0.000000001*0.4786;%CarSim输出的是km/h，转换为m/s
    y_dot=u(1)/3.6; %速度单位是km/h，转换为m/s
    x_dot=u(2)/3.6+0.0001;%CarSim输出的是km/h，转换为m/s.加一个很小的数是为了防止分母为零
    phi=u(3)*pi/180; %CarSim输出的为角度，角度转换为弧度
    phi_dot=u(4)*pi/180;%角速度，角度转换为弧度
    Y=u(5);%单位为m
    X=u(6);%单位为米
   
%% 参考轨迹生成
    shape=2.4;%参数名称，用于参考轨迹生成
    dx1=25;dx2=21.95;%没有任何实际意义，只是参数名称
    dy1=4.05;dy2=5.7;%没有任何实际意义，只是参数名称
    Xs1=27.19;Xs2=56.46;%参数名称
  	X_phi=1:1:220;%这个点的区间是根据纵向速度（x_dot）来定的，如果速度为10m/s则区间=10*0.1=1
    z1=shape/dx1*(X_phi-Xs1)-shape/2;
    z2=shape/dx2*(X_phi-Xs2)-shape/2;
    Y_ref=dy1/2.*(1+tanh(z1))-dy2/2.*(1+tanh(z2));
   % phi_ref=atan(dy1*(1./cosh(z1)).^2*(1.2/dx1)-dy2*(1./cosh(z2)).^2*(1.2/dx2));
%% 矩阵转换。将状态变量转化为状态变量矩阵
    State_Initial=zeros(Nx,1);
    State_Initial(1,1)=y_dot;
    State_Initial(2,1)=x_dot;
    State_Initial(3,1)=phi;
    State_Initial(4,1)=Y;
    State_Initial(5,1)=X;    
%% 障碍物信息设置
    X_obstacle=zeros(Nobs,1);
    X_obstacle(1:2)=30;
    X_obstacle(3:4)=35;
    X_obstacle(5:6)=32.5;
    Y_obstacle=zeros(Nobs,1);
    Y_obstacle(1)=0.5;
    Y_obstacle(2)=1;
    Y_obstacle(3)=0.5;
    Y_obstacle(4)=1;
    Y_obstacle(5)=0.5;
    Y_obstacle(6)=1;
    Yref=(Y_ref(1,round(State_Initial(5,1))+1:round(State_Initial(5,1))+15))';%Yref采用的是近似算法，此处为局部期望路径
    Q=100*eye(Np,Np);%这里设置评价矩阵，都设为了1。可以根据跟踪情况加以调整
    R=20*eye(Nc,Nc); %
    S=100;%避障函数的权重    
%% 开始求解过程
    %设置约束
    mu=0.4;%地面摩擦系数
    g=9.8;
    lb=[-mu*g;-mu*g];
    ub=[mu*g;mu*g];
    A=[];
    b=[];
    Aeq=[];
    beq=[];
    options = optimset('Algorithm','active-set');
    [A,fval,exitflag]=fmincon(@(x)MY_costfunction(x,State_Initial,Np,Nc,Nobs,T,Yref,Q,R,S,X_obstacle,Y_obstacle),[0;0;],A,b,Aeq,beq,lb,ub,[],options);%有约束求解，但速度慢
%   [A,fval,exitflag]=fminbnd(@(x)MY_costfunction(x,State_Initial,Np,Nc,T,Yref,Q,R,S),lb,ub);%只有上下界约束，但容易陷入局部最小
%   [A,fval,exitflag]=fminsearch(@(x)MY_costfunction(x,State_Initial,Np,Nc,Nobs,T,Yref,Q,R,S,X_obstacle,Y_obstacle),[0;0]);%无约束求解，速度最快
    fprintf('exitflag=%d\n',exitflag);
   
%% 计算输出
% 以下根据计算出的控制量推导所有的状态量
    y_dot_predict=zeros(Np,1);
    x_dot_predict=zeros(Np,1);
    phi_predict=zeros(Np,1);
    Y_predict=zeros(Np,1);
    X_predict=zeros(Np,1);

 for i=1:1:Np
     if i==Nc-1 
            ay(i)=A(1);
             % 以下完成状态量更新
            y_dot_predict(i,1)=State_Initial(1,1)+T*ay(i);
            x_dot_predict(i,1)=State_Initial(2,1);
            phi_predict(i,1)=State_Initial(3,1)+T*ay(i)/State_Initial(2,1);
            Y_predict(i,1)=State_Initial(4,1)+T*(State_Initial(2,1)*sin(State_Initial(3,1))+State_Initial(1,1)*cos(State_Initial(3,1)));
            X_predict(i,1)=State_Initial(5,1)+T*(State_Initial(2,1)*cos(State_Initial(3,1))-State_Initial(1,1)*sin(State_Initial(3,1)));  
      else %if i<=5
            ay(i)=A(2);%这种写法是仅仅考虑两个控制周期
            y_dot_predict(i,1)=y_dot_predict(i-1,1)+T*ay(i);
            x_dot_predict(i,1)=State_Initial(2,1);
            phi_predict(i,1)=phi_predict(i-1,1)+T*ay(i)/x_dot_predict(i-1,1);
            Y_predict(i,1)=Y_predict(i-1)+T*(State_Initial(2,1)*sin(phi_predict(i-1))+y_dot_predict(i-1)*cos(phi_predict(i-1)));
            X_predict(i,1)=X_predict(i-1)+T*(State_Initial(2,1)*cos(phi_predict(i-1))-y_dot_predict(i-1)*sin(phi_predict(i-1)));
      end 
 end
    Paramater_X_Y=polyfit(X_predict,Y_predict,4);
    Paramater_X_PHI=polyfit(X_predict,phi_predict,4);
    OutPut(1:5)=Paramater_X_Y;
    OutPut(6:10)=Paramater_X_PHI;
    sys=OutPut;
    toc
% End of mdlOutputs.
 %% 求代价函数的功能子函数
function cost = MY_costfunction(x,State_Initial,Np,Nc,Nobs,T,Yref,Q,R,S,X_obstacle,Y_obstacle)
    cost=0;
    y_dot=State_Initial(1,1);
    x_dot=State_Initial(2,1);
    phi=State_Initial(3,1);
    Y=State_Initial(4,1);
    X_start=State_Initial(5,1);
    
    y_dot_predict=zeros(Np,1);
    x_dot_predict=zeros(Np,1);
    phi_predict=zeros(Np,1);
    Y_predict=zeros(Np,1);
    X_predict=zeros(Np,1);
    Y_error=zeros(Np,1);
    J_obst=zeros(Np,1);
    ay=zeros(Np,1);
    
    for i=1:1:Np
        if i==Nc-1 
            ay(i,1)=x(1);
             % 以下完成状态量更新
            y_dot_predict(i,1)=y_dot+T*ay(i,1);
            x_dot_predict(i,1)=x_dot;
            phi_predict(i,1)=phi+T*ay(i)/x_dot;
            Y_predict(i,1)=Y+T*(x_dot*sin(phi)+y_dot*cos(phi));
            X_predict(i,1)=X_start+T*(x_dot*cos(phi)-y_dot*sin(phi));
            for j=1:1:Nobs
                J_obst(i,1)=J_obst(i,1)+1/(((X_predict(i,1))-X_obstacle(j,1))^2+(Y_predict(i,1)-Y_obstacle(j,1))^2+0.000001);
            end
        else %if i<=5
            ay(i,1)=x(2);%这种写法是仅仅考虑两个控制周期
            y_dot_predict(i,1)=y_dot_predict(i-1,1)+T*ay(i,1);
            x_dot_predict(i,1)=x_dot;
            phi_predict(i,1)=phi_predict(i-1,1)+T*ay(i)/x_dot_predict(i-1,1);
            Y_predict(i,1)=Y_predict(i-1)+T*(x_dot*sin(phi_predict(i-1))+y_dot_predict(i-1)*cos(phi_predict(i-1)));
            X_predict(i,1)=X_predict(i-1)+T*(x_dot*cos(phi_predict(i-1))-y_dot_predict(i-1)*sin(phi_predict(i-1)));
            for p=1:1:Nobs
                J_obst(i,1)=J_obst(i,1)+1/(((X_predict(i,1))-X_obstacle(p,1))^2+(Y_predict(i,1)-Y_obstacle(p,1))^2+0.000001);
            end
%             else 
%             ay(i)=x(2);
%             y_dot_predict(i,1)=y_dot_predict(i-1,1)+T*ay(i);
%             x_dot_predict(i,1)=x_dot;
%             phi_predict(i,1)=phi_predict(i-1,1)+T*ay(i)/x_dot_predict(i-1,1);
%             Y_predict(i,1)=Y_predict(i-1)+T*(x_dot*sin(phi_predict(5,1))+y_dot_predict(5,1)*cos(phi_predict(5,1)));
%             X_predict(i,1)=X_predict(i-1)+T*(x_dot*cos(phi_predict(5,1))-y_dot_predict(5,1)*sin(phi_predict(5,1)));
%             end 
        end
    %J_obst=J_obst+1/(((X_predict(i,1))-X_obstacle(2,1))^2+(Y_predict(i,1)-Y_obstacle(2,1))^2+0.00001);
        Y_error(i,1)=Y_predict(i,1)-Yref(i,1);%注意此处Yref与Y_ref要区分开来，Yref是局部期望路径,Y_ref为全局期望路径
    end 
        cost=cost+Y_error'*Q*Y_error+ay(1:2)'*R*ay(1:2)+S*sum(J_obst(:));
% End of CostFunction
