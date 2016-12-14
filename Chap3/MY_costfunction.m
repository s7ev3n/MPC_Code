
function cost = MY_costfunction(x, State_Initial, Np, Nc,T,Xref,Yref,PHIref,Q,R)
cost = 0;
l = 1;
%矩阵初始化，包含当前状态、预测状态、状态偏差和控制量
X=State_Initial(1,1);
Y=State_Initial(1,2);
PHI=State_Initial(1,3);

X_predict = zeros(Np,1);
Y_predict = zeros(Np,1);
PHI_predict = zeros(Np,1);

X_error = zeros(Np+1,1);
Y_error = zeros(Np+1,1);
PHI_error = zeros(Np+1,1);

v = zeros(Np,1); %控制量v
delta_f = zeros(Np,1); %控制量delta_f

%状态更新
for i =1:1:Np
    if i == 1
        v(i,1) = x(1);
        delta_f(i,1) = x(2);
        X_predict(i,1) = X +T*v(i,1)*cos(PHI);
        Y_predict(i,1) = Y +T*v(i,1)*sin(PHI);
        PHI_predict(i,1) = PHI_predict(i-1)+T*v(i,1)*tan(delta_f(i,1))/l;
    else
        v(i,1)=x(3);
        delta_f(i,1)=x(4);
        X_predict(i,1)=X_predict(i-1)+T*v(i,1)*cos(PHI_predict(i-1));
        Y_predict(i,1)=Y_predict(i-1)+T*v(i,1)*sin(PHI_PHI_predict(i-1));
        PHI_predict(i,1)=PHI_predict(i-1)+T*v(i,1)*tan(delta_f(i,1))/l;
    end
