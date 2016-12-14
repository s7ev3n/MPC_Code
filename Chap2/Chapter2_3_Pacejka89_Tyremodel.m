%Pacejka'89轮胎模型认为轮胎在垂直、侧向方向上是线性的、阻尼为常量
%这在侧向加速度常见范围≤0.4g，侧偏角≤5°的情景下对常规轮胎具有很高的拟合精度
%这个公式里没有考虑地面摩擦系数的影响
%%
Fz=5;%垂直载荷，单位：KN
%Longitudinal Force(pure longitudinal slip)  
%input:slip ratio s 
s=linspace(-20,20,40);                       %-20~20插值40个，生成滑移率横坐标
%*************longitudinal coefficients*******************************%
b0=2.37272;
b1=-9.46;
b2=1490;
b3=130;
b4=276;
b5=0.0886;
b6=0.00402;
b7=-0.0615;
b8=1.2;
b9=0.0299;
b10=-0.176;
%**********parameters *********************%
Cx=b0;%曲线形状因子
Dx=b1*Fz^2+b2*Fz;%曲线巅因子
BCDx=(b3*Fz^2+b4*Fz)*exp(-b5*Fz);%纵向力零点处的纵向刚度
Bx=BCDx/(Cx*Dx);%刚度因子
Shx=b9*Fz+b10;%曲线的水平方向漂移
kx=s+Shx;%输入变量X1
Svx=0;%%曲线的垂直方向漂移
Ex=b6*Fz^2+b7*Fz+b8;%曲线的曲率因子
Fx=Dx*sin(Cx*atan(Bx*kx-Ex*(Bx*kx-atan(Bx*kx))))+Svx; %纵向力的计算公式

figure (1);
plot(s,Fx);%绘制纵向力曲线
grid  on  %绘制栅格
set(gca,'xlim',[-20 20]);                              %设置x轴范围 
set(gca,'xtick',[-20:5:20]);                          %设置x轴间隔 
set(gca,'ylim',[-8000 8000])                        %设置x轴范围 
set(gca,'ytick',[-8000:1000:8000]);                   %设置x轴间隔 
xlabel('纵向滑移率'); 
ylabel('纵向力/（N）'); 
title('纵向力(纯纵滑)');

%%
%Lateral Force(pure side slip)  
%input:横向侧偏 是侧偏角 alpha
alpha=linspace(-8,8,16);   %-8~8插值16个，生成侧偏角横坐标
r=0;  %外倾角，设为零
%*************lateral coefficients*******************************%
a0 = 1.65;
a1 = -34;
a2 = 1250;
a3 = 3036;
a4 = 12.8;
a5 = 0.00501;
a6 = -0.02103;
a7 = 0.77394;
a8 = 0.0022890;
a9 = 0.013442;
a10 = 0.003709;
a11 = 19.1656;
a12 = 1.21356;
a13 = 6.26206;

%**********parameters *********************%
Cy=a0;%曲线形状因子
Dy=a1*Fz^2+a2*Fz;%曲线巅因子
BCDy=a3*sin(2*atan(Fz/a4))*(1-a5*abs(r));%侧向力零点处的侧向刚度
By=BCDy/(Cy*Dy);%刚度因子
Shy=a9*Fz+a10+a8*r;%曲线的水平方向漂移
ky=alpha+Shy;%输入变量X
Svy=a11*Fz*r+a12*Fz+a13;%曲线的垂直方向漂移
Ey=a6*Fz^2+a7;%曲线曲率因子
%********************lateral force formulation
Fy0=Dy*sin(Cy*atan(By*ky-Ey*(By*ky-atan(By*ky))))+Svy; %纵向力的计算公式

figure (2);
plot(alpha,Fy0);
grid  
set(gca,'xlim',[-8 8]);                              %设置x轴范围 
set(gca,'xtick',[-8:1:8]);                          %设置x轴间隔 
set(gca,'ylim',[-8000 8000])                        %设置x轴范围 
set(gca,'ytick',[-8000:1000:8000]);                   %设置x轴间隔 
xlabel('侧偏角'); 
ylabel('侧向力/（N）'); 
title('侧向力(纯侧偏)');

%%
%%Aligning Torque(pure side slip) 
%input:侧偏角 
%**********************ALIGNING_COEFFICIENTS*****************%
 c0 = 2.34000;
 c1 = 1.4950;
 c2 = 6.416654;
 c3 = -3.57403;
 c4 = -0.087737;
 c5 = 0.098410;
 c6 = 0.0027699;
 c7 = -0.0001151;
 c8 = 0.1000;
 c9 = -1.33329;
 c10 = 0.025501;
 c11 = -0.02357;
 c12 = 0.03027;
 c13 = -0.0647;
 c14 = 0.0211329;
 c15 = 0.89469;
 c16 = -0.099443;
 c17 = -3.336941;
%**********parameters *********************%
Cz=c0;%曲线形状因子
Dz=c1*Fz^2+c2*Fz;%曲线巅因子
BCDz=(c3*Fz^2+c4*Fz)*(1-c5*abs(r))*exp(-c5*Fz);%回正力矩零点处的扭转刚度
Bz=BCDz/(Cz*Dz);%刚度因子
Shz=c11*r+c12*Fz+c13;%曲线的水平方向漂移
kz=alpha+Shz;%输入变量X
Svz=r*(c14*Fz^2+c15*Fz)+c16*Fz+c17;%曲线的垂直方向漂移
Ez=(c7*Fz^2+c8*Fz+c9)*(1-c10*abs(r));%曲线曲率因子
%********************aligning torque formulation
Mz0=Dz*sin(Cz*atan(Bz*kz-Ez*(Bz*kz-atan(Bz*kz))))+Svz; %纵向力的计算公式

figure (3);
plot(alpha,Mz0);
grid  
set(gca,'xlim',[-8 8]);                         %设置x轴范围 
set(gca,'xtick',[-8:1:8]);                      %设置x轴间隔 
set(gca,'ylim',[-80 80])                        %设置x轴范围 
set(gca,'ytick',[-80:10:80]);                   %设置x轴间隔 
xlabel('侧偏角'); 
ylabel('回正力矩/（N）'); 
title('回正力矩(纯侧偏)');