function [RefTraj_x,RefTraj_y,RefTraj_theta,RefTraj_delta]=Func_CircularReferenceTrajGenerate(Pos_x,Pos_y,CEN_x,CEN_y,Radius,N,Velo,Ts,L)
%RefTraj为要生成的参考路径
%Pos_x,Pos_y为车辆坐标
%CEN_x,CEN_y,Radius圆心与半径
%N要生成几个参考点，即预测空间。
%Velo,Ts车速与采样时间
%L汽车的轴距
RefTraj=zeros(N,4);%生成的参考路径
Alpha_init=Func_Alpha_Pos(CEN_x,CEN_y,Pos_x,Pos_y);%首先根据车辆位置和圆心确定alpha

Omega=Velo/Radius%已知车速和半径，可以求得角速度。

DFWA=atan(L/Radius);

for k=1:1:N
    Alpha(k)=Alpha_init+Omega*Ts*(k-1);
    RefTraj(k,1)=Radius*cos(Alpha(k))+CEN_x;%x
    RefTraj(k,2)=Radius*sin(Alpha(k))+CEN_y;%y
    RefTraj(k,3)=Func_Theta_Pos(Alpha(k));%theta  
 
    RefTraj(k,4)=DFWA;%前轮偏角，可以当做前馈量

end
RefTraj_x= RefTraj(:,1);
RefTraj_y= RefTraj(:,2);
RefTraj_theta= RefTraj(:,3);
RefTraj_delta= RefTraj(:,4);

end