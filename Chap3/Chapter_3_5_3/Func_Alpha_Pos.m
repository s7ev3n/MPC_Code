function K=Func_Alpha_Pos(Xb,Yb,Xn,Yn)
AngleY=Yn-Yb;
AngleX=Xn-Xb;
%***求Angle*******%
if Xb==Xn
    if Yn>Yb
        K=pi/2;
    else
        K=3*pi/2;
    end
else
    if Yb==Yn
        if Xn>Xb
            K=0;
        else
            K=pi;
        end
    else
        K=atan(AngleY/AngleX);
    end    
end
%****修正K,使之在0~360°之间*****%
   if (AngleY>0&&AngleX>0)%第一象限
        K=K;
    elseif (AngleY>0&&AngleX<0)||(AngleY<0&&AngleX<0)%第二、三象限
        K=K+pi;
    else if (AngleY<0&&AngleX>0)%第四象限
            K=K+2*pi;  
        else
            K=K;
        end
    end
end