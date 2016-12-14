function Theta=Func_Theta_Pos(Alpha)

if Alpha >= 3*pi/2
    Theta = Alpha-3*pi/2;
else
    Theta = Alpha+pi/2;
end

end