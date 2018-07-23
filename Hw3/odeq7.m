% Tahsincan Kose
% 2188423

function velocity = odeq7(t,initial_condition,angular)
    %syms x y Theta D R real
    x = initial_condition(1);
    y = initial_condition(2);
    Theta = initial_condition(3);
    D = 1;
    R = 0.2;
    M = [ sin((60 + Theta)*pi/180), -cos((60 + Theta)*pi/180), -(x*cos((Theta+60)*pi/180) + y*sin((Theta+60)*pi/180) + D);
          -sin(Theta*pi/180), cos(Theta*pi/180), x*cos(Theta*pi/180) + y*sin(Theta*pi/180) - D;
          -sin((60-Theta)*pi/180), -cos((60-Theta)*pi/180), y*sin((60-Theta)*pi/180) - x*cos((60-Theta)*pi/180) - D];
    
    velocity = (R/D)* inv(M) * angular;

end