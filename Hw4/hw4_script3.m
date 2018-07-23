% Tahsincan Köse
% 2188423


function out = hw4_script3(q,rb,a1,a2,p0,r0)
    x = q(1);
    y = q(2);
    theta_1 = q(3);
    theta_2 = q(4);
    
    p1 =[x + a1*cosd(theta_1), y + a1*sind(theta_1)]; % Joint 1 position
    pe =[p1(1) + a2*cosd(theta_1+theta_2),p1(2) + a2*sind(theta_1+theta_2)]; % End effector position
    
    link1 = p1 - [x y]; % Has no effective use. Just written to denote what it means.
    ee_link = pe-p1; % Same as above.
   
    if hw4_script2_geo([x y],p0,rb,r0)==1 || hw4_script1(p1,[x y],p0,r0)==1 || hw4_script1(pe,p1,p0,r0)==1
        out = 1;
    else
        out = 0;
    end

end
