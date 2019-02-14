% Name: Tahsincan Kose
% ID: 2188423
% Usage: Give all arguments as scalars.
function q2_a(x,y,v,w,theta,delta)
    figure(1);
    hold on;
    pause on;
    axis([x-10 x+10 y-10 y+10]);
    quiver(x,y,cosd(theta)/2,sind(theta)/2,'ok','MarkerSize',15);
    for i=1:10
        pause(0.5);
        [x,y,theta] = motion_model_ideal(x,y,v,w,theta,delta/10);
        quiver(x,y,cosd(theta)/2,sind(theta)/2,'ok','MarkerSize',15);
    end
    
end


function [x_,y_,theta_] = motion_model_ideal(x,y,v,w,theta,delta)
    r = abs(v/w);
    x_ = x - r*sind(theta) + r*sind(theta + delta*w);
    y_ = y + r*cosd(theta) - r*cosd(theta + delta*w);
    theta_ = theta + delta*w;
end