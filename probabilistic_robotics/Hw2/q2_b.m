% Name: Tahsincan Kose
% ID: 2188423
% Usage: Give all arguments as scalars.
function q2_b(x,y,v,w,theta,delta,a1,a2,a3,a4,a5,a6,N)
    figure(1);
    hold on;
    pause on;
    grid on;
    axis([x-2 x+8 y-2 y+8]);
    quiver(x,y,cosd(theta)/4,sind(theta)/4,'ob','MarkerSize',5);
    for j=1:N
        x_ = x; y_ = y; theta_ = theta;
        [x_,y_,theta_] = motion_model_noisy(x_,y_,v,w,theta_,delta,a1,a2,a3,a4,a5,a6);
       quiver(x_,y_,cos(theta_)/4,sin(theta_)/4,'ob','MarkerSize',5);
    end
end


function [x_,y_,theta_] = motion_model_noisy(x,y,v,w,theta,delta,a1,a2,a3,a4,a5,a6)
    v_hat = v + sample_normal_distribution(a1*v + a2*w);
    w_hat = w + sample_normal_distribution(a3*v + a4*w);
    r = v_hat/w_hat;
    x_ = x - r*sin(theta) + r*sin(theta + delta*w_hat);
    y_ = y + r*cos(theta) - r*cos(theta + delta*w_hat);
    theta_ = theta + delta*w_hat + sample_normal_distribution(a5*v+a6*w)*delta;
end

function sample = sample_normal_distribution(b)
    sample = 0;
    for i=1:12
        sample = sample + 2*rand() - 1;
    end
    sample = sample*b/6;
end