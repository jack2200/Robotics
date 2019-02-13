% Tahsincan Kose
% 2188423

function hw2_script1()
clf;
figure(1);
%hold on;
plotvol([-2 2 -2 2]);
for gamma=0:0.01:1
    C_gamma = (1 + 0.2*sin(12*pi*gamma)) * [cos(2*pi*gamma);sin(2*pi*gamma)];

    Theta = atan2(10*cos(2*pi*gamma) + 2*sin(14*pi*gamma) + 10*cos(12*pi*gamma)*sin(2*pi*gamma),10*cos(12*pi*gamma)*cos(2*pi*gamma) + 2*cos(14*pi*gamma) - 10*sin(2*pi*gamma));
    
    translation = transl2(C_gamma');
    rotation = trot2(Theta-pi/2);
    T = translation * rotation;
    trplot2(T,'length',0.1);
    pause on;
    pause(0.1)
end
