function hw2_script3()
clf;
plotvol([-2 2 -2 2 -2 2]);
a1 = 0.75;
a2 = 0.5;
a3 = 0.2;
robot = SerialLink([ Revolute('a',a1) Revolute('a',a2) Revolute('a',a3)] , 'name', 'slave');

tf = 10;

a = 10*cos(12*pi*gamma_)*cos(2*pi*gamma_) + 2*cos(14*pi*gamma_) - 10*sin(2*pi*gamma_);
b = 10*cos(2*pi*gamma_) + 2*sin(14*pi*gamma_) + 10*cos(12*pi*gamma_)*sin(2*pi*gamma_);
for t=0:1:tf
    C_gamma = (1 + 0.2*sin(12*pi*gamma)) * [cos(2*pi*gamma);sin(2*pi*gamma)];
    Theta_d = atan2(10*cos(2*pi*gamma) + 2*sin(14*pi*gamma) + 10*cos(12*pi*gamma)*sin(2*pi*gamma),10*cos(12*pi*gamma)*cos(2*pi*gamma) + 2*cos(14*pi*gamma) - 10*sin(2*pi*gamma));
    
     
    x_d = C_gamma(1);
    y_d = C_gamma(2);
    
    translation = transl2(C_gamma');
    rotation = trot2(Theta_d);
    T = translation * rotation;
    
    x_w = x_d -a3*cos(Theta_d);
    y_w = y_d -a3*sin(Theta_d);
    
    Theta_2 = acos((x_w^2 + y_w^2 - a1^2 - a2^2) / 2*a1*a2);
    if Theta_2<0
        Theta_2 = Theta_2 * -1;
    end
    k1 = a1 + a2*cos(Theta_2);
    k2 = a2*sin(Theta_2);
    Theta_1 = atan2(y_w,x_w) - atan2(k2,k1);
    Theta_3 = Theta_d - Theta_1 - Theta_2;
    q = [ Theta_1 Theta_2 Theta_3];
    
	robot.plot(q);
    trplot2(T,'length',0.1);
end