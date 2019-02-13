% Tahsincan Kose
% 2188423


function [Q, t] =  hw3_script2(r,c,n,delta)
    import ETS2.*
    Q = [];
    t = [];
    xy = [];
    pause on;
    
    d2 = 1;
    % Construct the robot
    E = Tx('d1') * Rz('q2')*Tx(d2);
    E(1).qlim = [-5 5];
    % For prismatic joints, need to specify qlimit
    q_current = [50 0]';
    for i=1:1:n
        Q = [Q, q_current];
        t = [t, delta*i];
        clf;
       % fprintf("q_current: %.2f,%.2f\n",q_current(1),q_current(2));
        figure(1);
        J = [ 1 -d2*sin(q_current(2)*pi/180); 0 d2*cos(q_current(2)*pi/180)]; %Jacobian
        invJ = inv(J);
        QT =Q';
        current_pose = [QT(i,1) + d2*cos(QT(i,2)*pi/180) d2*sin(QT(i,2)*pi/180)];
        fprintf("current_pose: %.2f,%.2f\n",current_pose(1),current_pose(2));
        
        xy = [xy; current_pose];
        sphere_normal = (current_pose - c) * r/norm(current_pose-c);
        v = [-1*sphere_normal(2), sphere_normal(1)];
        fprintf("v: %.2f,%.2f\n",v(1),v(2));
        q_vel_required = invJ * v';
       
        fprintf("q_current: %.2f,%.2f\n",q_current(1),q_current(2));
        E.plot(q_current','deg','workspace',[0 100 -15 15 0 5]);
       
        pause(0.2);
        q_next = q_current + (delta*q_vel_required);
        q_current = q_next;
    end
    % First plot the motion of first two joints wrt time.
    figure(2);
    mplot(t',Q','label',["q1","q2"]);
    
    % Then plot the end-effector position as a function of time.
   
    
    figure(3);
    mplot(t',xy,'label',["x","y"]);
end