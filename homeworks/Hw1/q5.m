% Name: Tahsincan Köse
% ID: 2188423


function q5()    
clf;
figure(1); 
format compact 
% h(1) = axes('Position',[1 1 3 3]);
 vert = [1 1 -1; 
        -1 1 -1; 
        -1 1 1; 
        1 1 1; 
        -1 -1 1;
        1 -1 1; 
        1 -1 -1;
        -1 -1 -1];
 fac = [1 2 3 4; 
           4 3 5 6; 
           6 7 8 5; 
           1 2 8 7; 
           6 7 1 4; 
           2 3 5 8];  
 add = ones(size(vert,1),1);
 vert = [vert add];
 hold_vert = vert;
 angle_x = 2*pi;
 
 tw = Twist('R', [-1 0 1], [1,0,0],0.0);
 alpha = 0.2;
 while 1
    %T = trotx(alpha);
    T = tw.T(alpha);
    for i=1:8
        vert(i,:) = T * vert(i,:)';
    end
    vert = vert(:, 1:3);
    patch('Faces',fac,'Vertices',vert,'FaceColor','g','FaceAlpha',0.5);  % patch function
    grid on;
    plotvol([-5 5 -5 5 -5 5]);
    pause on;
    pause(0.25)
    clf;
    vert = hold_vert;
    alpha = alpha + 0.2;
    if alpha > 2*pi
        alpha = alpha - 2*pi;
    end
 end