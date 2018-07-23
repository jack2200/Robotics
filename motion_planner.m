mdl_ur5
boxes = [
    0.7750    0.3500    0.7500
    0.7800         0    0.7500
    0.7750   -0.3500    0.7500
   -0.7750   -0.3500    0.7500
   -0.7750    0.3500    0.7500
   -0.7800         0    0.7500];

table = [0.775 0 0.725];
table_size = [0.8 1.3 0.02];
links = [
        0.00,0.00,1.12
0.00,0.00,1.21
0.08,-0.11,1.21
0.27,0.18,0.93
0.49,0.34,0.65
0.54,0.27,0.65
0.60,0.31,0.72
0.82,0.00,1.11];
T = transl(links(8,:));
q = ur5.ikine(T)
if size(q,2)~=6
	return;
end

box_sizes = [0.125 0.125 0.05;0.125 0.125 0.05;0.125 0.125 0.05;0.125 0.125 0.05;0.125 0.125 0.05;0.125 0.125 0.05];
diameter = 0.12;
%q = [-3.01 2.49 0.84 -1.76 1.57 -1.44]

ur5.plot(q);
plotvol([-2 2 -2 2 0 4]);
hold on;
draw_boxes(table,table_size,diameter);
not_valid = collision_checker(table,table_size,diameter,links);
if ~not_valid
	[T,ALL] = ur5.fkine(q)
	
	
end

