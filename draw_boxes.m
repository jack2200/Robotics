function draw_boxes(obs_poses,dim,arm_bloat_factor) %arm_bloat_factor is a fixed variable. Should be placed to the function body as a constant variable.
	%plotvol([-1 1 -1 1 -1 1]);
	fac = [1 2 3 4; 
		   4 3 5 6; 
		   6 7 8 5; 
		   1 2 8 7; 
		   6 7 1 4; 
		   2 3 5 8];
	dim = dim + arm_bloat_factor; 
	for i=1:size(obs_poses,1)
		 
		x = obs_poses(i,1);
		y = obs_poses(i,2);
		z = obs_poses(i,3);
		
		xsz = dim(i,1);
		ysz = dim(i,2);
		zsz = dim(i,3);
		vert = [x + xsz/2,y + ysz/2,z - zsz/2;
		        x - xsz/2,y + ysz/2,z - zsz/2;
			x - xsz/2,y + ysz/2,z + zsz/2;
			x + xsz/2,y + ysz/2,z + zsz/2;
			x - xsz/2,y - ysz/2,z + zsz/2;
			x + xsz/2,y - ysz/2,z + zsz/2;
			x + xsz/2,y - ysz/2,z - zsz/2;
			x - xsz/2,y - ysz/2,z - zsz/2];
		
		patch('Faces',fac,'Vertices',vert,'FaceColor','g','FaceAlpha',0.5);  % patch function
	    	grid on;
		hold on;
	end






end
