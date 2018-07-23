%Out = 1 means there is at least one collision with the links of the robot
%Boxes-links collision checker
function out = collision_checker(obs_poses,dims,arm_bloat_factor,link_points)
	dims = dims + arm_bloat_factor;
	for obj=1:size(obs_poses,1)
		x = obs_poses(obj,1);
		y = obs_poses(obj,2);
		z = obs_poses(obj,3);

		xsz = dims(obj,1);
		ysz = dims(obj,2);
		zsz = dims(obj,3);
		min_bound = [x - xsz/2,y - ysz/2,z - zsz/2];
		max_bound = [x + xsz/2,y + ysz/2,z + zsz/2];
		fprintf("minx: %.2f,maxx: %.2f,miny: %.2f,maxy: %.2f,minz: %.2f,maxz: %.2f\n",min_bound(1),max_bound(1),min_bound(2),max_bound(2),min_bound(3),max_bound(3));
		for link_=1:size(link_points,1)
			line_start = link_points(link_,:);
			Ox = line_start(1);
			Oy = line_start(2);
			Oz = line_start(3);
			
			if(Ox >= min_bound(1) && Ox<=max_bound(1) ) && (Oy >= min_bound(2) && Oy<=max_bound(2)) && (Oz >= min_bound(3) && Oz<=max_bound(3))
				out = 1;
				return;
			end
		end	

	end
	out = 0;
end
