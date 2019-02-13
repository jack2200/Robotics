% Tahsincan Köse
% 2188423


function path = PRM(q0,qf,obstacles)
    % In order to construct a road map, a reasonable space is needed.
    % Assuming this is a 40x40 grid will satisfy that necessity.
    xmax = 20;xmin=-20;
    ymax = 20;ymin=-20;
    % Also restricting theta values into [0,90] range will provide a
    % consistent configuration path. And this assumption further simplifies
    % the problem in hand. Note that, qf must fit into this assumption.
    
    RoadMap = PGraph(4); % Make PGraph(6) for our case.
    v_init = RoadMap.add_node(q0); % Add start node to graph.
    v_end = RoadMap.add_node(qf); % Add end node to graph.
    number_of_samples =400;
    threshold = 2.5 ;
    for i=1:number_of_samples
        % Need to pick a point not in any one of the obstacles
        while true
            x = randi([xmin xmax]);
            y = randi([ymin ymax]);
            q1 = randi([0 90]);
            q1 = q1 * pi / 180;
            q2 = randi([0 90]);
            q2 = q2 * pi / 180;
            not_valid = false;
            for j=1:size(obstacles,1)
                if hw4_script3([x y q1 q2],0.2,1,1,[obstacles(j,1),obstacles(j,2)],obstacles(j,3))==1
                    not_valid = true;
                    break;
                end
            end
            if not_valid==false
                break
            end
        end
        % Successfuly chose a valid point in the configuration space.
        new_node = [x y q1 q2];
        nid = RoadMap.add_node(new_node);
        %fprintf("Inserted %d %d %.2f %.2f to graph\n",x,y,q1,q2);
        [distances,vertices] = RoadMap.distances(new_node); 
        
        % This is a hugely useful utility function that automatically sorts
        % nodes in increasing order with respect to their distances to newly sampled point.
        
        for i=1:size(distances,2)
            q = RoadMap.coord(vertices(i))';
            if q == new_node %Itself has the lowest distance trivially
                continue;
            end
            if distances(i) > threshold % Since it is sorted, latter vertices cannot have a lower distance to
                                       % newly sampled point.
               %fprintf("Vertex %d exceeds threshold with distance %.2f\n",vertices(i),distances(i));
               break;
            end
            if collision_checker(new_node,RoadMap.coord(vertices(i)),obstacles)== 1
               eid = RoadMap.add_edge(vertices(i),nid);
               RoadMap.setcost(eid,distances(i));
            end
        end
    end
    RoadMap.display();
    clf;
    figure(1);
    hold on;
    for i=1:size(obstacles,1)
        x = obstacles(i,1);
        y = obstacles(i,2);
        r = obstacles(i,3);
        th = 0:pi/50:2*pi;
        xunit = r * cos(th) + x;
        yunit = r * sin(th) + y;
        h = plot(xunit, yunit);
    end
    plotvol([-20 20 -20 20]);
    RoadMap.plot('NodeSize',2.5);
    
    %path_indices = RoadMap.Astar(v_init,v_end); PGraph way
	path_indices = A_star(RoadMap,v_init,v_end); % My way
    RoadMap.highlight_node(path_indices,'NodeSize',7);
    path = RoadMap.coord(path_indices)';
    %[path,cost] = dijkstra(adj_matrix,q_init,q_end);
end
