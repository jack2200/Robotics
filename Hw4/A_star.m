% Very similar to PGraphs A* implementation. 
% Also used http://mat.uab.cat/~alseda/MasterOpt/AStar-Algorithm.pdf as the pseudocode.

% Tahsincan Köse
% 2188423

function path = A_star(G,v_start,v_goal)
    closedlist = [];
    openlist = [v_start];
    parents = [];
    
    g_score(v_start) = 0;
    heuristics(v_start) = G.distance(v_start,v_goal);
    f_score(v_start) = g_score(v_start) + heuristics(v_start);
    
    while ~isempty(openlist)
        [~,ind] = min(f_score(openlist));
        current = openlist(ind);
        
        if current == v_goal
            path = [];
            p = v_goal;
            while true
                path = [p path];
                p = parents(p);
                if p==0
                    break;
                end
            end
            return;
        end
        
        openlist = setdiff(openlist,current); % Remove from open list and add to closed list.
        closedlist = union(closedlist,current);
        
        for n=G.neighbours(current)
            if ismember(n,closedlist) % If it is already evaluated, do not process it again.
                continue;
            end
            possible_neighbour_score = g_score(current) + G.distance(current,n);
            if ismember(n,openlist) %Already in the openlist. Check for a shorter path.
                if possible_neighbour_score < g_score(n)
                    update = true;
                else
                    update = false;
                end
            else
                openlist = union(openlist,n);
                heuristics(n) = G.distance(n,v_goal);
                update = true;
            end
            
            if update
                parents(n) = current;
                g_score(n) = possible_neighbour_score;
                f_score(n) = g_score(n) + heuristics(n);
            end
        end
    end
    path = []; % If no path is found return empty path.

end
