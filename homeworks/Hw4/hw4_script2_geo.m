% Tahsincan Köse
% 2188423


function out = hw4_script2_geo(p1,p2,r1,r2)
    if(p1==p2)%Both circles have the same center.Check for radiuses.
        if r1==r2
            out = 1;
        else
            out = 0;
        end
    else
        dist = norm(p1-p2);
        if(dist>r1+r2)
            out = 0;
        else
            out = 1;
        end
    end
end
