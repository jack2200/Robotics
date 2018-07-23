% Tahsincan Köse
% 2188423

function out = hw4_script1(p1,p2,pc,rc)
    % p1 : start point
    % p2 : end point
    % pc : circle center
    % rc : circle radius

    d = p2 - p1; % direction vector
    v = p1 - pc; % vector from circle center to the line segment start.
                 % it is defined to ease the computations latter in the
                 % solution.

    % Please refer to the report for step-by-step calculation of the below
    % formula.

    a = dot(d,d);
    b = 2 * dot(d,v);
    c = dot(v,v) - rc^2;

    discriminant = b^2 - 4*a*c;
    if discriminant < 0
        out = 0;
    else
        t1 = (-b-sqrt(discriminant))/(2*a);
        t2 = (-b+sqrt(discriminant))/(2*a);
        % Three cases of miss, even when discriminant > 0, because this is not a line, instead line segment.
        % If both t1 and t2 > 1, then the line segment falls too short to the
        % circle and cannot intersect with it. If both < 0, then it overshoots
        % the circle from the beginning, that is started to late.
        % The last case is an extreme one, which the line segment is totally
        % inside of the circle.
        
        if (t1>1 && t2>1) || (t1<0 && t2<0) || (t1<0 && t2>1)
            out = 0;
        else
            out = 1;
        end
    end
end
