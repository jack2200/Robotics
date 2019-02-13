% Tahsincan Köse
% 2188423


function out = hw4_script2_mat(p1,p2,r1,r2)
    if(p1==p2)%Both circles have the same center.Check for radiuses.
        if r1==r2
            out = 1;
        else
            out = 0;
        end
    else
        if p1(1) == p2(1) %Compute with y, since x's are equal.
            d = (r2^2 - r1^2 + p1(1)^2 + p1(2)^2 - p2(1)^2 - p2(2)^2) / (2*p1(2) - 2*p2(2));
            e = 0; % Because upper part is 2*(p1(1) - p2(1))
            
            a = 1;
            b = -2*p1(1);
            c = d^2 - 2*p1(2)*d + dot(p1,p1) - r1^2;
            
            discriminant = b^2 - 4*a*c;
            if discriminant < 0
                out = 0;
            else
                out = 1;
            end
        elseif p1(2) == p2(2) %Compute with x, since y's are equal.
            d = (r2^2 - r1^2 + p1(1)^2 + p1(2)^2 - p2(1)^2 - p2(2)^2) / (2*p1(1) - 2*p2(1));
            e = 0; % Because upper part is 2*(p1(2) - p2(2))
             
            a = 1;
            b = -2*p1(2);
            c = d^2 - 2*p1(1)*d + dot(p1,p1) - r1^2;
            %fprintf("d: %.2f, b: %.2f, c: %.2f\n",d,b,c);
            discriminant = b^2 - 4*a*c;
            %fprintf("Delta: %.2f\n",discriminant);
            if discriminant < 0
                out = 0;
            else
                out = 1;
            end
        else % Both are different, just choose x to compute as a convenience.
            d = (r2^2 - r1^2 + p1(1)^2 + p1(2)^2 - p2(1)^2 - p2(2)^2) / (2*p1(1) - 2*p2(1));
            e = (p1(2) - p2(2)) / (p1(1) - p2(1));
             
            a = e^2 + 1;
            b = 2*p1(1)*e - 2*d*e - 2*p1(2);
            c = d^2 - 2*p1(1)*d + dot(p1,p1) - r1^2;
            %fprintf("d: %.2f, e: %.2f, a: %.2f, b: %.2f, c: %.2f\n",d,e,a,b,c);
            discriminant = b^2 - 4*a*c;
            %fprintf("Delta: %.2f\n",discriminant);
            if discriminant < 0
                out = 0;
            else
                out = 1;
            end 
        end
end
