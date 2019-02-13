% Tahsincan Köse
% 2188423

function out = collision_checker(v1,v2,obstacles)
    % Create a 50 member configuration array between v1 and v2.
    freq = 50;
    samples = [linspace(v1(1),v2(1),freq);linspace(v1(2),v2(2),freq);linspace(v1(3),v2(3),freq);linspace(v1(4),v2(4),freq)];
    %fprintf("%d,%d\n",size(obstacles,1),size(obstacles,2));
    for i=1:size(samples,2)
        for j=1:size(obstacles,1)
            if hw4_script3([samples(1,i) samples(2,i) samples(3,i) samples(4,i)],0.2,1,1,[obstacles(j,1),obstacles(j,2)],obstacles(j,3))==1
                
              out = 0;
              return;
            end
        end
    end
    out = 1;
end
