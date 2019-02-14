% Name: Tahsincan Kose
% ID: 2188423
% Usage: call with an integer parameter

function weather_sequence_str = generate_weather(n)    
    TransitionTable = [0.8 0.2 0; 0.4 0.4 0.2; 0.2 0.6 0.2];
    
    k=randi(99);
    if k<34
        state = 1;
    elseif k<67
        state = 2;
    else
        state = 3;
    end
    % State 1-2-3 : Sunny-Cloudy-Rainy
    weather_sequence = zeros(n,1);
    weather_sequence(1) = state;
    for i=2:n
        k = randi(99);
        new_state = decide(TransitionTable(state,:),k);
        weather_sequence(i) = new_state;
        state = new_state;
    end
    weather_sequence_str = [];
    for i=1:n
        if weather_sequence(i) == 1
            weather_sequence_str = [weather_sequence_str "sunny"];
        elseif weather_sequence(i) == 2
            weather_sequence_str = [weather_sequence_str "cloudy"];
        else
            weather_sequence_str = [weather_sequence_str "rainy"];
        end
    end
end
 
function s = decide(v,randnum)
   if randnum<v(1)*100
       s = 1;
   elseif randnum<(v(1)+v(2))*100
       s = 2;
   else
       s = 3;
   end
end