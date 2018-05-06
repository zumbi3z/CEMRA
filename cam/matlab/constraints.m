function [c,ceq] = constraints(x)

    c = 0;
    ceq = [1 - norm(x(1:3)); 1 - norm(x(4:6)); 1 - norm(x(7:9))];
    
end