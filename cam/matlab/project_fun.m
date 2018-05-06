
function y = project_fun(x)

    global x_beacon;
    global u;
    global v;
    global K;
    
    a = inv(K) * [u;v;ones(1, size(u, 2))];
    ul = a(2, :);
    vl = a(3, :);
     
    A = [-ul(1) * (x_beacon(:, 1)') x_beacon(:, 1)' zeros(1, 3)  [-ul(1) 1 0];
         -vl(1) * (x_beacon(:, 1)') zeros(1, 3) x_beacon(:, 1)'  [-vl(1) 0 1];
         -ul(2) * (x_beacon(:, 2)') x_beacon(:, 2)' zeros(1, 3)  [-ul(2) 1 0];
         -vl(2) * (x_beacon(:, 2)') zeros(1, 3) x_beacon(:, 2)'  [-vl(2) 0 1];
         -ul(3) * (x_beacon(:, 3)') x_beacon(:, 3)' zeros(1, 3)  [-ul(3) 1 0];
         -vl(3) * (x_beacon(:, 3)') zeros(1, 3) x_beacon(:, 3)'  [-vl(3) 0 1];
         -ul(4) * (x_beacon(:, 4)') x_beacon(:, 4)' zeros(1, 3)  [-ul(4) 1 0];
         -vl(4) * (x_beacon(:, 4)') zeros(1, 3) x_beacon(:, 4)'  [-vl(4) 0 1];];
     
    y = (A*x)' * A*x;
%     T = [reshape(x(1:9), 3, 3)' x(end-2:end)]
    
end
