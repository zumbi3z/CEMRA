global x_beacon;
global u;
global v;
global K;

x_beacon = [0.20,0.0,0.0; 0.0,0.13,0.0; 0.10,0.10,0.00; 0.0,-0.02,0.00;]';

x_beacon_l = t_marker + R_marker * x_beacon;
x_beacon_p = K * x_beacon_l;
x_beacon_p = x_beacon_p ./ repmat(x_beacon_p(3,:), 3, 1);
x_beacon_p(1:2, :) = x_beacon_p(1:2, :) + 3*randn(2, size(x_beacon_p, 2));
u = x_beacon_p(1, :);
v = x_beacon_p(2, :);

x0 = [reshape((R_marker + 0.1*randn(3))', 9, 1);t_marker + 0.2*randn(3,1)];
options = optimoptions(@fmincon,'Algorithm','interior-point','MaxIterations',10);
tic
x = fmincon(@project_fun,x0,[],[],[],[],[],[],@constraints, options);
toc
R_marker_opt = reshape(x(1:9), 3, 3)';
t_marker_opt = x(end-2:end);

