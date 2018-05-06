function [u, w] = log_map_SE3(chi)

    R = chi(1:3, 1:3);
    t = chi(1:3, 4);
    w = log_map_SO3(R);
    theta = sqrt(w'*w);
    w_hat = lie_algebra_so3(w);
    invV = eye(3) - 0.5*w_hat + (1/theta^2)*(1 - ((theta*sin(theta))/(2-2*cos(theta))))*w_hat*w_hat;
    u = invV*t;

end

function w = log_map_SO3(R)

    theta = acos( (R(1, 1) + R(2, 2) + R(3, 3) - 1)/2 );
    if theta == 0
        w = [0 0 0]';
    else
        w = (theta/(2*sin(theta)))*[R(3, 2) - R(2, 3);R(1, 3) - R(3, 1); R(2, 1) - R(1, 2)];
    end

end

function w_hat = lie_algebra_so3(w)

    w_hat = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];

end