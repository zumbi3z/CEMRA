function [chi, V, invV] = exponential_map_SE3(u, w)

    theta = sqrt(w'*w);
    w_hat = lie_algebra_so3(w);
    R = eye(3) + (sin(theta)/theta)*w_hat + ((1 - cos(theta))/theta^2)*w_hat*w_hat;
    V = eye(3) + ((1 - cos(theta))/theta^2)*w_hat + ((1 - (sin(theta)/theta))/theta^2)*w_hat*w_hat;
    invV = eye(3) - 0.5*w_hat + (1/theta^2)*(1 - ((theta*sin(theta))/(2-2*cos(theta))))*w_hat*w_hat;
    chi = [R V*u;zeros(1, 3) 1];
%     chi = eye(4) + e_hat + ((1 - cos(theta))/theta^2)*e_hat*e_hat + ((1 - (sin(theta)/theta))/theta^2)*e_hat*e_hat*e_hat;

end

function w_hat = lie_algebra_so3(w)

    w_hat = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];

end

% function e_hat = lie_algebra_se3(x, w)
% 
%     w_hat = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
%     e_hat = [w_hat x;zeros(1, 3) 0];
% 
% end
