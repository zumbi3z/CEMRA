function R = exponential_map_SO3(w)

    theta = sqrt(w'*w);
    w_hat = lie_algebra_so3(w);
    R = eye(3) + (sin(theta)/theta)*w_hat + ((1 - cos(theta))/theta^2)*w_hat*w_hat;
    
end

function w_hat = lie_algebra_so3(w)

    w_hat = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];

end
