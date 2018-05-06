function w = log_map_SO3(R)

    theta = acos( (R(1, 1) + R(2, 2) + R(3, 3) - 1)/2 );
    if theta == 0
        w = [0 0 0]';
    else
        w = (theta/(2*sin(theta)))*[R(3, 2) - R(2, 3);R(1, 3) - R(3, 1); R(2, 1) - R(1, 2)];
    end

end
