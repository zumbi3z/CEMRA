function y = project_beacon(x, p)
    
    y = [];
    for l = 1:size(p.x_b_jm, 2)
        x_b_jm_C = inv(p.R_Ci) * (x(1:3) + exponential_map_SO3(x(4:6))*p.x_b_jm(:, l));
        z = (1/x_b_jm_C(1)) * p.K * x_b_jm_C;
        y = cat(1, y, z + x(7 + 2*(l-1):8 + 2*(l-1)));
    end

end