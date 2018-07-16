function marker = marker_move(marker, t)

    %check if it is time to control
    if (t - marker.t_control) > 1/marker.Hz_c
        
        %compute time delta
        dt = t - marker.t_control;
        
        %update control time
        marker.t_control = t;
        
        %marker controller
        marker.w(3) = marker.w(3) + dt*marker.w_speed;
        marker.R = exponential_map_SO3(marker.w);
        
    end

end
