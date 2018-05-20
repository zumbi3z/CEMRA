function marker = marker_set_trajectory(marker, x, w)

    marker.x = x;
    marker.w = w;
    marker.R = exponential_map_SO3(w);

end
