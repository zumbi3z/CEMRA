function marker = marker_init(x_beacon_loc, x_beacon_id, Hz_f, Hz_c, l_speed, w_speed)

    marker.x_beacon_loc = x_beacon_loc;
    marker.x_beacon_id = x_beacon_id;
    marker.id_beacons_flashing = 0;
    marker.Hz_f = Hz_f;
    marker.Hz_c = Hz_c;
    marker.l_speed = l_speed;
    marker.w_speed = w_speed;
    marker.x = [];
    marker.w = [];
    marker.R = [];

end
