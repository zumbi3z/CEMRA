function marker = marker_flash_id_beacons(marker, t)

    %check if it is time to flash
    if (t - marker.t_flash) >= 1/(2*marker.Hz_f)
        marker.id_beacons_flashing = 1;
    else
        marker.id_beacons_flashing = 0;
    end
    if (t - marker.t_flash) >= 1/(marker.Hz_f)
        marker.t_flash = t;
    end

end

