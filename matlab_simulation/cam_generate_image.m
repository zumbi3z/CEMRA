function cam = cam_generate_image(cam, markers, t)

    %check if it is time to generate image
    if (t - cam.t_acquisition) >= 1/cam.Hz
        
        %update image acquisition time
        cam.t_acquisition = t;
        cam.acquired = 1;

        %insert markers in the image
        for k = 1:length(markers)
            
            %progect marker localization beacons
            cam = cam_project_beacons(cam, markers(k).x, markers(k).R, markers(k).x_beacon_loc, 1);

            %check if it is time to project id beacons
            if markers(k).id_beacons_flashing
               cam = cam_project_beacons(cam, markers(k).x, markers(k).R, markers(k).x_beacon_id, 3);
            end

        end
        
        

    else
        cam.acquired = 0;
    end

end

function cam = cam_project_beacons(cam, x, R, x_beacon, color)

    %generate marker pixels
    x_beacon_g = x + R * x_beacon;
    x_beacon_l = inv(cam.Rc)*(x_beacon_g - cam.tc);
    x_beacon_p = cam.K * x_beacon_l;
    x_beacon_p = x_beacon_p ./ repmat(x_beacon_p(3,:), 3, 1);
    x_beacon_p(1:2,:) = x_beacon_p(1:2,:) + cam.noise*randn(2, size(x_beacon_p, 2)); %generate noise
    cam.indsT(1:end) = 0;
    for l = 1:size(x_beacon_p, 2)
        %inds = ((cam.pixel_n - cam.width *floor(cam.pixel_n/cam.width)) - x_beacon_p(1,l)).^2 + ...
        %(floor(cam.pixel_n/cam.width) - x_beacon_p(2,l)).^2  < 15;
        cam.indsT( ((cam.pixel_n - cam.width *floor(cam.pixel_n/cam.width)) - x_beacon_p(1,l)).^2 + ...
        (floor(cam.pixel_n/cam.width) - x_beacon_p(2,l)).^2  < 15 ) = 1; %= logical(cam.indsT + inds);
        %cam.indsT(cam.indsT > 1) = 1;
    end
    
    %place marker pixels on the image
    cam.data(cam.pixel_n(logical(cam.indsT))*3 + color) = uint8(255);

end