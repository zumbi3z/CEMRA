function cam = cam_init(camera_file_name, Hz, noise)

    %load camera parameters
    cam = load(camera_file_name, 'cam');
    cam = cam.cam;
    cam.Hz = Hz;
    cam.noise = noise;
    
    %adding auxiliar variables
    cam.pixel_n = 0:cam.width*cam.height-1;
    cam.im = zeros(cam.width, cam.height);
    cam.acquired = 0;

end
