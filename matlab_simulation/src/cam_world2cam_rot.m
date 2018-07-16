function x_projected = cam_world2cam_rot(cam, R)

    x_projected = inv(cam.Rc)*R;

end