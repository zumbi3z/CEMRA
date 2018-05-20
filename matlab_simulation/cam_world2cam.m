function x_projected = cam_world2cam(cam, x)

    x_projected = inv(cam.Rc)*(x - repmat(cam.tc, 1, size(x, 2)));

end