%% Experiment

%cleanup
close all;
clear;
clc;

%get repository path
[~, cmdout] = system('locate -r /CEMRA$');
splitted_string = strsplit(cmdout, '\n');
repo_path = char(splitted_string{1});
addpath(genpath(repo_path));

%so the version from matlab is not used (errors on webots and ros are generated otherwise)
setenv('LD_PRELOAD','/usr/lib/x86_64-linux-gnu/libstdc++.so.6:/usr/lib/x86_64-linux-gnu/libexpat.so');

%camera parameters
cam_noise = 0.5;
cam = cam_init('camera.mat', 10, cam_noise); %adding the frequnecy and the noise of the camera

%create some markers (which are simulating the robots
markers = [];
markers = cat(1, markers, marker_init([0.20 0.0 0.0; 0.0 0.13 0.0; 0.10 0.10 0.00; 0.0 -0.00 0.00;]', [0.0353 0.0353 0.00]', 5, 10, 0.2, 0.2));
markers = cat(1, markers, marker_init([0.20 0.0 0.0; 0.0 0.13 0.0; 0.10 0.10 0.00; 0.0 -0.00 0.00;]', [0.0353 0.0353 0.00]', 2.5, 10, 0.2, 0.2));
markers = cat(1, markers, marker_init([0.20 0.0 0.0; 0.0 0.13 0.0; 0.10 0.10 0.00; 0.0 -0.00 0.00;]', [0.0353 0.0353 0.00]', 1.8, 10, 0.2, 0.2));

%set marker position
% for k = 1:length(markers)
%     markers(k) = marker_set_trajectory(markers(k), [1.5 1.8 0.2]', [0 0 pi/3]');
% end
markers(1) = marker_set_trajectory(markers(1), [0 1.8 0.2]', [0 0 pi/3]');
markers(2) = marker_set_trajectory(markers(2), [1.5 0 0.2]', [0 0 pi/3]');
markers(3) = marker_set_trajectory(markers(3), [1.5 1.8 0.2]', [0 0 pi/3]');

%set experiment
n_robots = length(markers);
experiment = experiment_create('trajectory', 3, n_robots, cam_noise);

%experiment must be valid before it can be started
status = experiment_evaluate(experiment, repo_path);
if status > 0
    
    %get current time
    tstr = datestr(now,'yyyy-mm-dd-HH-MM-SS');

    %initialize ros
    !roscore &
    pause(7);
    rosinit
    pause(3);

    %record a rosbag for a period of time
    rosbag_file_name = strcat(repo_path, '/bags/simulation/trajectory/', tstr, '_.bag');
    setenv('ROSBAG_FILE_NAME', rosbag_file_name);
    topic_names = strcat('/camera1/image_raw /clock');
    setenv('TOPIC_NAMES', topic_names);
    !rosbag record -O $ROSBAG_FILE_NAME --duration=60 $TOPIC_NAMES &

    %create ros message structure
    pub = rospublisher('/camera1/image_raw', 'sensor_msgs/Image');
    pub_c = rospublisher('/clock', 'rosgraph_msgs/Clock');
    msg = rosmessage(pub);
    msg.Height = cam.height;
    msg.Width = cam.width;
    msg.Step = cam.width * 3;
    msg.Encoding = 'bgr8';
    msg_c = rosmessage(pub_c);

    %initializations
    %t = rostime('now');
    %t = t.Sec+t.Nsec/10^9;
    t = 0;
    cam.t_acquisition = t;
    for k = 1:length(markers)
        markers(k).t_control = t;
        markers(k).t_flash = t;
    end
    cam.data = 10*ones(cam.width * cam.height * 3, 1); %generate empty image
    cam.indsT = zeros(1, cam.width * cam.height);

    %run simulation
    while(1)

        %update time
        t = t + 0.01;

        %move markers
        for k = 1:length(markers)
            markers(k) = marker_move(markers(k), t);
        end
        
        %flash id beacons of the marker
        for k = 1:length(markers)
            markers(k) = marker_flash_id_beacons(markers(k), t);
        end
        
        %generate images to ros topic
        cam.data(1:end) = 0; %generate empty image
        cam = cam_generate_image(cam, markers, t);
        msg.Data = cam.data;
        if cam.acquired
            msg.Header.Seq = msg.Header.Seq + 1;
            msg.Header.Stamp.Sec = floor(t);
            nsecs = int32((t - floor(t))*10^9);
            if nsecs >= 1e+09, nsecs = 0; end
            msg.Header.Stamp.Nsec = nsecs;
            send(pub,msg);
        end
        
        %send clock
        msg_c.Clock_.Sec = floor(t);
        nsecs = int32((t - floor(t))*10^9);
        if nsecs >= 1e+09, nsecs = 0; end
        msg_c.Clock_.Nsec = nsecs;
        send(pub_c, msg_c);

    end

end

%% Finish experiment

'cleaning'
!killall -SIGINT rosbag
pause(3);
!killall -SIGINT roslaunch
pause(3);
!killall -SIGINT roscore
pause(3);
rosshutdown