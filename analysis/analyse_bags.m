%% Process bags from trajectory

%get repository path
[~, cmdout] = system('locate -r /CEMRA$');
splitted_string = strsplit(cmdout, '\n');
repo_path = char(splitted_string{1});
addpath(genpath(repo_path));

%so the version from matlab is not used (errors on webots and ros are generated otherwise)
setenv('LD_PRELOAD','/usr/lib/x86_64-linux-gnu/libstdc++.so.6:/usr/lib/x86_64-linux-gnu/libexpat.so');

%name of bag to process
bag_file = '2018-07-14-20-11-09.bag';
simulation = 0;

%process bags
!roscore &
experiment = experiment_create('trajectory', 3, 2, 0.5);
pause(7);
cd(strcat(repo_path, '/estimator_cam'));
if simulation
    !roslaunch estimator_cam node_multiple_robots_sim.launch > log_sim.txt &
else
    !roslaunch estimator_cam node_multiple_robots.launch > log.txt &
end
pause(5);
if simulation
    simulation_or_real = 'simulation';
else
    simulation_or_real = 'real';
end
setenv('BAG_FILE', strcat(repo_path, '/bags/', simulation_or_real, '/trajectory/', bag_file));
!rosbag play $BAG_FILE &
rosbag_file_name = strcat(repo_path, '/bags/', simulation_or_real, '/data/', bag_file(1:end-5), '_',num2str(experiment.test), '_', num2str(experiment.n), '_', num2str(experiment.cam_noise), '_.bag');
setenv('ROSBAG_FILE_NAME', rosbag_file_name);
if experiment.n == 1
    topic_names = [];
    topic_names = char(strcat(topic_names, {sprintf(' /robot2/cam_pose')}));
    topic_names = char(strcat(topic_names, {sprintf(' /robot2/cam_pose_GT')}));
    topic_names = char(strcat(topic_names, {sprintf(' /robot2/estimated_pose')}));
    topic_names = char(strcat(topic_names, {sprintf(' /robot2/estimated_pose_GT')}));
else
    topic_names = [];
    for k = 1:experiment.n
        topic_names = char(strcat(topic_names, {sprintf(' /robot%d/cam_pose', k)}));
        topic_names = char(strcat(topic_names, {sprintf(' /robot%d/cam_pose_GT', k)}));
        topic_names = char(strcat(topic_names, {sprintf(' /robot%d/estimated_pose', k)}));
        topic_names = char(strcat(topic_names, {sprintf(' /robot%d/estimated_pose_GT', k)}));
    end
end
setenv('TOPIC_NAMES', topic_names);
[~, cmdout] = system(char(strcat({'rosbag info -y -k duration '}, repo_path, '/bags/', simulation_or_real, '/trajectory/', bag_file)));
rosbag_time = str2num(cmdout) - 0.8;
setenv('ROSBAG_TIME', num2str(rosbag_time));
!rosbag record -O $ROSBAG_FILE_NAME --duration=$ROSBAG_TIME $TOPIC_NAMES &
pause(rosbag_time + 10);

%cleaning ros environment
!killall rosbag
pause(3);
!killall -SIGINT roslaunch
pause(3);
!killall -SIGINT roscore
pause(3);

%% Process bags from trajectory

%get repository path
[~, cmdout] = system('locate -r /CEMRA$');
splitted_string = strsplit(cmdout, '\n');
repo_path = char(splitted_string{1});
addpath(genpath(repo_path));

%name of bag to process
bag_file = '2018-07-14-20-11-0';

%process bags
!roscore &
experiment = experiment_create('trajectory', 3, 2, 0.5);
experiment_process(experiment, [0 0 0 0], repo_path, 0, 'process', bag_file);
!killall -SIGINT roscore