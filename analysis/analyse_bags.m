%% Process bags from trajectory

%get repository path
[~, cmdout] = system('locate -r /CEMRA$');
splitted_string = strsplit(cmdout, '\n');
repo_path = char(splitted_string{1});
addpath(genpath(repo_path));

%name of bag to process
bag_file = '2018-05-20-20-35-09_.bag';

%process bags
!roscore &
experiment = experiment_create('trajectory', 3, 3, 0.5);
pause(7);
cd(strcat(repo_path, '/estimator_cam'));
!roslaunch estimator_cam node_multiple_robots_sim.launch > log.txt &
pause(5);
setenv('BAG_FILE', strcat(repo_path, '/bags/simulation/trajectory/', bag_file));
!rosbag play $BAG_FILE &
rosbag_file_name = strcat(repo_path, '/bags/simulation/data/', bag_file(1:end-5), '_',num2str(experiment.test), '_', num2str(experiment.n), '_', num2str(experiment.cam_noise), '_.bag');
setenv('ROSBAG_FILE_NAME', rosbag_file_name);
if experiment.n == 1
    topic_names = '/robot2/cam_pose';
else
    topic_names = [];
    for k = 1:experiment.n
        topic_names = char(strcat(topic_name, {sprintf(' /robot%d/cam_pose', k)}));
    end
end
setenv('TOPIC_NAMES', topic_names);
!rosbag record -O $ROSBAG_FILE_NAME --duration=40 $TOPIC_NAMES &
pause(65);
!killall -SIGINT roscore

%% Process bags from trajectory

%get repository path
[~, cmdout] = system('locate -r /CEMRA$');
splitted_string = strsplit(cmdout, '\n');
repo_path = char(splitted_string{1});
addpath(genpath(repo_path));

%name of bag to process
bag_file = '2018-05-20-20-35-09';

%process bags
!roscore &
experiment = experiment_create('trajectory', 3, 3, 0.5);
experiment_process(experiment, [0 0 0 0], repo_path, 1, 'process', bag_file);
!killall -SIGINT roscore