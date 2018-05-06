%cleanup
close all;
clear all;
clc;

%load matlab libraries
matlab_repo = '/media/duartecdias/5A8CD5D48CD5AAB1/Dropbox/Public/Áreas/Trabalho/Investigação/Doutoramento/IST-EPFL/Tese/Implementação';
addpath(matlab_repo);
matlab_libraries = strcat(matlab_repo, '/matlab_libraries/CEMRA');
addpath(strcat(matlab_libraries, '/DataAnalysis'));

%main location
exp_repo = '/home/duartecdias/Repos/CEMRA/bags/bags-v3/';
setenv('EXP_REPO', exp_repo);

%so the version from matlab is not used (errors on webots and ros are generated otherwise)
setenv('LD_PRELOAD','/usr/lib/x86_64-linux-gnu/libstdc++.so.6:/usr/lib/x86_64-linux-gnu/libexpat.so');

%parameters
exp_number = 2;
mesh_repo = strcat(exp_repo, 'mesh/Exp', num2str(exp_number), '/');
setenv('MESH_REPO', mesh_repo);

%process bag
if exist(strcat(mesh_repo, '_robot2_cmd_vel_fixed.txt'), 'file') ~= 2
!roscore &
!roscore_pid=$!
!${EXP_REPO}extract_datasets.sh ${MESH_REPO}mesh.bag /robot2/cmd_vel_fixed /robot2/odom
!kill -SIGINT $roscore_pid
!mv _robot2_cmd_vel_fixed.txt ${MESH_REPO}
!mv _robot2_odom.txt ${MESH_REPO}
end

%load bag
files = [];
ind = 1;
s.name_i = strcat(mesh_repo, '_robot2_cmd_vel_fixed.txt'); s.name_o = strcat(mesh_repo, 'command.txt'); s.nheaders = 1; s.eliminations = [];  s.processed = 0; files = cat(1, files, s); fids.COMMAND = ind; ind = ind + 1;
s.name_i = strcat(mesh_repo, '_robot2_odom.txt'); s.name_o = strcat(mesh_repo, 'odometry.txt'); s.nheaders = 1; s.eliminations = [4,5];  s.processed = 0; files = cat(1, files, s); fids.ODOMETRY = ind; ind = ind + 1;
files = get_bag_file_names(mesh_repo, files);
command = load(files(fids.COMMAND).name_o);
odometry = load(files(fids.ODOMETRY).name_o);

%extract predicted commands
time_command = command(:, 1)/10^9;
time_command = time_command - time_command(1);
v = command(:, 2);
w = command(:, 7);

%extract odometry (ony the yaw angle is needed for ground robots)
time = odometry(:, 1)/10^9;
time = time - time(1);
x = odometry(:, 4:6);
q = odometry(:, 7:10); q1 = q(:,1); q2 = q(:,2); q3 = q(:,3); q4 = q(:,4);
theta = atan2(2*(q4.*q3+q1.*q2),(1-2*(q2.*q2+q3.*q3)));
r = [cos(theta'); sin(theta'); zeros(1, length(theta))];
uz = [zeros(1, length(theta)); zeros(1, length(theta)); ones(1, length(theta))];
rp = cross(uz, r);

%plot the raw measurements
figure(1);
h = subplot(1, 2, 1);
hold on;
plot(h, v, 'r');
plot(h, w, 'g');
h = subplot(1, 2, 2);
hold on;
plot(h, x(:,1), x(:,2), 'r');
line([x(:,1) (x(:,1) + 0.1*rp(1,:)')]', [x(:,2) (x(:,2) + 0.1*rp(2,:)')]', 'color', 'k');

%compute odometry speeds
dt = (time(10:10:end) - time(1:10:end-10));
dx_odometry = x(10:10:end, 1:2) - x(1:10:end-10, 1:2);
%pdx_odometry = cross(uz(:, 2:end)', [dx_odometry zeros(size(x, 1)-1, 1)]);
%pdx_odometry = pdx_odometry(:, 1:2);
n_odometry = sqrt(sum(dx_odometry.*dx_odometry, 2));
v_odometry = n_odometry./dt;
% n_odometry(n_odometry == 0) = 1;
%c_theta = sum(dx_odometry(2:end, :).*dx_odometry(1:end-1, :), 2)./(n_odometry(2:end).*n_odometry(1:end-1));
%s_theta = sum(dx_odometry(2:end, :).*pdx_odometry(1:end-1, :), 2)./(n_odometry(2:end).*n_odometry(1:end-1));
%theta_aux = atan2(s_theta, c_theta);
%w_odometry_aux = (theta_aux(2:end) - theta_aux(1:end-1))./(time(4:end) - time(3:end-1));
w_odometry = (theta(10:10:end) - theta(1:10:end-10))./dt;

%plot odometry speeds vs command speeds
figure(2);
hold on;
t = time(1:10:end-10);
plot(time_command, v, 'r');
plot(t, v_odometry, '.r');
plot(time_command, w, 'g');
plot(t, w_odometry, '.g');
% plot(time(4:end), w_odometry_aux, '.b');

%plot odometry time
figure(3);
plot(dt);

%compute odometry noise
figure(2);
pause;
[x y] = ginput(4);
v_odometry_l = v_odometry( logical((t > x(1)) .* (t < x(2))) );
w_odometry_l = w_odometry( logical((t > x(3)) .* (t < x(4))) );
w_odometry_l1 = w_odometry( logical((t > x(1)) .* (t < x(2))) );
q1 = std(v_odometry_l)/0.05
q2 = std(w_odometry_l)/0.3
bias = mean(w_odometry_l1)/0.05
err = std(w_odometry_l1)/0.05

%results:
%-command overshoot of 15% of the command signal
%-negative rotation bias of 0.02
%-noise std of 10% of the command signal
