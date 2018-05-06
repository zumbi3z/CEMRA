%% Create a camera

%cleanup
close all;
clear;
clc;

%empty camera
cam = [];

%camera intrinsic and extrinsic parameters
cam.width = 688; cam.height = 504;
cam.fx = 359; cam.fy = 350; cam.cx = cam.width/2; cam.cy = cam.height/2; %no distortion
cam.K = [cam.cx -cam.fx 0;cam.cy 0 -cam.fy;1 0 0];
cam.tc = [1.5 3 2]';
yaw = -90; roll = 45;
Ryaw = [cosd(yaw) -sind(yaw) 0; sind(yaw) cosd(yaw) 0; 0 0 1];
Rroll = [1 0 0; 0 cosd(roll) -sind(roll); 0 sind(roll) cosd(roll);]; 
cam.Rc = Rroll * Ryaw;

%% Save camera

%main location
exp_repo = '/home/duartecdias/Repos/CEMRA/cam/analysis/bags-v3/';

%save
save(strcat(exp_repo, 'camera.mat'), 'cam');

%% Create marker geometry

%cleanup
close all;
clear;
clc;

%marker geometry
%x_beacon = [0.0212,0.0212,0.075; 0.0,0.20,0.075; 0.20,0.0,0.075; 0.0,0.0,0.21;]'; %new one
% x_beacon = [0.22,0.0,0.0; 0.0,0.22,0.0; 0.08,0.08,0.00; 0.0,0.0,0.25;]'; %old one
x_beacon = [0.20,0.0,0.0; 0.0,0.13,0.0; 0.10,0.10,0.00; 0.0,-0.02,0.00;]'; %testing ones
% x_beacon = [0.00,-0.10,0.00; 0.0,0.15,0.00; 0.0,0.0,0.05; 0.20,0.0,0.00]';

%% Simulate camera in ROS with a theoretical strucutre, so it can be detected by the software

%prepare simulation
close all;
clear;
clc;

%main location
exp_repo = '/home/duartecdias/Repos/CEMRA/cam/analysis/bags-v3/';

%load camera parameters
cam = load(strcat(exp_repo, 'camera.mat'), 'cam');
cam = cam.cam;

%marker position
t_marker = [1.5 1.8 0.2]';
yaw_marker = pi/3;
Ryaw = [cos(yaw_marker) -sin(yaw_marker) 0; sin(yaw_marker) cos(yaw_marker) 0; 0 0 1];
R_marker = Ryaw;
vyaw_marker = 0.4189;

%auxiliary variables
pixel_n = 0:cam.width*cam.height-1;
im = zeros(cam.width, cam.height);

%connect to ROS
%rosinit;

%create message structure
pub = rospublisher('/camera1/image_raw');
msg = rosmessage(pub);
msg.Height = cam.height;
msg.Width = cam.width;
msg.Step = cam.width * 3;
msg.Encoding = 'bgr8';

%run publisher
time = rostime('now');
Hz = 10;
h = [];
figure(1); hold on; axis([0 704 0 472]);
while(1)
    
    %check if it is time to publish image
    time1 = rostime('now');
    dt = (time1.Sec+time1.Nsec/10^9 - (time.Sec+time.Nsec/10^9) );
    if dt > 1/Hz
        
        %delete debug figure
        delete(h);
        
        %reset time
        time = rostime('now');
        
        %move marker as proper
        yaw_marker = yaw_marker + dt*vyaw_marker
        Ryaw = [cos(yaw_marker) -sin(yaw_marker) 0; sin(yaw_marker) cos(yaw_marker) 0; 0 0 1];
        R_marker = Ryaw;
        %marker position remains the same

        %generate image
        x_beacon_g = t_marker + R_marker * x_beacon;
        x_beacon_l = inv(cam.Rc)*(x_beacon_g - cam.tc);
        x_beacon_p = cam.K * x_beacon_l;
        x_beacon_p = x_beacon_p ./ repmat(x_beacon_p(3,:), 3, 1);
        x_beacon_p(1:2,:) = x_beacon_p(1:2,:) + 1*randn(2, size(x_beacon_p, 2)); %generate noise
        %data_p = zeros(width*height, 1);
        msg.Data = 10*ones(cam.width * cam.height * 3, 1);
        indsT = zeros(1, cam.width * cam.height);
        for k = 1:size(x_beacon_p, 2)
            inds = ((pixel_n - cam.width *floor(pixel_n/cam.width)) - x_beacon_p(1,k)).^2 + ...
            (floor(pixel_n/cam.width) - x_beacon_p(2,k)).^2  < 15;
            indsT = logical(indsT + inds);
            indsT(indsT > 1) = 1;
            %data_p(inds) = 1;
        end
%         im(logical(data_p)) = 1;
%         h = imshow(im);
%         im(logical(data_p)) = 0;

        %send image to topic
        msg.Data(pixel_n(indsT)*3 + 1) = uint8(255);
        msg.Data(3*300*cam.width + (3*300 + 1:3*300 + 3) ) = [uint8(255) 3 2];
        msg.Header.Seq = msg.Header.Seq + 1;
        msg.Header.Stamp = rostime('now');
        send(pub,msg);
        
    end
    
end

%% Procustres analysis

%clean
clear all;
close all;
clc;

%load matlab libraries
matlab_repo = '/media/duartecdias/5A8CD5D48CD5AAB1/Dropbox/Public/Áreas/Trabalho/Investigação/Doutoramento/IST-EPFL/Tese/Implementação';
addpath(matlab_repo);
matlab_libraries = strcat(matlab_repo, '/matlab_libraries/CEMRA');
addpath(strcat(matlab_libraries, '/DataAnalysis'));

%main location
exp_repo = '/home/duartecdias/Repos/CEMRA/bags/bags-v3/';

%so the version from matlab is not used (errors on webots and ros are generated otherwise)
setenv('LD_PRELOAD','/usr/lib/x86_64-linux-gnu/libstdc++.so.6:/usr/lib/x86_64-linux-gnu/libexpat.so');

%parameters
exp_number = 1;
number_of_points = 6;
topic_name = '/robot2/cam_pose';
procrustres_repo = strcat(exp_repo, 'single_point/Exp', num2str(exp_number), '/');
setenv('PROCRUSTRES_REPO', procrustres_repo);
setenv('NUMBER_OF_POINTS', num2str(number_of_points));
setenv('TOPIC_NAME', topic_name);

%process bag
if exist(strcat(procrustres_repo, 'data_procrustres.txt'), 'file') ~= 2
!./procrustres_bag.sh $PROCRUSTRES_REPO $NUMBER_OF_POINTS $TOPIC_NAME
end

%load bag
files = [];
ind = 1;
s.name_i = strcat(procrustres_repo, 'data_procrustres.txt'); s.name_o = strcat(procrustres_repo, 'data_procrustres_processed.txt'); s.nheaders = 1; s.eliminations = 4;  s.processed = 0; files = cat(1, files, s); fids.MEASUREMENT_CAM = ind; ind = ind + 1;
files = get_bag_file_names(procrustres_repo, files);
data = load(files(1).name_o);

%get attitude information
x = data(:,7);
y = data(:,8);
z = data(:,9);
w = data(:,10);
roll=atan2(2*(w.*x+z.*y),(1-2*(x.*x+y.*y)));
pitch=asin(2*(w.*y-x.*z));
yaw=atan2(2*(w.*z+x.*y),(1-2*(y.*y+z.*z)));
R_markers = [];
for k = 1:size(x, 1)
    r = roll(k); p = pitch(k); y = yaw(k);
    Rroll=[1,0,0;0,cos(r),-sin(r);0,sin(r),cos(r)];
    Rpitch=[cos(p),0,sin(p);0,1,0;-sin(p),0,cos(p)];
    Ryaw=[cos(y),-sin(y),0;sin(y),cos(y),0;0,0,1];
    R_marker=Ryaw*Rpitch*Rroll;
    R_markers = cat(3, R_markers, R_marker);
end

%select points for procustres analysis
close all;
figure(1); hold on;
plot(data(:,4), 'r');
plot(data(:,5), 'g');
plot(data(:,6), 'b');
n = 6;
[x,y] = ginput(n);

%visualize selected points
x = floor(x);
PI = [data(x, 4:6)];
pf = PI + 0.2*reshape(R_markers(:,3,x), 3, length(x))';
close all;
figure(1); hold on;
axis([0 5 -3 3 -2 2]);
plot3(PI(1,1), PI(1,2), PI(1,3),'*r')
plot3(PI(2,1), PI(2,2), PI(2,3),'*g')
plot3(PI(3,1), PI(3,2), PI(3,3),'*b')
plot3(PI(4,1), PI(4,2), PI(4,3),'*k')
plot3(PI(5,1), PI(5,2), PI(5,3),'*y')
plot3(PI(6,1), PI(6,2), PI(6,3),'*m')
line([PI(:,1) pf(:,1)]', [PI(:,2) pf(:,2)]', [PI(:,3) pf(:,3)]', 'color', 'k');

%define the real points
X_true = [2.7, 0.3, 0.2;
          0.2, 0.3, 0.2;
          0.2, 2.5, 0.2;
          0.8, 2.83, 0.2;
          2.7, 2.83, 0.2;
          1.5, 1.5, 0.2;];

%compute procustres transform
[d,Z,transform] = procrustes(PI, X_true);

%check transform and finalize
if transform.b > 1.1 || transform.b < 0.9
    'something is wrong with the points'
else
    
    %load camera parameters
    cam = load(strcat(procrustres_repo, 'camera.mat'), 'cam');
    cam = cam.cam;
    
    %compute the transform
    cam.Rc = transform.T;
    cam.tc = -cam.Rc*(transform.c(1, :)');
    
    %check the error (plot and quantitative)
    X_true_proj = inv(cam.Rc)*((X_true - repmat(cam.tc', length(X_true), 1))');
    X_meas_proj = (cam.Rc*PI' + repmat(cam.tc, 1, size(PI, 1)))';
    figure(1); hold on;
    plot3(X_true_proj(1,:), X_true_proj(2,:), X_true_proj(3,:), 'r*');
    dX_true = X_meas_proj - X_true;
    err = sqrt(sum(dX_true(:,1:2).*dX_true(:,1:2), 2)) %this is the 2D error
    
    %load camera parameters
    save(strcat(procrustres_repo, 'camera.mat'), 'cam');
    
end

%% Predicted blob analysis

%marker definition
x_beacon = [0.0, 0.0,0.00; 0.0,0.13,0.0; 0.106,0.106,0.00; 0.191,0.0,0.0]'; %testing ones

%chosing third positiong from the procustres analysis
t_marker =  data(x(6), 4:6)';
R_marker = R_markers(:, :, x(6));

%project position into image
x_beacon_l = repmat(t_marker, 1, size(x_beacon, 2)) + R_marker * x_beacon;
x_beacon_l = x_beacon_l;
x_beacon_p = cam.K * x_beacon_l;
x_beacon_p = x_beacon_p ./ repmat(x_beacon_p(3,:), 3, 1);

%plot projection
figure(2); hold on; axis([0 cam.width 0 cam.height]);
plot(x_beacon_p(1, :), cam.height - x_beacon_p(2, :), 'b*');

%% Write camera configuration file

%write configuration file
fileID = fopen(strcat(procrustres_repo, 'measurement_params.txt'),'w');
fprintf(fileID,'camera_pose\n');
fprintf(fileID,'vp: 0.0333\n');
fprintf(fileID,'vh: 0.001\n');
fprintf(fileID,'R: ');
fprintf(fileID,'%.4f ', cam.Rc(1,:));
fprintf(fileID,'%.4f ', cam.Rc(2,:));
fprintf(fileID,'%.4f ', cam.Rc(3,:));
fprintf(fileID,'\nt: ');
fprintf(fileID,'%.4f ', cam.tc(:));
fclose(fileID);

%% Mesh analysis

%clean
clear all;
close all;
clc;

%load matlab libraries
matlab_repo = '/media/duartecdias/5A8CD5D48CD5AAB1/Dropbox/Public/Áreas/Trabalho/Investigação/Doutoramento/IST-EPFL/Tese/Implementação';
addpath(matlab_repo);
matlab_libraries = strcat(matlab_repo, '/matlab_libraries/CEMRA');
addpath(strcat(matlab_libraries, '/DataAnalysis'));

%main location
exp_repo = '/home/duartecdias/Repos/CEMRA/bags/bags-v3/';

%so the version from matlab is not used (errors on webots and ros are generated otherwise)
setenv('LD_PRELOAD','/usr/lib/x86_64-linux-gnu/libstdc++.so.6:/usr/lib/x86_64-linux-gnu/libexpat.so');

%parameters
exp_number = 1;
topic_name = '/robot2/cam_pose';
mesh_repo = strcat(exp_repo, 'mesh/Exp', num2str(exp_number), '/');
setenv('MESH_REPO', mesh_repo);
setenv('TOPIC_NAME', topic_name);

%process bag
if exist(strcat(mesh_repo, 'data_mesh.txt'), 'file') ~= 2
!./mesh_bag.sh $MESH_REPO $TOPIC_NAME
end

%load bag
files = [];
ind = 1;
s.name_i = strcat(mesh_repo, 'data_mesh.txt'); s.name_o = strcat(mesh_repo, 'data_mesh_processed.txt'); s.nheaders = 1; s.eliminations = 4;  s.processed = 0; files = cat(1, files, s); fids.MEASUREMENT_CAM = ind; ind = ind + 1;
files = get_bag_file_names(mesh_repo, files);
data = load(files(1).name_o);

%get attitude information
x = data(:,7);
y = data(:,8);
z = data(:,9);
w = data(:,10);
roll=atan2(2*(w.*x+z.*y),(1-2*(x.*x+y.*y)));
pitch=asin(2*(w.*y-x.*z));
yaw=atan2(2*(w.*z+x.*y),(1-2*(y.*y+z.*z)));
R_markers = [];
for k = 1:size(x, 1)
    r = roll(k); p = pitch(k); y = yaw(k);
    Rroll=[1,0,0;0,cos(r),-sin(r);0,sin(r),cos(r)];
    Rpitch=[cos(p),0,sin(p);0,1,0;-sin(p),0,cos(p)];
    Ryaw=[cos(y),-sin(y),0;sin(y),cos(y),0;0,0,1];
    R_marker=Ryaw*Rpitch*Rroll;
    R_markers = cat(3, R_markers, R_marker);
end

%visualize selected points
PI = [data(:, 4:6)];
pf = PI + 0.2*reshape(R_markers(:,3,:), 3, size(R_markers, 3))';
close all;
figure(1); hold on;
axis([0 5 -3 3 -2 2]);
plot3(PI(:,1), PI(:,2), PI(:,3),'.r')
line([PI(:,1) pf(:,1)]', [PI(:,2) pf(:,2)]', [PI(:,3) pf(:,3)]', 'color', 'k');

figure(2); hold on;
plot(data(:,4), 'r');
plot(data(:,5), 'g');
plot(data(:,6), 'b');
