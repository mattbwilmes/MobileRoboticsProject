clear all
% Note: need to chnage the path when you use it!!


%load('/Volumes/Seagate Backup Plus Drive/MobileRoboticsProject-wy-full/MobileRoboticsProject-wy/cvo_xyz/new_edge/transform_dict_with_new_edge.mat')
%load('/Volumes/Seagate Backup Plus Drive/MobileRoboticsProject-wy-full/MobileRoboticsProject-wy/cvo_desk/new_edge/transform_dict_new_edge.mat')


%load('/Volumes/Seagate Backup Plus Drive/MobileRoboticsProject-wy-full/MobileRoboticsProject-wy/dvo_xyz/freiburg1_xyz_tform.mat')
%load('/Volumes/Seagate Backup Plus Drive/MobileRoboticsProject-wy-full/MobileRoboticsProject-wy/dvo_desk/freiburg1_desk_tform.mat')

%load('/Volumes/Seagate Backup Plus Drive/MobileRoboticsProject-wy-full/MobileRoboticsProject-wy/open3d_xyz/new/relative_xyz.mat')
%load('/Volumes/Seagate Backup Plus Drive/MobileRoboticsProject-wy-full/MobileRoboticsProject-wy/open3d_xyz/new/relative_xyz_color.mat')


%load('/Volumes/Seagate Backup Plus Drive/MobileRoboticsProject-wy-full/MobileRoboticsProject-wy/open3d_desk/new/relative_desk.mat')
%load('/Volumes/Seagate Backup Plus Drive/MobileRoboticsProject-wy-full/MobileRoboticsProject-wy/simple_dvo_xyz/freiburg1_xyz_frame_tforms.mat')
load('simple_dvo_desk/freiburg1_desk_frame_tforms.mat')


%fileId = fopen('assG_rgb_xyz2.txt','r');
%fileId = fopen('assG_rgb_desk2.txt','r');

%fileId = fopen('simple_dvo_xyz/s_dv0_assG_RGB_xyz.txt', 'r')
fileId = fopen('simple_dvo_desk/s_dvo_assG_RGB_desk.txt', 'r')

%tx ty tz qx qy qz qw
Ground_raw = fscanf(fileId, '%f', [9, 573])'; %generate the groundtruth mat file xyz 792 desk573
%tx ty tz qx qy qz qw
Ground_q = Ground_raw(:,2:8);
Ground_q_rot = Ground_q(:,4:7);
Ground_q_trans = Ground_q(:,1:3);
Ground_q_rot(:,[1,4]) = Ground_q_rot(:,[4,1]);
Ground_ts = Ground_raw(:,1);
Ground_rot_mat = quat2rotm(Ground_q_rot);
Ground_mat = {};
for i = 1:length(Ground_rot_mat)
    Ground_mat{i,1} = Ground_ts(i);
    Ground_mat{i,2} = padarray(Ground_rot_mat(:,:,i), [1 1], 'post');
    Ground_mat{i,2}(1:3,4)= Ground_q_trans(i,:); 
    Ground_mat{i,2}(4,4)=1;
end


%dataset_name = 'freiburg1_xyz'; %792
dataset_name = 'freiburg1_desk'; %573

dataset_path = ...
    strcat('rgbd_tum/', ...
    dataset_name, '/');
pcd_full_path = strcat(dataset_path,'pcd_full');
pcd_full_dir_info = dir(fullfile(pcd_full_path, '*.pcd'));
num_pcd_files = length(pcd_full_dir_info);
time_step = [];

%num_pcd_files = length(Ground_mat);

for i = 1:num_pcd_files-1
    time_step(i)=str2num(pcd_full_dir_info(i+1).name(1:end-4)); % the real time step from current frame to world
end

% cvo_xyz_align_gt = {};
% 
% for i = 1:num_pcd_files-1
%     cvo_xyz_align_gt{i,1} = time_step(i);
%     cvo_xyz_align_gt{i,2} = Ground_mat{i,2}*transform_dict{i,2};% need to change this file!!!
% end

% cvo_desk_align_gt = {};
% 
% for i = 1:num_pcd_files-1
%     cvo_desk_align_gt{i,1} = time_step(i);
%     cvo_desk_align_gt{i,2} = Ground_mat{i,2}*transform_dict{i,2};% need to change this file!!!
% end

% dvo_xyz_align_gt = {};
% 
% for i = 1:num_pcd_files-1
%     dvo_xyz_align_gt{i,1} = time_step(i);
%     dvo_xyz_align_gt{i,2} = Ground_mat{i,2}*transformation{i,2};% need to change this file!!!
% end


% dvo_desk_align_gt = {};
% 
% for i = 1:num_pcd_files-1
%     dvo_desk_align_gt{i,1} = time_step(i);
%     dvo_desk_align_gt{i,2} = Ground_mat{i,2}*transformation{i,2};% need to change this file!!!
% end


% 
% open3d_xyz_align_gt = {};
% 
% for i = 1:num_pcd_files-1
%     open3d_xyz_align_gt{i,1} = time_step(i);
%     open3d_xyz_align_gt{i,2} = Ground_mat{i,2}*transformationMatrix{i};% need to change this file!!!
% end
    

% open3d_desk_align_gt = {};
% 
% for i = 1:num_pcd_files-1
%     open3d_desk_align_gt{i,1} = time_step(i);
%     open3d_desk_align_gt{i,2} = Ground_mat{i,2}*transformationMatrix{i};% need to change this file!!!
% end

% s_dvo_xyz_align_gt = {};
% 
% for i = 1:num_pcd_files-1
%     s_dvo_xyz_align_gt{i,1} = Ground_raw(i+1,9);
%     s_dvo_xyz_align_gt{i,2} = Ground_mat{i,2}*frame_transform{i,2};% need to change this file!!!
% end

s_dvo_desk_align_gt = {};

for i = 1:num_pcd_files-1
    s_dvo_desk_align_gt{i,1} = Ground_raw(i+1,9);
    s_dvo_desk_align_gt{i,2} = Ground_mat{i,2}*frame_transform{i,2};% need to change this file!!!
end






