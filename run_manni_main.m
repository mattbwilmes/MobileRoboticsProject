%   Author: Maani Ghaffari Jadidi
%   Date:   January 1, 2019
clear all

makeVideo = false;
% Register Two Point Clouds
if makeVideo
    try
        votype = 'avifile';
        vo = avifile('video.avi', 'fps', 5);
    catch
        votype = 'VideoWriter';
        vo = VideoWriter('video', 'MPEG-4');
        set(vo, 'FrameRate', 5);
        open(vo);
    end
end




%dataset_name = 'freiburg1_xyz';
dataset_name = 'freiburg1_desk';
% dataset_path = ...
%     strcat('/path_to_your_dataset_folder/', ...
%     dataset_name, '/');
% TODO: Make this a custom path based on user
dataset_path = ...
    strcat('rgbd_tum/', ...
    dataset_name, '/');
pcd_full_path = strcat(dataset_path,'pcd_full');
%assoc = import_assoc_file(assoc_filename);

transform_dict = {};
transform_accumu_dict = {};
pcd_full_dir_info = dir(fullfile(pcd_full_path, '*.pcd'));

% assoc_filename = strcat(dataset_path, 'assoc.txt');
% % Load rows startRow to endRow (or 1 to end if not chosen)
% assoc = import_assoc_file(assoc_filename,1,2);

% Make a cell of these filepath names
num_pcd_files = length(pcd_full_dir_info);
ptcloud_files = cell(num_pcd_files,2);
for iter = 1:num_pcd_files    
    % Add .pcd file to ptcloud_files cell
    ptcloud_files{iter,1} = fullfile(pcd_full_path, pcd_full_dir_info(iter).name);
%     % Add corresponding rgb file to ptcloud_files cell
%     ptcloud_files{iter,2} = fullfile(dataset_path, assoc(iter,2));
end


% % Load the point cloud
ptCloudRef_full = pcread(ptcloud_files{1,1});

% Load second .pcd file as the source (moving) point cloud
ptCloudCurrent_full = pcread(ptcloud_files{2,1});


% 
ptCloudRef = ptcloud_edge_filter(ptCloudRef_full);
ptCloudCurrent = ptcloud_edge_filter(ptCloudCurrent_full);


gridSize = 0.1;
fixed = pcdownsample(ptCloudRef, 'gridAverage', gridSize);
moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);

% make rkhs registration object
rkhs_se3 = rkhs_se3_registration();
rkhs_se3.set_ptclouds(fixed, moving);
tic
rkhs_se3.align();
toc
fprintf('\n')
tform = rkhs_se3.tform;
transform_dict{1,1} = pcd_full_dir_info(1).name(1:end-4);
transform_dict{1,2} = tform.T';
% Transform the current point cloud to the reference coordinate system
% defined by the first point cloud.
% ptCloudAligned = pctransform(livingRoomData{2},tform);
% 
% mergeSize = 0.015;
% ptCloudScene = pcmerge(livingRoomData{1}, ptCloudAligned, mergeSize);


ptCloudAligned = pctransform(ptCloudCurrent_full,tform);

mergeSize = 0.015;
ptCloudScene = pcmerge(ptCloudRef_full, ptCloudAligned, mergeSize);


% Stitch a Sequence of Point Clouds
% To compose a larger 3-D scene, repeat the same procedure as above to
% process a sequence of point clouds. Use the first point cloud to
% establish the reference coordinate system. Transform each point cloud to
% the reference coordinate system. This transformation is a multiplication
% of pairwise transformations.

% Store the transformation object that accumulates the transformation.
accumTform = tform; 
transform_accumu_dict{1,1} = pcd_full_dir_info(1).name(1:end-4);
transform_accumu_dict{1,2} = accumTform.T';
figure
hAxes = pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
title('Updated world scene')
% Set the axes property for faster rendering
hAxes.CameraViewAngleMode = 'auto';
hScatter = hAxes.Children;

for i = 3:iter
    cur_frame_num = i
    ptcloud_files{i,1};
    ptCloudCurrent = ptcloud_edge_filter(pcread(ptcloud_files{i,1}));
       
    % Use previous moving point cloud as reference.
    fixed = moving;
    moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
    
    % Apply RKHS registration.
    rkhs_se3.set_ptclouds(fixed, moving); 
    tic
    rkhs_se3.align();
    toc
    fprintf('\n')
    tform = rkhs_se3.tform;
    
    transform_dict{i-1,1} = pcd_full_dir_info(i-1).name(1:end-4);
    transform_dict{i-1,2} = tform.T';

    % Transform the current point cloud to the reference coordinate system
    % defined by the first point cloud.
    accumTform = affine3d((accumTform.T'*tform.T')');
    ptCloudAligned = pctransform(pcread(ptcloud_files{i,1}), accumTform);
    
    transform_accumu_dict{i-1,1} = pcd_full_dir_info(i-1).name(1:end-4);

    transform_accumu_dict{i-1,2} = accumTform.T';

    % Update the world scene.
    ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned, mergeSize);

    % Visualize the world scene.
    hScatter.XData = ptCloudScene.Location(:,1);
    hScatter.YData = ptCloudScene.Location(:,2);
    hScatter.ZData = ptCloudScene.Location(:,3);
    hScatter.CData = ptCloudScene.Color;
    drawnow('limitrate')

    if makeVideo
        F = getframe(gcf);
        switch votype
          case 'avifile'
            vo = addframe(vo, F);
          case 'VideoWriter'
            writeVideo(vo, F);
          otherwise
            error('unrecognized votype');
        end
    end
    
    
end
if makeVideo
    close(vo)
end
% During the recording, the Kinect was pointing downward. To visualize the
% result more easily, let's transform the data so that the ground plane is
% parallel to the X-Z plane.
% angle = -pi/10;
% A = [1,0,0,0;...
%      0, cos(angle), sin(angle), 0; ...
%      0, -sin(angle), cos(angle), 0; ...
%      0 0 0 1];
% ptCloudScene = pctransform(ptCloudScene, affine3d(A));
% pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down', ...
%         'Parent', hAxes)
% title('Updated world scene')
% xlabel('X (m)')
% ylabel('Y (m)')
% zlabel('Z (m)')
