% used for 

clear all
%load('open3d_desk\groundMatrix_desk.mat')
%load('open3d_xyz/groundMatrix_xyz.mat')
load('dvo_xyz/freiburg1_xyz_tform.mat')
makeVideo = true;
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
dataset_name = 'freiburg1_xyz';
%dataset_name = 'freiburg1_desk';

dataset_path = ...
    strcat('rgbd_tum/', ...
    dataset_name, '/');
pcd_full_path = strcat(dataset_path,'pcd_full');
pcd_full_dir_info = dir(fullfile(pcd_full_path, '*.pcd'));
num_pcd_files = length(pcd_full_dir_info);

ptcloud_files = cell(num_pcd_files,2);

for iter = 1:num_pcd_files    
    ptcloud_files{iter,1} = fullfile(pcd_full_path, pcd_full_dir_info(iter).name);
end

% % Load the point cloud
ptCloudRef_full = pcread(ptcloud_files{1,1});

ptCloudCurrent_full = pcread(ptcloud_files{2,1});


ptCloudRef = ptcloud_edge_filter(ptCloudRef_full);
ptCloudCurrent = ptcloud_edge_filter(ptCloudCurrent_full);


gridSize = 0.1;

%ptCloudAligned = pctransform(ptCloudCurrent_full,affine3d(groundMatrix{1}'));
ptCloudAligned = pctransform(ptCloudCurrent_full,affine3d(transformation{1,2}'));

mergeSize = 0.015;
ptCloudScene = pcmerge(ptCloudRef_full, ptCloudAligned, mergeSize);


figure
hAxes = pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
title('Updated world scene')
% Set the axes property for faster rendering
hAxes.CameraViewAngleMode = 'auto';
hScatter = hAxes.Children;


for i = 3:iter-2
    cur_frame_num = i
    ptcloud_files{i,1};
    %groundMatrix{i};
    transformation{i,2}
    %accumTform = affine3d(groundMatrix{i-1}');

    accumTform = affine3d(transformation{i-1,2}');
    ptCloudAligned = pctransform(pcread(ptcloud_files{i,1}), accumTform);

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


