%{
This script processes any dataset of rgb and depth images found here:
https://vision.in.tum.de/data/datasets/rgbd-dataset/download

The user must download the dataset as a .tgz file and unzip it in the
proper folder structure:
-->dataset_path
---->dataset_name
------>unzipped contents of .tgz file
------>assoc.txt (provided by user)

%Author: Matt Wilmes
%Date: 03-24-2019
%}

clc; clear; close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% User Input Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% RGB intrinsic calibration parameters
% Freiburg 1
fx = 517.3;  % focal length x
fy = 516.5;  % focal length y
cx = 318.6;  % optical center x
cy = 255.3;  % optical center y

% Freiburg 2
% fx = 520.9;  % focal length x
% fy = 521.0;  % focal length y
% cx = 325.1;  % optical center x
% cy = 249.7;  % optical center y

% Freiburg 3
% fx = 535.4;  % focal length x
% fy = 539.2;  % focal length y
% cx = 320.1;  % optical center x
% cy = 247.6;  % optical center y

scaling_factor = 5000;  % depth scaling factor

% % TODO: Make this a custom path based on user
% Path to the folder containing your rgbd_tum datasets
rgbd_tum_path = ...
    '/Users/MatthewWilmes/Documents/MATLAB/School/EECS568_MATLAB/Project/rgbd_tum/';
% Select the dataset you want to use
dataset_name = 'freiburg1_xyz';
% The extension of the images in the dataset (e.g. png, jpeg, etc.)
image_suffix = '.png';

% Set this flag to true if you have not made .pcd files for your dataset yet
make_pcd_files = false;
% If make_pcd_files is true, set the first row and last row you want to
%   read from the assoc.txt file
start_row = 1; % set to 1 if you want to start at the beginning of the file
end_row = inf; % set to inf if you want to go to the end of the file

% Set this flag to true if you want to see the full point cloud output
%   instead of the edges. NOTE: This is only useful for frames that have a
%   large offset
view_full_ptcloud = false;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic
total_time = 0;

% Generate name of folder path to dataset
dataset_path = strcat(rgbd_tum_path, dataset_name, '/');
% Generate names of folders to .pcd files
pcd_full_path = strcat(dataset_path,'pcd_full');
pcd_edge_path = strcat(dataset_path,'pcd_edge');

% Generate .pcd files from dataset if true
if make_pcd_files
    generate_pointclouds(fx, fy, cx, cy, scaling_factor, ...
        dataset_path, pcd_full_path, pcd_edge_path, start_row, end_row);
end

% Get the directory info of all .pcd files in the pcd_edge_path folder
pcd_edge_dir_info = dir(fullfile(pcd_edge_path, '*.pcd'));
% Make a cell of these filepath names
num_pcd_files = length(pcd_edge_dir_info);
ptcloud_files = cell(num_pcd_files,2);
% Remove non-alphabet letters from image extension in case a period was
%   included
image_suffix(isletter(image_suffix)==0)=[];

% Initialize cell matrix for transformations
transformation = cell(num_pcd_files-1,2);

for iter = 1:num_pcd_files
    % Get name of the iter'th pcd file
    pcd_file = pcd_edge_dir_info(iter).name;
    % Get name of the corresping rgb file
    rgb_file = replace(pcd_file,'pcd',image_suffix);
    % Add .pcd file to ptcloud_files cell
    ptcloud_files{iter,1} = fullfile(pcd_edge_path, pcd_file);
    % Add corresponding rgb file to same row of ptcloud_files cell
    ptcloud_files{iter,2} = fullfile(dataset_path, 'rgb/', rgb_file);
end

% Construct an instance of the rgbd_dvo class
rgbd_dvo = rgbd_dvo();

% Load first .pcd file as the target (fixed) point cloud
target_ptcloud = [];
% % Load the target point cloud
target_ptcloud.ptcloud = pcread(ptcloud_files{1,1});
% % Load the corresponding rgb image as grayscale
target_ptcloud.image = rgb2gray(imread(ptcloud_files{1,2}));
% Load second .pcd file as the source (moving) point cloud
%source_ptcloud = pcread(ptcloud_files{2,1});
source_ptcloud = [];
% % Load the source point cloud
source_ptcloud.ptcloud = pcread(ptcloud_files{2,1});
% % Load the corresponding rgb image as grayscale
source_ptcloud.image = rgb2gray(imread(ptcloud_files{2,2}));

total_time = total_time + toc;
% Index from the second .pcd to the last .pcd
for index_source_ptcloud = 2
    tic
    % Make a temporary target point cloud for down-sampling
%     target_ptcloud_temp.ptcloud = target_ptcloud.ptcloud;
%     target_ptcloud_temp.image = target_ptcloud.image;
%     % Make a temporary source point cloud for down-sampling
%     source_ptcloud_temp = source_ptcloud.ptcloud;

    % tic
    % Coarse-to-fine approach
    %grid_step = 0.005; 
    %while grid_step > 1*1e-2
    
    % Down-sample target point cloud
    target_ptcloud_downsampled.ptcloud = ...
        target_ptcloud.ptcloud;
        %pcdownsample(target_ptcloud.ptcloud,'gridAverage',grid_step);
        
    % Down-sample source point cloud
    source_ptcloud_downsampled = ...
        source_ptcloud.ptcloud;
        %pcdownsample(source_ptcloud.ptcloud,'gridAverage',grid_step);


    % Add corresponding image to down-sampled target point cloud struct
    target_ptcloud_downsampled.image = target_ptcloud.image;
    
    % Add the target and source point clouds to the object
    %rgbd_dvo.set_ptclouds(target_ptcloud, source_ptcloud_temp);

    for scale = [0.0625 0.125 0.25 0.5 1]
        
        % Scale the camera parameters in proportion with the camera image
        rgbd_dvo.fx = fx * scale;
        rgbd_dvo.fy = fy * scale;
        rgbd_dvo.cx = cx * scale;
        rgbd_dvo.cy = cy * scale;
        
        % Set point clouds as normal if no scaling required
        if scale == 1
            rgbd_dvo.set_ptclouds( ...
                target_ptcloud_downsampled, source_ptcloud_downsampled);
        % Otherwise implement modified function to scale necessary variables
        else
            rgbd_dvo.set_scaled_ptclouds( ...
                target_ptcloud_downsampled, source_ptcloud_downsampled, scale);
        end
        % Down-sample the target image
%         rgbd_dvo.fixed_image = imresize(target_ptcloud.image, scale);
%         rgbd_dvo.u_max = size(rgbd_dvo.fixed_image,2);
%         rgbd_dvo.v_max = size(rgbd_dvo.fixed_image,1);
%         % Down-smple the target gradient
% %         rgbd_dvo.
% 
%         %[Gmag,Gdir] = imresize(imgradient(target_ptcloud.image), scale);
%         [Gmag,Gdir] = imgradient(imresize(target_ptcloud.image, scale));
%         rgbd_dvo.imgrad.u = Gmag .* cosd(Gdir);
%         rgbd_dvo.imgrad.v = Gmag .* sind(Gdir);
%         rgbd_dvo.imgrad.mag = Gmag;
%         rgbd_dvo.imgrad.dir = Gdir;
% 
%         % Scale the point clouds
%         rgbd_dvo
        
%         imresize(source_ptcloud.image
%         rgbd_dvo.rgb2intensity(source.Color)
        % Align the point clouds
        rgbd_dvo.align();
        rgbd_dvo.tform.T
        % Down-sample
        %grid_step = grid_step*0.85;
    end

    % Reset the camera parameters
%     rgbd_dvo.fx = fx;
%     rgbd_dvo.fy = fy;
%     rgbd_dvo.cx = cx;
%     rgbd_dvo.cy = cy;
%     
%     % Reset the image dimensions
%     rgbd_dvo.u_max = size(target_ptcloud.image,2);
%     rgbd_dvo.v_max = size(target_ptcloud.image,1);
    
    % loop_time = toc;
    % total_time = total_time + loop_time

    % Plot full point clouds and transformed point clouds if true
    if view_full_ptcloud
        % Generate filenames for the full point clouds
        target_ptcloud_filename = ...
            replace(ptcloud_files{1,1},'pcd_edge','pcd_full');
        source_ptcloud_filename = ...
            replace(ptcloud_files{2,1},'pcd_edge','pcd_full');

        % Read full target and source point clouds
        target_ptcloud_full = pcread(target_ptcloud_filename);
        source_ptcloud_full = pcread(source_ptcloud_filename);
        % Apply final transform to full source point cloud
        source_ptcloud_full_transformed = pctransform(source_ptcloud_full, rgbd_dvo.tform);

        % Plot original point clouds on top of one another to show misalignment
        figure(2*index_source_ptcloud-1)
        pcshow(target_ptcloud_full)
        hold on
        pcshow(source_ptcloud_full)
        view(0,-90)
        title('Target and Source Point Clouds without Transform')

        % Plot point clouds on top of one another to show improved alignment
        figure(2*index_source_ptcloud)
        pcshow(target_ptcloud_full)
        hold on
        pcshow(source_ptcloud_full_transformed)
        view(0,-90)
        title('Target and Source Point Clouds with Transform')

    % Otherwise, plot edge point clouds and transformed point clouds
    else
        % Read full target and source point clouds
        target_ptcloud_edge = pcread(ptcloud_files{1,1});
        source_ptcloud_edge = pcread(ptcloud_files{2,1});
        % Apply final transform to edge source point cloud
        source_ptcloud_edge_transformed = pctransform(source_ptcloud_edge, rgbd_dvo.tform);
    
        % Plot original point clouds on top of one another to show misalignment
        figure(2*index_source_ptcloud-1)
        pcshow(target_ptcloud_edge)
        hold on
        pcshow(source_ptcloud_edge)
        view(0,-90)
        title('Target and Source Point Clouds without Transform')
    
        % Plot point clouds on top of one another to show improved alignment
        figure(2*index_source_ptcloud)
        pcshow(target_ptcloud_edge)
        hold on
        pcshow(source_ptcloud_edge_transformed)
        view(0,-90)
        title('Target and Source Point Clouds with Transform')
    end

    rgbd_dvo.tform.T
    sum(rgbd_dvo.residual'*rgbd_dvo.residual)
    
    % Set up for the next iteration, unless there is no next iteration
    if index_source_ptcloud ~= num_pcd_files
        % Choose the next target point cloud
        % % The source point cloud is now the target point cloud for the next pair
        target_ptcloud.ptcloud = source_ptcloud.ptcloud;
        % % Assocaite the proper grayscale image
        target_ptcloud.image = source_ptcloud.image;
        % Load the next source point cloud
        source_ptcloud.ptcloud = pcread(ptcloud_files{index_source_ptcloud+1,1});
        % % Load the corresponding rgb image as grayscale
        source_ptcloud.image = rgb2gray(imread(ptcloud_files{index_source_ptcloud+1,2}));
    end

    % Reset the predicted transformation matrix
    rgbd_dvo.R = eye(3);
    rgbd_dvo.T = zeros(3,1);
    rgbd_dvo.R_prev = eye(3);
    rgbd_dvo.T_prev = zeros(3,1);
    % Reset number of iterations
    rgdb_dvo.iterations = 0;

    % The timestamp is the filename of the source point cloud (minus the extension)
    timestamp = str2double(pcd_edge_dir_info(index_source_ptcloud-1).name(1:end-4));
    % Add timestamp to transformation cell
    transformation{index_source_ptcloud-1,1} = timestamp;
    % Add final transformation to corresponding cell
    transformation{index_source_ptcloud-1,2} = rgbd_dvo.tform.T';

    fprintf('Just finished transform from %d to %d\n',index_source_ptcloud,index_source_ptcloud-1)
    total_time = total_time + toc

end

% Save transformation cell as a .mat file
mat_file = strcat(dataset_path,dataset_name,'_tform','.mat');
save(mat_file,'transformation');
disp('done!')