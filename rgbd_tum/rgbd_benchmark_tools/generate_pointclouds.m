
function generate_pointclouds(fx, fy, cx, cy, scaling_factor, ...
    dataset_path, pcd_full_path, pcd_edge_path, start_row, end_row)
%{
- The color images are stored as 640×480 8-bit RGB images in PNG format.
- The depth maps are stored as 640×480 16-bit monochrome images in PNG format.
- The color and depth images are already pre-registered using the OpenNI 
driver from PrimeSense, i.e., the pixels in the color and depth images 
correspond already 1:1.
- The depth images are scaled by a factor of 5000, i.e., a pixel value 
of 5000 in the depth image corresponds to a distance of 1 meter from the 
camera, 10000 to 2 meter distance, etc. A pixel value of 0 means missing 
value/no data.


The depth images in our datasets are reprojected into the frame of the color 
camera, which means that there is a 1:1 correspondence between pixels in the 
depth map and the color image.

The conversion from the 2D images to 3D point clouds works as follows. Note 
that the focal lengths (fx/fy), the optical center (cx/cy), the distortion 
parameters (d0-d4) and the depth correction factor are different for each 
camera. The Python code below illustrates how the 3D point can be computed 
from the pixel coordinates and the depth value:

fx = 525.0  # focal length x
fy = 525.0  # focal length y
cx = 319.5  # optical center x
cy = 239.5  # optical center y

factor = 5000 # for the 16-bit PNG files
# OR: factor = 1 # for the 32-bit float images in the ROS bag files

for v in range(depth_image.height):
  for u in range(depth_image.width):
    Z = depth_image[v,u] / factor;
    X = (u - cx) * Z / fx;
    Y = (v - cy) * Z / fy;

Camera          fx      fy      cx      cy      d0      d1      d2      d3      d4
(ROS default)	525.0	525.0	319.5	239.5	0.0     0.0     0.0     0.0     0.0
Freiburg 1 RGB	517.3	516.5	318.6	255.3	0.2624	-0.9531	-0.0054	0.0026	1.1633
Freiburg 2 RGB	520.9	521.0	325.1	249.7	0.2312	-0.7849	-0.0033	-0.0001	0.9172
Freiburg 3 RGB	535.4	539.2	320.1	247.6	0       0       0       0       0

For more information see: https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats

Author: Maani Ghaffari Jadidi
Date: 26-12-2018
%}

% % clc; clear; close all
% 
% % RGB intrinsic calibration parameters
% % Freiburg 1
% fx = 517.3;  % focal length x
% fy = 516.5;  % focal length y
% cx = 318.6;  % optical center x
% cy = 255.3;  % optical center y
% 
% % Freiburg 2
% % fx = 520.9;  % focal length x
% % fy = 521.0;  % focal length y
% % cx = 325.1;  % optical center x
% % cy = 249.7;  % optical center y
% 
% % Freiburg 3
% % fx = 535.4;  % focal length x
% % fy = 539.2;  % focal length y
% % cx = 320.1;  % optical center x
% % cy = 247.6;  % optical center y
% 
% scaling_factor = 5000;  % depth scaling factor
% 
% % set this flag true if want edge extracted point clouds instead of full
% % extract_edge = true;
% 
% % load association file, i.e., matched RGB and Depth timestamps
% dataset_name = 'freiburg1_xyz';
% % dataset_path = ...
% %     strcat('/path_to_your_dataset_folder/', ...
% %     dataset_name, '/');
% % TODO: Make this a custom path based on user
% 
% dataset_path = ...
%     strcat('/Users/MatthewWilmes/Documents/MATLAB/School/EECS568_MATLAB/Project/rgbd_tum/', ...
%     dataset_name, '/');

% load association file, i.e., matched RGB and Depth timestamps
assoc_filename = strcat(dataset_path,'assoc.txt');
% Load rows startRow to endRow (or 1 to end if not chosen)
assoc = import_assoc_file(assoc_filename, start_row, end_row);
%assoc = import_assoc_file(assoc_filename);

% Make a pcd_full subfolder inside the dataset folder if it doesn't exist
% pcd_full_path = strcat(dataset_path,'pcd_full');
if ~exist(pcd_full_path,'dir')
    mkdir(pcd_full_path);
    addpath(pcd_full_path);
end

% Make a pcd_edge subfolder inside the dataset folder if it doesn't exist
% pcd_edge_path = strcat(dataset_path,'pcd_edge');
if ~exist(pcd_edge_path,'dir')
    mkdir(pcd_edge_path);
    addpath(pcd_edge_path);
end


% create point clouds
for i = 1:size(assoc,1)
%    pc_name = assoc(i,1); % using RGB image timestamp
   
   % load RGB image
   rgb = imread(strcat(dataset_path, assoc(i,2)));
   
   % load Depth image
   depth = double(imread(strcat(dataset_path, assoc(i,4))));
   depth(depth == 0) = nan;
   
   % compute points xyz
   points = double(rgb);
   U = repmat(0:size(depth,2)-1, size(depth,1), 1); % x in R^2
   V = repmat([0:size(depth,1)-1]', 1, size(depth,2)); % y in R^3
   points(:,:,3) = depth / scaling_factor; % z in R^3
   points(:,:,1) = (U - cx) .* points(:,:,3) ./ fx; % x in R^3
   points(:,:,2) = (V - cy) .* points(:,:,3) ./ fy; % y in R^3
   
   point_cloud = pointCloud(points, 'Color', rgb);
   
%    % Remove non-edge points from point cloud if true
%    if extract_edge
%        point_cloud = ptcloud_edge_filter(point_cloud);
%    end
   
   % Write full point cloud to <dataset_path>/pcd_full/<rgb_timestamp>.pcd
   path_to_save = strcat(dataset_path, 'pcd_full/', assoc(i,1), '.pcd');
   pcwrite(point_cloud, path_to_save,'Encoding','ascii');
   
   % Remove non-edge points from point cloud
   edge_point_cloud = ptcloud_edge_filter(point_cloud);

   % Write edge point cloud to <dataset_path>/pcd_edge/<rgb_timestamp>.pcd
   path_to_save = strcat(dataset_path, 'pcd_edge/', assoc(i,1), '.pcd');
   pcwrite(edge_point_cloud, path_to_save,'Encoding','ascii');
   
end

end