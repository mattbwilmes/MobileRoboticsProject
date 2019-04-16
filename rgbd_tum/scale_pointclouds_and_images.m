function scale_pointclouds_and_images(fx, fy, cx, cy, depth_scaling_factor, ...
    scale_num, dataset_path, start_row, end_row)
%{
% Reads rgb and depth images from TUM dataset and outputs scaled versions
% of images and point clouds

For more information see: https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats

Author: Matt Wilmes
Date: 04-15-19
%}

% load association file, i.e., matched RGB and Depth timestamps
assoc_filename = strcat(dataset_path,'assoc.txt');
% Load rows startRow to endRow (or 1 to end if not chosen)
assoc = import_assoc_file(assoc_filename, start_row, end_row);
%assoc = import_assoc_file(assoc_filename);

% Create vector of down-scale values
scale_vec = zeros(1,scale_num);
for power = 1:scale_num
    scale_vec(power) = 1/(2^power);
end

% TODO: Hard-coded for now, need to make this adaptable
scale_folder = {'scale_05/','scale_025/','scale_0125/','scale_00625/'};

% Initialize cells
rgb_scaled = cell(size(assoc,1),length(scale_vec));
depth_scaled = cell(size(assoc,1),length(scale_vec));
point_cloud = cell(size(assoc,1),length(scale_vec));

% create edge point clouds
for i = 1:size(assoc,1)
%    pc_name = assoc(i,1); % using RGB image timestamp
   
    % load RGB image
    rgb = imread(strcat(dataset_path, assoc(i,2)));
   
    % Down-sample rgb images based on scale_vec
    for scale = scale_vec
        index = log2(1/scale);
        
        % Down-scale rgb image 4 times
        rgb_scaled{i,index} = imresize(rgb,scale);
        % Write scaled rgb image to <dataset_path>/<scale_folder>/rgb/<rgb_timestamp>.png
        path_to_save = strcat(dataset_path, scale_folder{index},'rgb/',assoc(i,1),'.png');
        imwrite(rgb_scaled{i,index}, path_to_save);
    end
   
    % load Depth image
    depth = double(imread(strcat(dataset_path, assoc(i,4))));
    % Set all zeros to NaN
    depth(depth == 0) = NaN;
    
    % Vectorize depth image
    depth_vec = depth(:);

    % Initial values for depth down-sample loop
    height = size(depth,1);
    width = size(depth,2);
    scaled_height = height/2;
    scaled_width = width/2;
    depth_vec_temp = zeros(scaled_height*scaled_width,1);

    % Down-sample depth images based on scale_vec
    % Algorithm based on simple_dvo's method: 
    %   https://github.com/muskie82/simple_dvo/blob/master/src/image_alignment.cpp
    % Note that this inserts along columns instead of rows
    for scale = scale_vec
    	index = log2(1/scale);
        % Scale down depth image, ignoring NaN values
        for row = 0:scaled_height-1
            for col = 0:scaled_width-1
                % Get the pixel values of a 2x2 grid
                top_left = depth_vec((2*col * scaled_height*2 + 2*row) + 1);
                top_right = depth_vec(2*col * scaled_height*2 + 2*row + 2);
                bottom_left = depth_vec(((2*col+1) * scaled_height*2 + 2*row) + 1);
                bottom_right = depth_vec((2*col+1) * scaled_height*2 + 2*row + 2);
                % Form 2x2 grid from depth image
                grid = [top_left top_right; bottom_left bottom_right];
                % Average the grid, ignoring nan values
                %depth_scaled_temp(row+1,col+1) = nanmean(grid,'all');
%                 depth_vec_temp(row*scaled_width + col + 1) = nanmean(grid,'all');
                depth_vec_temp(col*scaled_height + row + 1) = nanmean(grid,'all');
            end
        end
        
        % Set up for next loop
        depth_vec = depth_vec_temp;
        depth_scaled{i,index} = reshape(depth_vec_temp,[scaled_height scaled_width]);
        scaled_height = scaled_height/2;
        scaled_width = scaled_width/2;
        depth_vec_temp = zeros(scaled_height, scaled_width);
    end
   
    % Generate point clouds
    for scale = scale_vec
        index = log2(1/scale);
        % Careful here, assumes particular order/pattern for scale
        fx = fx / 2;
        fy = fy / 2;
        cx = cx / 2;
        cy = cy / 2;
        depth_scaling_factor = depth_scaling_factor * 2;
        
        % compute points xyz
        points = double(rgb_scaled{i,index});
        U = repmat(0:size(depth_scaled{i,index},2)-1, size(depth_scaled{i,index},1), 1); % x in R^2
        V = repmat([0:size(depth_scaled{i,index},1)-1]', 1, size(depth_scaled{i,index},2)); % y in R^2
        points(:,:,3) = depth_scaled{i,index} / depth_scaling_factor; % z in R^3
        points(:,:,1) = (U - cx) .* points(:,:,3) ./ fx; % x in R^3
        points(:,:,2) = (V - cy) .* points(:,:,3) ./ fy; % y in R^3

        point_cloud{i,index} = pointCloud(points, 'Color', rgb_scaled{i,index});
        % Remove non-edge points from point cloud
        edge_point_cloud = ptcloud_edge_filter(point_cloud{i,index});
        
        % Write edge point cloud to <dataset_path>/<scale_folder>/pcd_edge/<rgb_timestamp>.pcd
        path_to_save = strcat(dataset_path, scale_folder{index}, 'pcd_edge/', assoc(i,1), '.pcd');
        pcwrite(edge_point_cloud, path_to_save,'Encoding','ascii');
    end
   
    % Reset camera parameters for next iteration
    fx = fx * 2^scale_num;
    fy = fy * 2^scale_num;
    cx = cx * 2^scale_num;
    cy = cy * 2^scale_num;
    depth_scaling_factor = depth_scaling_factor / 2^scale_num;
end

end