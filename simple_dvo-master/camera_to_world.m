function camera_to_world(gt_cam_assoc,sdvo_mat,output_mat)
% Generates a .mat file that has transformed the simple_dvo cam_origin
%   frame quaternions into ground truth frame quaternions
% Inputs:
%   gt_cam_assoc - File that was output by associate.py that matches the
%       groundtruth timestamps with the simple_dvo timestamps
%       format: [gt_time gt_t gt_q simple_dvo_time]
%   sdvo_mat - File containing timestamps and cam_origin transformations
%       format: {timestamp} {transformation}
%   output_mat - Desired file name of .mat file which will contain
%       timestamps and s_dvo transformations between frames
% Example function call:
% camera_to_world('s_dvo_assG_RGB_desk.txt', ...
%   'freiburg1_desk_tforms_newest.mat','freiburg1_desk_frame_tforms.mat')

% Open assoc file
file_id_assoc = fopen(gt_cam_assoc,'r');

% Get first line
assoc_line = fgetl(file_id_assoc);

% Find the first uncommented line
while assoc_line(1) == '#'
    assoc_line = fgetl(file_id_assoc);
end
fclose(file_id_assoc);
% Split the line into cells based on the space character
assoc_line_cells = strsplit(assoc_line);
% Extract translation vector
translation = str2double(assoc_line_cells(2:4));
% Extract quaternion vector
quaternion = str2double([assoc_line_cells(8) assoc_line_cells(5:7)]);
% Convert quaternion to rotation matrix
rotation = quat2rotm(quaternion);
% Generate the transformation matrix from 1 to world in ground truth data
w_T_1 = [rotation translation'; 0 0 0 1];

% Load the sdvo_mat file
sdvo_mat_cells_struct = load(sdvo_mat);
% Get the name of the cell inside
sdvo_mat_contents = fieldnames(sdvo_mat_cells_struct);
% Dereference the struct
sdvo_mat_cells = eval(strcat('sdvo_mat_cells_struct.',sdvo_mat_contents{1}));

% % Calculate transformation matrix from cam_origin to world frame
% % Get transformation matrix from 1 to cam_origin (c_T_1)
% 1_T_c = sdvo_mat_cells{1,2};
% % Get the cam_origin to world transformation
% %   w_T_1*inv(c_T_1)* = w_T_1*1_T_c = w_T_c
% w_T_c = w_T_1 / sdvo_tr1_T_cansform;
% 
% 

% Initialize cells for frame transformations
frame_transform = cell(length(sdvo_mat_cells)-3,2);

for row = 1:(length(sdvo_mat_cells)-2)
    % Get the next frame to cam_origin transformation
    cur_transform = sdvo_mat_cells{row,2};
    % Don't start inserting into output file until we have the first two frames
    if row ~= 1
        % Add timestamp
        frame_transform{row-1,1} = sdvo_mat_cells{row-1,1};
        % Add frame transformation
        frame_transform{row-1,2} = prev_transform \ cur_transform;
    end
    % Change current frame to cam_origin transform to previous transform
    prev_transform = cur_transform;
end

% Save frame_transform to a .mat file
save(output_mat,'frame_transform')

end

