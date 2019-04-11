% Select the /tf file you want to read from
tf_file = 'freiburg1_desk_tforms_newest.txt';
file_id = fopen(tf_file,'r');

% Number of lines in one block of the rostopic
num_lines = 20;

% Initialization of variables
rostopic = cell(num_lines,1);
transformation = cell(1,1);
num_tforms = 0;
camera_line_2 = {''};
dont_get_next_line = false;

% Repeat until the last transformation
while ~feof(file_id)
%for i = 1:2
    % Look at the first block of the file and put each line into the cell
    % There is no need to get the line_before if this flag is true
    if ~dont_get_next_line        
        line_before = strsplit(fgetl(file_id),',');
    end
    dont_get_next_line = false;
    % This line has the child_frame_id of "camera"
    camera_line = strsplit(fgetl(file_id),',');
    % The next line may have a valid timestamp
    line_after = strsplit(fgetl(file_id),',');
    
    % If there are two cameras in the block
    if strcmp(line_after{5},'camera')
        timestamp = str2double(line_before{3})*1e-9;
        camera_line_2 = line_after;
    % Otherwise extract timestamp from the next line
    else
        timestamp = str2double(line_after{3})*1e-9;
    end
    
    % Index to the next cell entry
    num_tforms = num_tforms + 1;
    
    % Set timestamp for this transformation
    transformation{num_tforms,1} = timestamp;

    % Extract translation from the cell: [tx ty tz]
    translation = ...
        [str2double(camera_line{6}) str2double(camera_line{7}) str2double(camera_line{8})];
    % Extract quaternion from the cell: [qw qx qy qz]
    quaternion = [str2double(camera_line{12}) ...
        str2double(camera_line{9}) str2double(camera_line{10}) str2double(camera_line{11})];
    % Convert quaternion to a rotation matrix
    rotation = quat2rotm(quaternion);
    % Save the transformation matrix
    transformation{num_tforms,2} = [rotation translation'; 0 0 0 1];
    
    % Consider the other camera line in this group
    if camera_line_2{1}
        
        % camerera_line_2 is the new camera_line now
        camera_line = camera_line_2;
        
        % Get last line of block
        % If second camera is the end of the file
        if feof(file_id)
            % Estimate the timestamp
            timestamp = timestamp + 0.01;
        % Otherwise, get the timestamp from the next line
        else
            line_after = strsplit(fgetl(file_id),',');
            timestamp = str2double(line_after{3})*1e-9;
        end
        
        % Increase count of transforms
        num_tforms = num_tforms + 1;
        
        % Set timestamp for this transformation
        transformation{num_tforms,1} = timestamp;
        
        % Extract translation from the cell: [tx ty tz]
        translation = ...
            [str2double(camera_line{6}) str2double(camera_line{7}) str2double(camera_line{8})];
        % Extract quaternion from the cell: [qw qx qy qz]
        quaternion = [str2double(camera_line{12}) ...
            str2double(camera_line{9}) str2double(camera_line{10}) str2double(camera_line{11})];
        % Convert quaternion to a rotation matrix
        rotation = quat2rotm(quaternion);
        % Save the transformation matrix
        transformation{num_tforms,2} = [rotation translation'; 0 0 0 1];

        % Clear contents of camera_line_2
        camera_line_2 = {''};
    end
    
    % Stop if at end of file
    if feof(file_id)
        break;
    else
        % Check next line
        line_after = strsplit(fgetl(file_id),',');
        
        
        % If line_after is not a delimiter line
        if ~strcmp(line_after,'--')
            % If line_after is a camera line
            if strcmp(line_after{5},'camera')
                camera_line = line_after;
                line_after = strsplit(fgetl(file_id),',');
                timestamp = str2double(line_after{3})*1e-9;

                % Index to the next cell entry
                num_tforms = num_tforms + 1;

                % Set timestamp for this transformation
                transformation{num_tforms,1} = timestamp;

    %             if num_tforms == 200
    %                 disp('pause')
    %             end

                % Extract translation from the cell: [tx ty tz]
                translation = ...
                    [str2double(camera_line{6}) str2double(camera_line{7}) str2double(camera_line{8})];
                % Extract quaternion from the cell: [qw qx qy qz]
                quaternion = [str2double(camera_line{12}) ...
                    str2double(camera_line{9}) str2double(camera_line{10}) str2double(camera_line{11})];
                % Convert quaternion to a rotation matrix
                rotation = quat2rotm(quaternion);
                % Save the transformation matrix
                transformation{num_tforms,2} = [rotation translation'; 0 0 0 1];            

                % Get delimiter so the next block can be started
                delimiter = fgetl(file_id);
            % Otherwise, go back to the beginning to finish this block
            else
                dont_get_next_line = true;
                line_before = line_after;
            end
        end
    end
end

mat_file = replace(tf_file,'.txt','.mat');
save(mat_file,'transformation')

fclose(file_id);
