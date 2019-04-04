function clout_out = ptcloud_edge_filter(cloud_in)
% extract edges from an organized RGB-D point cloud

% BW is a 480x640 logical (i.e. true or false)
BW = edge(rgb2gray(cloud_in.Color),'canny'); % can use sobel or canny
% Reshape Location from 480x640x3 to 307200x3
X = reshape(cloud_in.Location,[], 3, 1);
% Reshape Color from 480x640x3 to 307200x3
C = reshape(cloud_in.Color,[], 3, 1);
% Take only the values of X that were detected as edges by the Canny method
X = X(reshape(BW, [], 1), :);
% Take only the values of C that were detected as edges by the Canny method
C = C(reshape(BW, [], 1), :);
% Remove all rows of X that are NaN (Not a Number)
X = X(~isnan(X(:,1)),:);
% Remove all rows of C that are NaN (Not a Number)
C = C(~isnan(X(:,1)),:);

clout_out = pointCloud(X, 'Color', C);