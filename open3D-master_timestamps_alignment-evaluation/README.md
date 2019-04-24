# Open 3D Method

Based on the code found here: http://www.open3d.org/docs/tutorial/Basic/rgbd_odometry.html

See also, this paper by Zhou et al.: http://www.open3d.org/paper.pdf

## Procedure/Folder Structure:

### Evaluation

**associate.py**

*To associate the rgb figures with depth figures*
- Run code—navigate to this folder in Terminal and type the following code:

`python associate.py rgb.txt depth.txt`
- Save the .txt file
- Copy the output data and save as rgb_depth.txt

*To associate groundtruth with associated rgb and depth figures*
- Run code—navigate to this folder in Terminal and type the following code:

`python associate.py groundtruth.txt rgb_depth.txt`
- Save the txt file:
  - If fr1/desk: 
    - Copy the output data and save as ass_groundtruth_desk.txt 
  - If fr1/xyz:
    - Copy the output data and save as ass_groundtruth_xyz.txt 

**evalutate_rpe.py**

*To evaluate relative pose error*
- Navigate to this folder in Terminal and type the following code:

`python evaluate_rpe.py #groundtruth_file.txt #filename.txt --fixed_delta --delta_unit f --save rpe.txt --plot rpe.png --verbose`
- Save the txt file
- Save the output

### For dvo, cvo, open3D_color, open3D_hybrid:

**If you want to evaluate relative pose error of fr1/desk data set put the following name to replace the italicized .txt file**

*groundtruth_file.txt*
- ass_groundtruth_desk.txt

*filename.txt*
- dvo: Quat_desk_dvo.txt
- cvo: Quat_desk_cvo.txt
- open3D_color: Quat_desk_color.txt
- open3D_hybrid: Quat_desk_hybrid.txt

**If you want to evaluate relative pose error of fr1/xyz data set put the following name to replace the italicized .txt file**

*groundtruth_file.txt*
- ass_groundtruth_xyz.txt

*filename.txt*
- dvo: Quat_xyz_dvo.txt
- cvo: Quat_xyz_cvo.txt
- open3D_color: Quat_xyz_color.txt
- open3D_hybrid: Quat_xyz_hybrid.txt

### For simple Dvo:

**If you want to evaluate relative pose error of fr1/desk data set put the following name to replace the italicized .txt file**

*groundtruth_file.txt*
- S_dvo_assG_RGB_desk.txt

*filename.txt*
- Quat_desk_sDvo.txt

**If you want to evaluate relative pose error of fr1/xyz data set put the following name to replace the italicized .txt file**

*groundtruth_file.txt*
- S_dvo_assG_RGB_xyz.txt

*filename.txt*
- Quat_xyz_sDvo.txt

**evaluate_ate.py**

*To evaluate absolute trajectory error:*
- Navigate to this folder in Terminal and type the following code

`python evaluate_ate.py #groundtruth_file.txt #filename.txt --save ate.txt --plot ate.png --verbose`
- Save the txt file:
- Save the output

NOTE: The filenames are the same as what used in evaluate_rpe.py

**plot_trajectory.py**

*To plot the trajectory:*
- Navigate to this folder in Terminal and type the following code
  - If you want to plot trajectory of fr1/desk:

`python plot_trajectory.py ass_groundtruth_desk.txt Direct_Quat_desk_dvo.txt Direct_Quat_desk_cvo.txt Direct_Quat_desk_color.txt Direct_Quat_desk_hybrid.txt`
  - If you want to plot trajectory of fr1/xyz:

`python plot_trajectory.py ass_groundtruth_xyz.txt Direct_Quat_xyz_dvo.txt Direct_Quat_xyz_cvo.txt Direct_Quat_xyz_color.txt Direct_Quat_xyz_hybrid.txt`

**calMatrix.m**

*To calculate the absolute transformation matrix. You may want to change a little bit based on the dataset you used*
- Load the dataset you used, e.g. fram_trans_desk_hybrid.mat
- Correct the variable name based on the dataset you used
- Correct the iteration times based on the dataset you used, desk:791; xyz: 572; desk_sDvo:179; xyz_sDvo:231.
- Run the file and get the corresponding absolute transformation matrix.

**generateQuaternion.m**

*To generate Quaternion text file to be compared  with groundtruth*

- Change the ass_ground name based on the dataset used, i.e. ass_groundtruth_desk.mat or ass_groundtruth_xyz.mat
- Load the .mat file based on the dataset to be analyzed, e.g. dvo_xyz_align_gt.mat
- Change the fileID based on the method to be analyzed, e.g. fileID = fopen(‘ Quat_xyz_dvo.txt’, ‘w’);
- Run the code

### Open3D:

**generateTMatrix.py**

*To generate .mat  file for transformation matrix*
- Change the file_location and pinhole_camera_intrinsic into your own location path where you store the dataset.
- Run the code and the .mat files for both color and hybrid are saved.

