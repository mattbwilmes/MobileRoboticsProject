#!/usr/bin/python

from open3d import *
import numpy as np
import matplotlib.pyplot as plt
import glob
from pytransform3d import transformations, rotations

if __name__ == "__main__":
    imgColor_list = []
    imgDepth_list = []
    for filename in glob.glob('/media/psf/Dropbox/project_568/rgbd_dataset_freiburg1_xyz/rgb/*.png'):
        im = read_image(filename)
        imgColor_list.append(im)
    for filename in glob.glob('/media/psf/Dropbox/project_568/rgbd_dataset_freiburg1_xyz/depth/*.png'):
        im = read_image(filename)
        imgDepth_list.append(im)
    # color method
    """f1 = open('trans_color .txt', 'wb')
    np.savetxt(f1, [])
    """
    # trans_hybrid method
   # f2 = open('trans_hybrid.txt', 'wb')
   # np.savetxt(f2, [])
    # trajectory by transformation matrix
   # f3 = open('trajectory_cal.txt', 'wb')
   # np.savetxt(f3, [])
    # error
    #f4 = open('error.txt', 'wb')
   # np.savetxt(f4, [])
    # read ground truth
    f = open(r"/media/psf/Dropbox/project_568/rgbd_dataset_freiburg1_xyz/groundtruth.txt", "r")
    lines = f.readlines()

    pinhole_camera_intrinsic = read_pinhole_camera_intrinsic(
        "/media/psf/Dropbox/project_568/rgbd_dataset_freiburg1_xyz/camera_primesense.json")
    quaternion = [1.3543, 0.6306, 1.6360, 0.6129, 0.5966, -0.3316, -0.3980]
    #np.savetxt(f3, quaternion)
    Error_list = []
    Error_pos = []
    Error_ang = []
    for i in range(len(imgColor_list)-2):

        # read camera intrinsic matrix

        #print(pinhole_camera_intrinsic.intrinsic_matrix)
        # read first two frames
        img1_color = imgColor_list[i]
        img2_color = imgColor_list[i+1]
        img1_depth = imgDepth_list[i]
        img2_depth = imgDepth_list[i+1]
        # generate rgbd image from color and depth image
        rgbd_img1 = create_rgbd_image_from_tum_format(
            img1_color, img1_depth);
        rgbd_img2 = create_rgbd_image_from_tum_format(
            img2_color, img2_depth);

        # define source image and target image
        target_rgbd_image = create_rgbd_image_from_tum_format(img1_color, img1_depth)
        source_rgbd_image = create_rgbd_image_from_tum_format(img2_color, img2_depth)
        # generate point cloud data for target image
        target_pcd = create_point_cloud_from_rgbd_image(target_rgbd_image, pinhole_camera_intrinsic)
        # Flip it, otherwise the pointcloud will be upside down
        # target_pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        option = OdometryOption()
        odo_init = np.identity(4)
        """
        [success_color_term, trans_color_term, info] = compute_rgbd_odometry(
            source_rgbd_image, target_rgbd_image,
            pinhole_camera_intrinsic, odo_init,
            RGBDOdometryJacobianFromColorTerm(), option)
            """
        [success_hybrid_term, trans_hybrid_term, info] = compute_rgbd_odometry(
            source_rgbd_image, target_rgbd_image,
            pinhole_camera_intrinsic, odo_init,
            RGBDOdometryJacobianFromHybridTerm(), option)
        """
        if success_color_term:
            
            np.savetxt(f1, trans_color_term)
        """

        if success_hybrid_term:

            #np.savetxt(f2, trans_hybrid_term)
            trans_target2source = np.linalg.pinv(trans_hybrid_term)
            quaternion = quaternion + transformations.pq_from_transform(trans_target2source)

            quaternion = np.asarray(quaternion)
            #np.savetxt(f3, quaternion)
            b = lines[i+5]
            groundtruth = [float(b[16:22]), float(b[23:29]), float(b[30:36]), float(b[59:66]), float(b[37:43]), float(b[44:50]), float(b[51:58])]
            distance_rot = rotations.quaternion_dist(groundtruth[3:7], quaternion[3:7])
            distance_trans = ((groundtruth[0] - quaternion[0]) ** 2 + (groundtruth[1] - quaternion[1]) ** 2 + (groundtruth[2] - quaternion[2]) ** 2) ** 0.5
            error = distance_trans+distance_rot
            error = np.asarray(error)
            Error_ang.append(distance_rot)
            Error_pos.append(distance_trans)
            Error_list.append(error)

            #np.savetxt(f4, error)


    #f1.close()
    #f2.close()
    #f3.close()
    #f4.close()
    f.close()
    plt.plot(range(len(imgColor_list)-2), Error_list, 'r')
    plt.show()

    plt.subplot(1, 2, 1)
    plt.title('error_angle')
    plt.plot(range(len(imgColor_list) - 2), Error_ang, 'r')
    plt.subplot(1, 2, 2)
    plt.title('error_pos')
    plt.plot(range(len(imgColor_list) - 2), Error_pos, 'r')
    plt.show()

    print("mean of error_angle")
    print(np.mean(Error_ang))
    print("mean of error_pos")
    print(np.mean(Error_pos))