#!/usr/bin/python

from open3d import *
import numpy as np
import matplotlib.pyplot as plt
import glob


if __name__ == "__main__":

    # read camera intrinsic matrix
    pinhole_camera_intrinsic = read_pinhole_camera_intrinsic(
        "/media/psf/Dropbox/project_568/rgbd_dataset_freiburg1_xyz/camera_primesense.json")
    print(pinhole_camera_intrinsic.intrinsic_matrix)
    # read first two frames
    print("Read first two frames.")
    img1_color = read_image("/media/psf/Dropbox/project_568/rgbd_dataset_freiburg1_xyz/rgb/1305031102.175304.png")
    img2_color = read_image("/media/psf/Dropbox/project_568/rgbd_dataset_freiburg1_xyz/rgb/1305031102.211214.png")
    img1_depth = read_image("/media/psf/Dropbox/project_568/rgbd_dataset_freiburg1_xyz/depth/1305031102.160407.png")
    img2_depth = read_image("/media/psf/Dropbox/project_568/rgbd_dataset_freiburg1_xyz/depth/1305031102.194330.png")
    # generate rgbd image from color and depth image
    rgbd_img1 = create_rgbd_image_from_tum_format(
        img1_color, img1_depth);
    rgbd_img2 = create_rgbd_image_from_tum_format(
        img2_color, img2_depth);

    print(rgbd_img1)
    plt.subplot(1, 2, 1)
    plt.title('grayscale image')
    plt.imshow(rgbd_img1.color)
    plt.subplot(1, 2, 2)
    plt.title('depth image')
    plt.imshow(rgbd_img1.depth)
    plt.show()
    print(rgbd_img2)
    plt.subplot(1, 2, 1)
    plt.title('grayscale image')
    plt.imshow(rgbd_img2.color)
    plt.subplot(1, 2, 2)
    plt.title('depth image')
    plt.imshow(rgbd_img2.depth)
    plt.show()

    # define source image and target image
    target_rgbd_image = create_rgbd_image_from_tum_format(img1_color, img1_depth)
    source_rgbd_image = create_rgbd_image_from_tum_format(img2_color, img2_depth)
    # generate point cloud data for target image
    target_pcd = create_point_cloud_from_rgbd_image(target_rgbd_image, pinhole_camera_intrinsic)
    # Flip it, otherwise the pointcloud will be upside down
    # target_pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    option = OdometryOption()
    odo_init = np.identity(4)

    print ("\n")
    print(option)

    [success_color_term, trans_color_term, info] = compute_rgbd_odometry(
        source_rgbd_image, target_rgbd_image,
        pinhole_camera_intrinsic, odo_init,
        RGBDOdometryJacobianFromColorTerm(), option)
    [success_hybrid_term, trans_hybrid_term, info] = compute_rgbd_odometry(
        source_rgbd_image, target_rgbd_image,
        pinhole_camera_intrinsic, odo_init,
        RGBDOdometryJacobianFromHybridTerm(), option)

    if success_color_term:
        print ("\n")
        print("Using RGB-D Odometry")
        print(trans_color_term)
        # print type(trans_color_term)
        source_pcd_color_term = create_point_cloud_from_rgbd_image(
            source_rgbd_image, pinhole_camera_intrinsic)

        source_pcd_color_term.transform(trans_color_term)
        draw_geometries([target_pcd, source_pcd_color_term])

    if success_hybrid_term:
        print ("\n")
        print("Using Hybrid RGB-D Odometry")
        print(trans_hybrid_term)
        source_pcd_hybrid_term = create_point_cloud_from_rgbd_image(
            source_rgbd_image, pinhole_camera_intrinsic)
        source_pcd_hybrid_term.transform(trans_hybrid_term)
        draw_geometries([target_pcd, source_pcd_hybrid_term])
