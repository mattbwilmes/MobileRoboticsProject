#!/usr/bin/python

from open3d import *
import numpy as np
import matplotlib.pyplot as plt
import glob

if __name__ == "__main__":
    imgColor_list = []
    imgDepth_list = []
    for filename in glob.glob('/media/psf/Dropbox/project_568/rgbd_dataset_freiburg1_xyz/rgb/*.png'):
        im = read_image(filename)
        imgColor_list.append(im)
    for filename in glob.glob('/media/psf/Dropbox/project_568/rgbd_dataset_freiburg1_xyz/depth/*.png'):
        im = read_image(filename)
        imgDepth_list.append(im)
    f1 = open('trans_color .txt', 'wb')
    np.savetxt(f1, [])
    f2 = open('trans_hybrid.txt', 'wb')
    np.savetxt(f2, [])
    pinhole_camera_intrinsic = read_pinhole_camera_intrinsic(
        "/media/psf/Dropbox/project_568/rgbd_dataset_freiburg1_xyz/camera_primesense.json")
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

        [success_color_term, trans_color_term, info] = compute_rgbd_odometry(
            source_rgbd_image, target_rgbd_image,
            pinhole_camera_intrinsic, odo_init,
            RGBDOdometryJacobianFromColorTerm(), option)
        [success_hybrid_term, trans_hybrid_term, info] = compute_rgbd_odometry(
            source_rgbd_image, target_rgbd_image,
            pinhole_camera_intrinsic, odo_init,
            RGBDOdometryJacobianFromHybridTerm(), option)

        if success_color_term:
            #f = open('trans_color.txt', 'w')
            #f.write(trans_color_term)
            #f.write("\n")
            #f.close()
            #f.write(trans_color_term)
            #f.write('\n')
            #print(trans_color_term)
            np.savetxt(f1, trans_color_term)
            #source_pcd_color_term = create_point_cloud_from_rgbd_image(
            #    source_rgbd_image, pinhole_camera_intrinsic)

            #source_pcd_color_term.transform(trans_color_term)
            #draw_geometries([target_pcd, source_pcd_color_term])

        if success_hybrid_term:
            #f = open('trans_hybrid.txt', 'w')
            #f.write(trans_hybrid_term)
            #f.write("\n")
            #f.close()
            np.savetxt(f2, trans_hybrid_term)
            #f.write(trans_color_term)
            #f.write('\n')
            #print(trans_hybrid_term)
            #source_pcd_hybrid_term = create_point_cloud_from_rgbd_image(
            #    source_rgbd_image, pinhole_camera_intrinsic)
            #source_pcd_hybrid_term.transform(trans_hybrid_term)
            #draw_geometries([target_pcd, source_pcd_hybrid_term])
    f1.close()
    f2.close()