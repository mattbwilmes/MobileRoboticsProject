#!/usr/bin/python

from open3d import *
import numpy as np
import matplotlib.pyplot as plt
import glob
from pytransform3d import transformations, rotations
from scipy import io


def read_file_list(filename):
    """
    Reads a trajectory from a text file.

    File format:
    The file format is "stamp d1 d2 d3 ...", where stamp denotes the time stamp (to be matched)
    and "d1 d2 d3.." is arbitary data (e.g., a 3D position and 3D orientation) associated to this timestamp.

    Input:
    filename -- File name

    Output:
    dict -- dictionary of (stamp,data) tuples

    """
    file = open(filename)
    data = file.read()
    lines = data.replace(",", " ").replace("\t", " ").split("\n")
    list = [[v.strip() for v in line.split(" ") if v.strip() != ""] for line in lines if
            len(line) > 0 and line[0] != "#"]
    list = [(float(l[0]), l[1:]) for l in list if len(l) > 1]
    return list


if __name__ == "__main__":

    # read matched pair of rgb and depth images
    image_list = read_file_list("rgb_depth.txt")
    imgColor_list = []
    imgDepth_list = []
    for i in range(len(image_list)):
        images = image_list[i]
        image_pair = images[1]
        rgb_image = image_pair[0]
        file_location = "/media/psf/Dropbox/project_568/rgbd_dataset_freiburg1_xyz/"
        rgb_filename = file_location+str(rgb_image)
        im_rgb = read_image(rgb_filename)
        imgColor_list.append(im_rgb)
        depth_image = image_pair[2]
        depth_filename = file_location + str(depth_image)
        im_depth = read_image(rgb_filename)
        imgDepth_list.append(im_depth)

    # save trans_hybrid method transformation matrix file
    fram_trans_dic = {}

    # read camera intrinsic matrix
    pinhole_camera_intrinsic = read_pinhole_camera_intrinsic(
        "/media/psf/Dropbox/project_568/rgbd_dataset_freiburg1_xyz/camera_primesense.json")

    # calculate transformation matrix for each pair
    for i in range(len(imgColor_list)-2):

        # read one pair of frames
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
        # target_pcd = create_point_cloud_from_rgbd_image(target_rgbd_image, pinhole_camera_intrinsic)
        # Flip it, otherwise the pointcloud will be upside down
        # target_pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

        option = OdometryOption()
        odo_init = np.identity(4)

        # generate transformation matrix
        [success_hybrid_term, trans_hybrid_term, info] = compute_rgbd_odometry(
            source_rgbd_image, target_rgbd_image,
            pinhole_camera_intrinsic, odo_init,
            RGBDOdometryJacobianFromHybridTerm(), option)

        if success_hybrid_term:
            key_dic = 'num:'+str(i)
            fram_trans_dic[key_dic] = trans_hybrid_term

    io.savemat('fram_trans_dic1.mat', mdict={'fram_trans_dic': fram_trans_dic})