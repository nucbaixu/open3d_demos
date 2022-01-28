#深度图重建点云算法原理
#blog:https://zhuanlan.zhihu.com/p/34645454

# examples/python/geometry/camera_trajectory.py

import numpy as np
import open3d as o3d
import sys
import os

# only needed for tutorial, monkey patches visualization
sys.path.append('..')

if __name__ == "__main__":

    

    print("当前路径 -> %s" %os.getcwd())

    print("Testing camera in open3d ...")
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
    print(intrinsic.intrinsic_matrix)
    print(o3d.camera.PinholeCameraIntrinsic())
    x = o3d.camera.PinholeCameraIntrinsic(640, 480, 525, 525, 320, 240)
    print(x)
    print(x.intrinsic_matrix)
    o3d.io.write_pinhole_camera_intrinsic("test.json", x)
    y = o3d.io.read_pinhole_camera_intrinsic("test.json")
    print(y)
    print(np.asarray(y.intrinsic_matrix))

    print("Read a trajectory and combine all the RGB-D images.")
    pcds = []
    trajectory_path = '../test_data/RGBD/trajectory.log'
    trajectory = o3d.io.read_pinhole_camera_trajectory(trajectory_path)
    o3d.io.write_pinhole_camera_trajectory("test.json", trajectory)
    print(trajectory)
    print(trajectory.parameters[0].extrinsic)
    print(np.asarray(trajectory.parameters[0].extrinsic))
    for i in range(5):
        im1 = o3d.io.read_image(
            "../test_data/RGBD/depth/{:05d}.png".format(i))
        im2 = o3d.io.read_image(
            "../test_data/RGBD/color/{:05d}.jpg".format(i))
        im = o3d.geometry.RGBDImage.create_from_color_and_depth(
            im2, im1, 1000.0, 5.0, False)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            im, trajectory.parameters[i].intrinsic,
            trajectory.parameters[i].extrinsic)
       
        pcds.append(pcd)
    o3d.visualization.draw_geometries(pcds)
    print("")
