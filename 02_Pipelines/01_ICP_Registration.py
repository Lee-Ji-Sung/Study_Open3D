#####################################################################
#   01_ICP_Registration.py
#       - This shows how to use ICP_Registration in Open3D
#           . helper visualization function
#           . Input
#           . Point-to-Point ICP
#           . Point-to-Plane ICP
#       - open3d version : 0.15.1
#   written by jslee
#   date : 2000.00.00
#####################################################################
import copy

import numpy as np


import open3d as o3d



# helper visualization function
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.7, 0])
    target_temp.paint_uniform_color([0, 0.6, 0.9])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])




if __name__ == '__main__':



    ######################################################################################
    # Input
    ######################################################################################
    demo_icp_pcds = o3d.data.DemoICPPointClouds()
    source = o3d.io.read_point_cloud(demo_icp_pcds.paths[0])
    target = o3d.io.read_point_cloud(demo_icp_pcds.paths[1])

    threshold = 0.02
    trans_init = np.asarray([[0.862, 0.011, -0.507, 0.5],
                             [-0.139, 0.967, -0.215, 0.7],
                             [0.487, 0.255, 0.835, -1.4],
                             [0.0, 0.0, 0.0, 1.0]])

    draw_registration_result(source, target, trans_init)


    # evaluate_registration
    #   - fitness : which measures the overlapping area (# of inlier correspondences / # of points in target). The higher the better.
    #   - inlier_rmse : which measures the RMSE of all inlier correspondences. The lower the better.
    print(f'Initial alignment')
    evaluation = o3d.pipelines.registration.evaluate_registration(source, target, threshold, trans_init)
    print(f'evaluation : {evaluation}')
    ######################################################################################



    ######################################################################################
    # Point-to-Point ICP
    ######################################################################################
    # IPC algorithm step
    #   1. Find correspondence set K={(p,q)} from target point cloud P, and source point cloud Q transformed with current transformation matrix T.
    #   2. Update the transformation T by minimizing an objective function E(T) defined over the correspondence set K.
    print(f'Apply point-to-point ICP')
    reg_p2p = o3d.pipelines.registration.registration_icp(source, target, threshold, trans_init,
                                                          o3d.pipelines.registration.TransformationEstimationPointToPoint())
    print(f'reg_p2p : {reg_p2p}')
    print(f'Transformation is :')
    print(f'reg_p2p.transformation :\n{reg_p2p.transformation}')
    draw_registration_result(source, target, reg_p2p.transformation)


    reg_p2p = o3d.pipelines.registration.registration_icp(source, target, threshold, trans_init,
                                                          o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                                                          o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
    print(f'reg_p2p : {reg_p2p}')
    print(f'Transformation is :')
    print(f'reg_p2p.transformation : \n{reg_p2p.transformation}')
    draw_registration_result(source, target, reg_p2p.transformation)
    ######################################################################################



    ######################################################################################
    # Point-to-Plane ICP
    ######################################################################################
    print(f'Apply point-to-plaen ICP')
    reg_p2l = o3d.pipelines.registration.registration_icp(source, target, threshold, trans_init,
                                                          o3d.pipelines.registration.TransformationEstimationPointToPlane())
    print(f'reg_p2l : {reg_p2l}')
    print(f'Transformation is :')
    print(f'reg_p2l.transformation :\n{reg_p2l.transformation}')
    draw_registration_result(source, target, reg_p2l.transformation)
    ######################################################################################



    print(f'Done')