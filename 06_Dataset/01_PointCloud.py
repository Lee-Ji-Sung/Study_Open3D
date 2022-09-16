#####################################################################
#   01_PointCloud.py
#       - This shows how to use PointCloud in Open3D
#           . PCDPointCloud
#           . PLYPointCloud
#           . EaglePointCloud
#           . LivingRoomPointCloud
#           . OfficePointCloud
#   written by jslee
#   date : 2000.00.00
#####################################################################





import numpy as np


import open3d as o3d


if __name__ == '__main__':


    ######################################################################################
    # PCDPointCloud
    ######################################################################################
    dataset = o3d.data.PCDPointCloud()
    pcd = o3d.io.read_point_cloud(dataset.path)
    o3d.visualization.draw_geometries([pcd])
    ######################################################################################

    ######################################################################################
    # PLYPointCloud
    ######################################################################################
    dataset = o3d.data.PLYPointCloud()
    pcd = o3d.io.read_point_cloud(dataset.path)
    o3d.visualization.draw_geometries([pcd])
    ######################################################################################



    ######################################################################################
    # EaglePointCloud
    ######################################################################################
    dataset = o3d.data.EaglePointCloud()
    pcd = o3d.io.read_point_cloud(dataset.path)
    o3d.visualization.draw_geometries([pcd])
    ######################################################################################



    ######################################################################################
    # LivingRoomPointCloud
    ######################################################################################
    dataset = o3d.data.LivingRoomPointClouds()
    pcds = []
    for pcd_path in dataset.paths:
        pcds.append(o3d.io.read_point_cloud(pcd_path))
    o3d.visualization.draw_geometries([pcds])
    ######################################################################################



    ######################################################################################
    # OfficePointCloud
    ######################################################################################
    dataset = o3d.data.OfficePointClouds()
    pcds = []
    for pcd_path in dataset.paths:
        pcds.append(o3d.io.read_point_cloud(pcd_path))
    ######################################################################################


    print(f'Done')