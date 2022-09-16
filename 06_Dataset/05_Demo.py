#####################################################################
#   05_Demo.py
#       - This shows how to use Demo in Open3D
#           . Demo ICP Point Clouds
#           . Demo Colored ICP Point Clouds
#           . Demo Crop Point Cloud
#           . Demo Feature Matching Point Clouds
#           . Demo Pose Graph Optimization
#   written by jslee
#   date : 2000.00.00
#####################################################################





import numpy as np


import open3d as o3d


if __name__ == '__main__':


    ######################################################################################
    # Demo ICP Point Clouds - 3 point cloud fragments
    ######################################################################################
    dataset = o3d.data.DemoICPPointClouds()
    pcd0 = o3d.io.read_point_cloud(dataset.paths[0])
    pcd1 = o3d.io.read_point_cloud(dataset.paths[1])
    pcd2 = o3d.io.read_point_cloud(dataset.paths[2])

    # o3d.visualization.draw_geometries([pcd0])
    # o3d.visualization.draw_geometries([pcd1])
    # o3d.visualization.draw_geometries([pcd2])
    o3d.visualization.draw_geometries([pcd0 + pcd1 + pcd2])
    ######################################################################################



    ######################################################################################
    # Demo Colored ICP Point Clouds - 2 point cloud fragments of binary PCD format
    ######################################################################################
    dataset = o3d.data.DemoColoredICPPointClouds()
    pcd0 = o3d.io.read_point_cloud(dataset.paths[0])
    pcd1 = o3d.io.read_point_cloud(dataset.paths[1])

    o3d.visualization.draw_geometries([pcd0 + pcd1])
    ######################################################################################




    ######################################################################################
    # Demo Crop Point Cloud
    ######################################################################################
    dataset = o3d.data.DemoCropPointCloud()
    pcd = o3d.io.read_point_cloud(dataset.point_cloud_path)
    vol = o3d.visualization.read_selection_polygon_volume(dataset.cropped_json_path)
    chair = vol.crop_point_cloud(pcd)

    o3d.visualization.draw_geometries([chair])
    ######################################################################################



    ######################################################################################
    # Demo Feature Matching Point Clouds
    ######################################################################################
    dataset = o3d.data.DemoFeatureMatchingPointClouds()

    pcd0 = o3d.io.read_point_cloud(dataset.point_cloud_paths[0])
    pcd1 = o3d.io.read_point_cloud(dataset.point_cloud_paths[1])

    fpfh_feature0 = o3d.io.read_feature(dataset.fpfh_feature_paths[0])
    fpfh_feature1 = o3d.io.read_feature(dataset.fpfh_feature_paths[1])

    l32d_feature0 = o3d.io.read_feature(dataset.l32d_feature_paths[0])
    l32d_feature1 = o3d.io.read_feature(dataset.l32d_feature_paths[1])

    o3d.visualization.draw_geometries([pcd0 + pcd1])
    ######################################################################################



    ######################################################################################
    # Demo Pose Graph Optimization <-- 다시....
    ######################################################################################
    dataset = o3d.data.DemoPoseGraphOptimization()
    pose_graph_fragment = o3d.io.read_pose_graph(dataset.pose_graph_fragment_path)
    pose_graph_global = o3d.io.read_pose_graph(dataset.pose_graph_global_path)

    ######################################################################################


    print(f'Done')