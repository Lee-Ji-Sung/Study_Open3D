#####################################################################
#   06_Point_Cloud_Outlier_Removal.py
#       - This shows how to use Point_Cloud_Outlier_Removal in Open3D
#           . prepare input data
#           . select down sample
#           . statistical outlier removal
#           . radius outlier removal
#       - open3d version : 0.15.1
#   written by jslee
#   date : 2000.00.00
#####################################################################




import numpy as np


import open3d as o3d



def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print(f'Showing outlier (red) and inliers (gray):')
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
                                      zoom=0.3412,
                                      front=[0.4257, -0.2125, -0.8795],
                                      lookat=[2.6172, 2.0475, 1.532],
                                      up=[-0.0694, -0.9768, 0.2024])



if __name__ == '__main__':



    ######################################################################################
    # prepare input data
    ######################################################################################
    print(f'Load a ply point cloud, print it, and render it')
    sample_pcd_data = o3d.data.PCDPointCloud()
    pcd = o3d.io.read_point_cloud(sample_pcd_data.path)
    o3d.visualization.draw_geometries([pcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])



    print(f'Downsample the point cloud with a voxel of 0.02')
    voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.02)
    o3d.visualization.draw_geometries([voxel_down_pcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])


    # uniform_down_sample
    print(f'Every 5th points are selected')
    uni_down_pcd = pcd.uniform_down_sample(every_k_points=5)
    o3d.visualization.draw_geometries([uni_down_pcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])
    ######################################################################################





    ######################################################################################
    # select down sample
    ######################################################################################

    # def display_inlier_outlier(cloud, ind):

    ######################################################################################




    ######################################################################################
    # statistical outlier removal
    ######################################################################################
    print(f'Statistical outlier removal')
    cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    display_inlier_outlier(voxel_down_pcd, ind)
    ######################################################################################





    ######################################################################################
    # radius outlier removal
    ######################################################################################
    print(f'Radius oulier removal')
    cl, ind = voxel_down_pcd.remove_radius_outlier(nb_points=16, radius=0.05)
    display_inlier_outlier(voxel_down_pcd, ind)
    ######################################################################################



    print('Done')