#####################################################################
#   04_KDTree.py
#       - This shows how to use KDTree in Open3D
#           . Build KDTree from point cloud
#           . find neighboring points
#               - using search_knn_vector_3d
#               - using search_radius_vector_3d
#       - open3d version : 0.15.1
#   written by jslee
#   date : 2000.00.00
#####################################################################




import numpy as np


import open3d as o3d





if __name__ == '__main__':


    ######################################################################################
    # Build KDTree from point cloud
    ######################################################################################
    print(f'Testing kdtree in Open3D...')
    print(f'Load a point cloud and paint it gray.')

    sample_pcd_data = o3d.data.PCDPointCloud()
    pcd = o3d.io.read_point_cloud(sample_pcd_data.path)
    pcd.paint_uniform_color([0.5, 0.5, 0.5])
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)
    ######################################################################################




    ######################################################################################
    # Find neighboring points
    #   . using search_knn_vector_3d
    #   . using search_radius_vector_3d
    ######################################################################################
    print(f'Paint the 1501st point red.')
    pcd.colors[1500] = [1, 0, 0]


    # using search_knn_vector_3d
    print(f'Find its 200 nearest neighbors, and paint them blue.')
    [k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[1500], 200)
    np.asarray(pcd.colors)[idx[1:], :] = [0, 0, 1]


    print(f'Visualization the point cloud.')
    o3d.visualization.draw_geometries([pcd])



    # using search_radius_vector_3d
    print(f'Find its neighbors with distance less than 0.2, and paint them gree')
    [k, idx, _] = pcd_tree.search_radius_vector_3d(pcd.points[1500], 0.2)
    np.asarray(pcd.colors)[idx[1:], :] = [0, 1, 0]


    print(f'Visualization the point cloud.')
    o3d.visualization.draw_geometries([pcd])
    ######################################################################################


    print('Done')