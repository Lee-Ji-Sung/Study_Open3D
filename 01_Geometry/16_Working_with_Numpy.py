#####################################################################
#   16_Working_with_Numpy.py
#       - This shows how to use Working_with_Numpy in Open3D
#           . From Numpy to open3d.PointCloud
#           . From open3d.PointCloud to Numpy
#       - open3d version : 0.15.1
#   written by jslee
#   date : 2000.00.00
#####################################################################




import numpy as np


import open3d as o3d



if __name__ == '__main__':


    # Generate some neat n times 3 matrix using a variant of sync function
    x = np.linspace(-3, 3, 401)
    mesh_x, mesh_y = np.meshgrid(x, x)
    z = np.sinc((np.power(mesh_x, 2) + np.power(mesh_y, 2)))
    z_norm = (z - z.min()) / (z.max() - z.min())
    xyz = np.zeros((np.size(mesh_x), 3))
    xyz[:, 0] = np.reshape(mesh_x, -1)
    xyz[:, 1] = np.reshape(mesh_y, -1)
    xyz[:, 2] = np.reshape(z_norm, -1)
    print('xyz')
    print(xyz)



    ######################################################################################
    # From Numpy to open3d.PointCloud
    ######################################################################################
    # Pass xyz to Open3D.o3d.geometry.PointCloud and visualize
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    o3d.io.write_point_cloud("../../test_data/sync.ply", pcd)
    ######################################################################################


    ######################################################################################
    # From open3d.PointCloud to Numpy
    ######################################################################################
    # Load saved point cloud and visualize it
    pcd_load = o3d.io.read_point_cloud("../../test_data/sync.ply")

    # Convert Open3D.o3d.geometry.PointCloud to numpy array
    xyz_load = np.asarray(pcd_load.points)
    print('xyz_load')
    print(xyz_load)
    o3d.visualization.draw_geometries([pcd_load])
    ######################################################################################
    print(f'Done')