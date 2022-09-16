#####################################################################
#   15_Python_Interface.py
#       - This shows how to use Python_Interface in Open3D
#           . install Open3D Python package
#           . install Open3D from source
#           . Getting started
#           . Using build-in help function
#               - browse open3d
#               - description of a class in open3d
#               - description of a function in open3d
#       - open3d version : 0.15.1
#   written by jslee
#   date : 2000.00.00
#####################################################################




import numpy as np


import open3d as o3d



if __name__ == '__main__':



    ######################################################################################
    # Getting started
    ######################################################################################
    sample_pcd_data = o3d.data.PCDPointCloud()
    pcd = o3d.io.read_point_cloud(sample_pcd_data.path)
    print(pcd)
    ######################################################################################



    ######################################################################################
    # Using build-in help function
    #   - browse open3d
    #   - description of a class in open3d
    #   - description of a function in open3d
    ######################################################################################
    # browse open3d
    # help(o3d)

    # description of a class in open3d
    # help(o3d.geometry.PointCloud)


    # description of a function in open3d
    # help(o3d.io.read_point_cloud)

    print(f'Done')