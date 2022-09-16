#####################################################################
#   05_File_IO.py
#       - This shows how to use File_IO in Open3D
#           . point cloud
#           . mesh
#           . image
#       - open3d version : 0.15.1
#   written by jslee
#   date : 2000.00.00
#####################################################################




import numpy as np

import open3d as o3d



if __name__ == '__main__':



    ######################################################################################
    # point cloud
    ######################################################################################
    print(f'Testing IO for point cloud...')
    sample_pcd_data = o3d.data.PCDPointCloud()
    pcd = o3d.io.read_point_cloud(sample_pcd_data.path)
    print(pcd)
    # o3d.io.write_point_cloud('copy_of_fragment.pcd', pcd)
    ######################################################################################




    ######################################################################################
    # mesh
    ######################################################################################
    print(f'Testing IO for meshes...')
    knot_data = o3d.data.KnotMesh()
    mesh = o3d.io.read_triangle_mesh(knot_data.path)
    print(mesh)
    o3d.io.write_triangle_mesh('copy_of_knot.ply', mesh)
    ######################################################################################



    ######################################################################################
    # Image
    ######################################################################################
    print(f'Testing IO for images...')
    image_data = o3d.data.JuneauImage()
    img = o3d.io.read_image(image_data.path)
    print(img)
    # o3d.io.write_image('copy_of_Juneau.jpg', image_data)
    ######################################################################################



    print(f'Done')