#####################################################################
#   02_TriangleMesh.py
#       - This shows how to use TriangleMesh in Open3D
#           . BunnyMesh
#           . ArmadilloMsh
#           . KnotMesh
#   written by jslee
#   date : 2000.00.00
#####################################################################





import numpy as np


import open3d as o3d


if __name__ == '__main__':


    ######################################################################################
    # BunnyMesh
    ######################################################################################
    dataset = o3d.data.BunnyMesh()
    mesh = o3d.io.read_triangle_mesh(dataset.path)
    o3d.visualization.draw_geometries([mesh])
    ######################################################################################



    ######################################################################################
    # ArmadilloMsh
    ######################################################################################
    dataset = o3d.data.ArmadilloMesh()
    mesh = o3d.io.read_triangle_mesh(dataset.path)
    o3d.visualization.draw_geometries([mesh])
    ######################################################################################



    ######################################################################################
    # KnotMesh
    ######################################################################################
    dataset = o3d.data.KnotMesh()
    mesh = o3d.io.read_triangle_mesh(dataset.path)
    o3d.visualization.draw_geometries([mesh])
    ######################################################################################

    print(f'Done')