#####################################################################
#   Voxelization.py
#       - This shows how to use Voxelization in Open3D
#       - 세부 사항 2
#   written by jslee
#   date : 2000.00.00
#####################################################################



import numpy as np

import open3d as o3d




if __name__ == '__main__':


    ######################################################################################
    # Voxelization
    ######################################################################################
    print('input')
    armadillo = o3d.data.ArmadilloMesh()
    mesh = o3d.io.read_triangle_mesh(armadillo.path)
    o3d.visualization.draw_geometries([mesh])


    N = 2000
    pcd = mesh.sample_points_poisson_disk(N)
    o3d.visualization.draw_geometries([pcd])

    # fit to unit cube
    pcd.scale(1 / np.max(pcd.get_max_bound() - pcd.get_min_bound()), center=pcd.get_center())
    pcd.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(N, 3)))
    o3d.visualization.draw_geometries([pcd])

    print(f'test git')

