#####################################################################
#   09_Surface_Reconstruction.py
#       - This shows how to use Surface_Reconstruction in Open3D
#           . Alpha shapes
#           . Ball pivoting
#           . Poisson surface reconstruction
#           . normal estimation
#       - open3d version : 0.15.1
#   written by jslee
#   date : 2000.00.00
#####################################################################




import numpy as np
import matplotlib.pyplot as plt

import open3d as o3d



if __name__ == '__main__':


    ######################################################################################
    # Alpha shapes
    ######################################################################################
    bunny = o3d.data.BunnyMesh()
    mesh = o3d.io.read_triangle_mesh(bunny.path)
    mesh.compute_vertex_normals()



    pcd = mesh.sample_points_poisson_disk(750)
    o3d.visualization.draw_geometries([pcd])
    alpha = 0.03
    print(f"alpha={alpha:.3f}")
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
    mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)




    tetra_mesh, pt_map = o3d.geometry.TetraMesh.create_from_point_cloud(pcd)
    for alpha in np.logspace(np.log10(0.5), np.log10(0.01), num=4):
        print(f"alpha={alpha:.3f}")
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
            pcd, alpha, tetra_mesh, pt_map)
        mesh.compute_vertex_normals()
        o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
    ######################################################################################



    ######################################################################################
    # Ball pivoting
    ######################################################################################
    bunny = o3d.data.BunnyMesh()
    gt_mesh = o3d.io.read_triangle_mesh(bunny.path)
    gt_mesh.compute_vertex_normals()

    pcd = gt_mesh.sample_points_poisson_disk(3000)
    o3d.visualization.draw_geometries([pcd])



    radii = [0.005, 0.01, 0.02, 0.04]
    rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd, o3d.utility.DoubleVector(radii))
    o3d.visualization.draw_geometries([pcd, rec_mesh])
    ######################################################################################




    ######################################################################################
    # Poisson surface reconstruction
    ######################################################################################
    eagle = o3d.data.EaglePointCloud()
    pcd = o3d.io.read_point_cloud(eagle.path)

    print(pcd)
    o3d.visualization.draw_geometries([pcd],
                                      zoom=0.664,
                                      front=[-0.4761, -0.4698, -0.7434],
                                      lookat=[1.8900, 3.2596, 0.9284],
                                      up=[0.2304, -0.8825, 0.4101])



    print('run Poisson surface reconstruction')
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=9)
    print(mesh)
    o3d.visualization.draw_geometries([mesh],
                                      zoom=0.664,
                                      front=[-0.4761, -0.4698, -0.7434],
                                      lookat=[1.8900, 3.2596, 0.9284],
                                      up=[0.2304, -0.8825, 0.4101])


    print('visualize densities')
    densities = np.asarray(densities)
    density_colors = plt.get_cmap('plasma')(
        (densities - densities.min()) / (densities.max() - densities.min()))
    density_colors = density_colors[:, :3]
    density_mesh = o3d.geometry.TriangleMesh()
    density_mesh.vertices = mesh.vertices
    density_mesh.triangles = mesh.triangles
    density_mesh.triangle_normals = mesh.triangle_normals
    density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)
    o3d.visualization.draw_geometries([density_mesh],
                                      zoom=0.664,
                                      front=[-0.4761, -0.4698, -0.7434],
                                      lookat=[1.8900, 3.2596, 0.9284],
                                      up=[0.2304, -0.8825, 0.4101])



    print('remove low density vertices')
    vertices_to_remove = densities < np.quantile(densities, 0.01)
    mesh.remove_vertices_by_mask(vertices_to_remove)
    print(mesh)
    o3d.visualization.draw_geometries([mesh],
                                      zoom=0.664,
                                      front=[-0.4761, -0.4698, -0.7434],
                                      lookat=[1.8900, 3.2596, 0.9284],
                                      up=[0.2304, -0.8825, 0.4101])
    ######################################################################################




    ######################################################################################
    # normal estimation
    ######################################################################################
    bunny = o3d.data.BunnyMesh()
    gt_mesh = o3d.io.read_triangle_mesh(bunny.path)

    pcd = gt_mesh.sample_points_poisson_disk(5000)
    pcd.normals = o3d.utility.Vector3dVector(np.zeros(
        (1, 3)))  # invalidate existing normals

    pcd.estimate_normals()
    o3d.visualization.draw_geometries([pcd], point_show_normal=True)



    pcd.orient_normals_consistent_tangent_plane(100)
    o3d.visualization.draw_geometries([pcd], point_show_normal=True)
    ######################################################################################





    print('Done')