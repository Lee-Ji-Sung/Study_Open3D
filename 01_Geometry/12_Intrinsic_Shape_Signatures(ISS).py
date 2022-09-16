#####################################################################
#   12_Intrinsic_Shape_Signatures(ISS).py
#       - This shows how to use Intrinsic_Shape_Signatures(ISS) in Open3D
#           . ISS Keypoints --> paper :"Performance Evaluation of 3D Keypoint Detectors"
#           . ISS Keypoint detection example
#       - open3d version : 0.15.1
#   written by jslee
#   date : 2000.00.00
#####################################################################




import numpy as np
import time

import open3d as o3d


# This function is only used to make the keypoints look better on the rendering
def keypoints_to_spheres(keypoints):
    spheres = o3d.geometry.TriangleMesh()
    for keypoint in keypoints.points:
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.001)
        sphere.translate(keypoint)
        spheres += sphere
    spheres.paint_uniform_color([1.0, 0.75, 0.0])
    return spheres



if __name__ == '__main__':



    ######################################################################################
    # ISS Keypoint detection example
    ######################################################################################
    # Compute ISS Keypoints on ArmadilloMesh
    armadillo = o3d.data.ArmadilloMesh()
    mesh = o3d.io.read_triangle_mesh(armadillo.path)
    mesh.compute_vertex_normals()

    pcd = o3d.geometry.PointCloud()
    pcd.points = mesh.vertices

    tic = time.time()
    keypoints = o3d.geometry.keypoint.compute_iss_keypoints(pcd)
    toc = 1000 * (time.time() - tic)
    print("ISS Computation took {:.0f} [ms]".format(toc))

    mesh.compute_vertex_normals()
    mesh.paint_uniform_color([0.5, 0.5, 0.5])
    keypoints.paint_uniform_color([1.0, 0.75, 0.0])
    o3d.visualization.draw_geometries([keypoints, mesh]) #, front=[0, 0, -1.0])





    # Compute ISS Keypoints on Standford BunnyMesh, changing the default parameters
    bunny = o3d.data.BunnyMesh()
    mesh = o3d.io.read_triangle_mesh(bunny.path)
    mesh.compute_vertex_normals()

    pcd = o3d.geometry.PointCloud()
    pcd.points = mesh.vertices

    tic = time.time()
    keypoints = o3d.geometry.keypoint.compute_iss_keypoints(pcd,
                                                            salient_radius=0.005,
                                                            non_max_radius=0.005,
                                                            gamma_21=0.5,
                                                            gamma_32=0.5)
    toc = 1000 * (time.time() - tic)
    print("ISS Computation took {:.0f} [ms]".format(toc))

    mesh.compute_vertex_normals()
    mesh.paint_uniform_color([0.5, 0.5, 0.5])
    o3d.visualization.draw_geometries([keypoints_to_spheres(keypoints), mesh])
    ######################################################################################


    print(f'Done')