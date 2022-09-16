#####################################################################
#   01_Visualization.py
#       - This shows how to use Visualization in Open3D
#           . Function draw_geometries
#           . Store view point
#           . Rendering styles
#           . Geometry primitives
#           . Draw line set
#   written by jslee
#   date : 2000.00.00
#####################################################################




import numpy as np


import open3d as o3d


if __name__ == '__main__':


    ######################################################################################
    # Function draw_geometries
    ######################################################################################
    print(f'Load a ply point cloud, print it, and render it')
    sample_ply_data = o3d.data.PLYPointCloud()
    pcd = o3d.io.read_point_cloud(sample_ply_data.path)
    o3d.visualization.draw_geometries([pcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])
    ######################################################################################




    ######################################################################################
    # Store view point
    ######################################################################################

    ######################################################################################




    ######################################################################################
    # Rendering styles
    ######################################################################################

    ######################################################################################




    ######################################################################################
    # Geometry primitives
    ######################################################################################
    print(f"Let's define some primitives")
    # box
    mesh_box = o3d.geometry.TriangleMesh.create_box(width=1.0, height=1.0, depth=1.0)
    mesh_box.compute_vertex_normals()
    mesh_box.paint_uniform_color([0.9, 0.1, 0.1])

    # sphere
    mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1.0)
    mesh_sphere.compute_vertex_normals()
    mesh_sphere.paint_uniform_color([0.1, 0.1, 0.7])

    # cylinder
    mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=0.3, height=4.0)
    mesh_cylinder.compute_vertex_normals()
    mesh_cylinder.paint_uniform_color([0.1, 0.9, 0.1])

    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[-2, -2, -2])


    print(f'We draw a few primitives using collection')
    o3d.visualization.draw_geometries([mesh_box, mesh_sphere, mesh_cylinder, mesh_frame])

    print(f'We draw a few primitives using + operator of mesh')
    o3d.visualization.draw_geometries([mesh_box + mesh_sphere + mesh_cylinder + mesh_frame])
    ######################################################################################




    ######################################################################################
    # Draw line set
    ######################################################################################
    print("Let's draw a box using o3d.geometry.LineSet.")
    points = [
        [0, 0, 0],
        [1, 0, 0],
        [0, 1, 0],
        [1, 1, 0],
        [0, 0, 1],
        [1, 0, 1],
        [0, 1, 1],
        [1, 1, 1],
    ]
    lines = [
        [0, 1],
        [0, 2],
        [1, 3],
        [2, 3],
        [4, 5],
        [4, 6],
        [5, 7],
        [6, 7],
        [0, 4],
        [1, 5],
        [2, 6],
        [3, 7],
    ]
    colors = [[1, 0, 0] for i in range(len(lines))]
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines),
    )
    line_set.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([line_set]) #, zoom=0.8)
    ######################################################################################



    print(f'Done')