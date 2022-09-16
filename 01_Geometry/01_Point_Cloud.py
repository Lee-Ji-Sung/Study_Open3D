#####################################################################
#   01_Point_Cloud.py
#       - This shows how to use Geometry in Open3D
#           . visualize point cloud
#           . voxel downsampling
#           . vertex normal estimation
#           . access estimated vertex normal
#           . crop point cloud
#           . paint point cloud
#           . point cloud distance
#           . bounding volumes
#           . convex hull
#           . DBSCAN clustering
#           . plane segmentation
#           . hidden point removal
#       - open3d version : 0.15.1
#   written by jslee
#   date : 2000.00.00
#####################################################################




import numpy as np
import matplotlib.pyplot as plt

import open3d as o3d



if __name__ == '__main__':


    """
    ######################################################################################
    # open3d version check
    ######################################################################################
    print(o3d.__version__)
    ######################################################################################


    ######################################################################################
    # visualize point cloud
    ######################################################################################
    print(f'Load a ply point cloud, print it, and render it')
    ply_point_cloud = o3d.data.PLYPointCloud()
    pcd = o3d.io.read_point_cloud(ply_point_cloud.path)
    print(f'pcd : {pcd}')
    print(f'numpy pcd : {np.asarray(pcd.points)}')
    o3d.visualization.draw_geometries([pcd], zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])
    ######################################################################################


    ######################################################################################
    # voxel downsampling
    ######################################################################################
    print(f'Downsample the point cloud with a voxel of 0.05')
    downpcd = pcd.voxel_down_sample(voxel_size=0.05)
    print(f'downpcd : {downpcd}')
    o3d.visualization.draw_geometries([downpcd], zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])
    ######################################################################################


    ######################################################################################
    # vertex normal estimation
    ######################################################################################
    print(f'Recompute the normal of the dwonsampled point cloud')
    downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    o3d.visualization.draw_geometries([downpcd], zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024],
                                  point_show_normal=True)
    ######################################################################################



    ######################################################################################
    # crop point cloud
    ######################################################################################
    print(f'Load a polygon volume and use it to crop the original point cloud')
    demo_crop_data = o3d.data.DemoCropPointCloud()
    pcd = o3d.io.read_point_cloud(demo_crop_data.point_cloud_path)
    vol = o3d.visualization.read_selection_polygon_volume(demo_crop_data.cropped_json_path)
    chair = vol.crop_point_cloud(pcd)
    o3d.visualization.draw_geometries([chair],
                                  zoom=0.7,
                                  front=[0.5439, -0.2333, -0.8060],
                                  lookat=[2.4615, 2.1331, 1.338],
                                  up=[-0.1781, -0.9708, 0.1608])
    ######################################################################################



    ######################################################################################
    # paint point cloud
    ######################################################################################
    print(f'Paint chair')
    chair.paint_uniform_color([1, 0.706, 0])
    o3d.visualization.draw_geometries([chair],zoom=0.7,
                                  front=[0.5439, -0.2333, -0.8060],
                                  lookat=[2.4615, 2.1331, 1.338],
                                  up=[-0.1781, -0.9708, 0.1608])
    ######################################################################################



    ######################################################################################
    # point cloud distance
    ######################################################################################
    # load data
    demo_crop_data = o3d.data.DemoCropPointCloud()
    pcd = o3d.io.read_point_cloud(demo_crop_data.point_cloud_path)
    vol = o3d.visualization.read_selection_polygon_volume(demo_crop_data.cropped_json_path)
    chair = vol.crop_point_cloud(pcd)

    dists = pcd.compute_point_cloud_distance(chair)
    dists = np.asarray(dists)
    ind = np.where(dists > 0.01)[0]
    pcd_without_chair = pcd.select_by_index(ind)
    o3d.visualization.draw_geometries([pcd_without_chair],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])
    ######################################################################################


    ######################################################################################
    # bounding volumes
    ######################################################################################
    aabb = chair.get_axis_aligned_bounding_box()
    aabb.color = (1, 0, 0)
    obb = chair.get_oriented_bounding_box()
    obb.color = (0, 1, 0)
    o3d.visualization.draw_geometries([chair, aabb, obb],
                                  zoom=0.7,
                                  front=[0.5439, -0.2333, -0.8060],
                                  lookat=[2.4615, 2.1331, 1.338],
                                  up=[-0.1781, -0.9708, 0.1608])
    ######################################################################################



    ######################################################################################
    # convex hull
    ######################################################################################
    bunny = o3d.data.BunnyMesh()
    mesh = o3d.io.read_triangle_mesh(bunny.path)
    mesh.compute_vertex_normals()

    pcl = mesh.sample_points_poisson_disk(number_of_points=2000)
    hull, _ = pcl.compute_convex_hull()
    hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
    hull_ls.paint_uniform_color((1, 0, 0))
    o3d.visualization.draw_geometries([pcl, hull_ls])
    ######################################################################################


    ######################################################################################
    # DBSCAN clustering
    ######################################################################################
    ply_point_cloud = o3d.data.PLYPointCloud()
    pcd = o3d.io.read_point_cloud(ply_point_cloud.path)

    with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

    max_label = labels.max()
    print(f'point cloud has {max_label + 1} clusters')
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    o3d.visualization.draw_geometries([pcd],
                                  zoom=0.455,
                                  front=[-0.4999, -0.1659, -0.8499],
                                  lookat=[2.1813, 2.0619, 2.0999],
                                  up=[0.1204, -0.9852, 0.1215])
    ######################################################################################


    ######################################################################################
    # plane segmentation
    ######################################################################################
    pcd_point_cloud = o3d.data.PCDPointCloud()
    pcd = o3d.io.read_point_cloud(pcd_point_cloud.path)

    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    [a, b, c, d] = plane_model
    print(f'plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0')
    inlier_cloud  = pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1., 0, 0])
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
                                  zoom=0.8,
                                  front=[-0.4999, -0.1659, -0.8499],
                                  lookat=[2.1813, 2.0619, 2.0999],
                                  up=[0.1204, -0.9852, 0.1215])
    ######################################################################################

    """

    ######################################################################################
    # hidden point removal
    ######################################################################################
    print(f'Convert mesh to a point cloud and estimate dimensions')
    armadillo = o3d.data.ArmadilloMesh()
    mesh = o3d.io.read_triangle_mesh(armadillo.path)
    mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh])

    pcd = mesh.sample_points_poisson_disk(5000)
    diameter = np.linalg.norm(np.asarray(pcd.get_max_bound()) - np.asarray(pcd.get_min_bound()))
    o3d.visualization.draw_geometries([pcd])

    print(f'Define parameters used for hidden_point_removal')
    camera = [0, 0, diameter]
    radius = diameter * 100

    print(f'Get all points that are visible from given view point')
    _, pt_map = pcd.hidden_point_removal(camera, radius)

    print(f'visualize result')
    pcd = pcd.select_by_index(pt_map)
    o3d.visualization.draw_geometries([pcd])
    ######################################################################################



    print('Done')