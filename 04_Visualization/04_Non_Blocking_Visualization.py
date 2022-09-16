#####################################################################
#   04_Non_Blocking_Visualization.py
#       - This shows how to use Non_Blocking_Visualization in Open3D
#           . /example/python/visualization/non_blocking_visualization.py
#           . Review draw_geometries
#           . Prepare example data
#           . Initialize Visualize class
#           . Transform geometry and visualize it
#   written by jslee
#   date : 2000.00.00
#####################################################################




import numpy as np


import open3d as o3d



if __name__ == '__main__':


    ######################################################################################
    # Non-blocking visualization
    ######################################################################################
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

    # Prepare example data
    pcd_data = o3d.data.DemoICPPointClouds()
    source_raw = o3d.io.read_point_cloud(pcd_data.paths[0])
    target_raw = o3d.io.read_point_cloud(pcd_data.paths[1])

    source = source_raw.voxel_down_sample(voxel_size=0.02)
    target = target_raw.voxel_down_sample(voxel_size=0.02)
    trans = [[0.862, 0.011, -0.507, 0.0], [-0.139, 0.967, -0.215, 0.7],
             [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]]
    source.transform(trans)

    flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
    source.transform(flip_transform)
    target.transform(flip_transform)


    # Initialize Visualize class
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(source)
    vis.add_geometry(target)
    threshold = 0.05
    icp_iteration = 100
    save_image = False

    for i in range(icp_iteration):
        # Transform geometry and visualize it
        reg_p21 = o3d.pipelines.registration.registration_icp(source, target, threshold, np.identity(4),
                                                              o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                                                              o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1))
        source.transform(reg_p21.transformation)
        vis.update_geometry(source)
        vis.poll_events()
        vis.update_renderer()
        if save_image:
            vis.capture_screen_image("temp_%04d.jpg" % i)
        # vis.destroy_window()
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Info)
    ######################################################################################






    print(f'Done')