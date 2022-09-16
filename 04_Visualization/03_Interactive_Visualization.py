#####################################################################
#   03_Interactive_Visualization.py
#       - This shows how to use Interactive_Visualization in Open3D
#           . Crop geometry
#           . Manual registration
#               - Select correspondences
#               - Registration using user correspondences
#   written by jslee
#   date : 2000.00.00
#####################################################################




import numpy as np
import copy

import open3d as o3d



# examples/python/visualization/interactive_visualization.py
def demo_crop_geometry():
    print(f'Demo for manual geometry cropping')
    print(f'1) Press "Y" twice to align geometry with negative direction of y-axis')
    print(f'2) Press "K" to lock screen and to switch to selection mode')
    print(f'3) Drag for rectangle selection, ')
    print(f'   or use ctrl + left click for polygon selection')
    print(f'4) Press "C" to get selected geometry and to save it')
    print(f'5) Press "F" to switch to freeview mode')
    pcd_data = o3d.data.DemoICPPointClouds()
    pcd = o3d.io.read_point_cloud(pcd_data.paths[0])
    o3d.visualization.draw_geometries_with_editing([pcd])



def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.7, 0])
    target_temp.paint_uniform_color([0, 0.6, 0.9])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


def pick_points(pcd):
    print(f'""')
    print(f'1) Please pick at least three correspondences using [shift + left click]')
    print(f'   Press [shift + right click] to undo point picking')
    print(f'2) After picking points, press "Q" to close the window')
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()
    print(f'""')
    return vis.get_picked_points()


def demo_manual_registration():
    print(f'Demo for manual ICP')
    pcd_data = o3d.data.DemoICPPointClouds()
    source = o3d.io.read_point_cloud(pcd_data.paths[0])
    target = o3d.io.read_point_cloud(pcd_data.paths[2])

    print(f'Visualization of two point clouds before manual alignment')
    draw_registration_result(source, target, np.identity(4))

    # pick points from two point clouds and builds correspondences
    picked_id_source = pick_points(source)
    picked_id_target = pick_points(target)
    assert (len(picked_id_source) >= 3 and len(picked_id_target) >= 3)
    assert (len(picked_id_source) == len(picked_id_target))
    corr = np.zeros((len(picked_id_source), 2))
    corr[:, 0] = picked_id_source
    corr[:, 1] = picked_id_target


    # estimate rough transformation using correspondences
    print(f'Compute a rough transform using the correspondences given by user')
    p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    trans_init = p2p.compute_transformation(source, target, o3d.utility.Vector2iVector(corr))

    # point-to-point ICP for refinement
    print(f'Perform point-to-point ICP refinement')
    threshold = 0.03    # 3cm distance threshold
    reg_p2p = o3d.pipelines.registration.registration_icp(source, target, threshold, trans_init,
                                                          o3d.pipelines.registration.TransformationEstimationPointToPoint())
    draw_registration_result(source, target, reg_p2p.transformation)
    print(f'""')







if __name__ == '__main__':


    # examples/python/visualization/interactive_visualization.py
    demo_crop_geometry()
    demo_manual_registration()



    ######################################################################################
    # Crop geometry
    ######################################################################################
    ######################################################################################


    ######################################################################################
    # Manual registration
    #   - Select correspondences
    #   - Registration using user correspondences
    ######################################################################################
    ######################################################################################

    print(f'Done')