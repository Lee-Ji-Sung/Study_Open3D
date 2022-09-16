#####################################################################
#####################################################################
#   02_Customized_Visualization.py
#       - This shows how to use Customized_Visualization in Open3D
#           . Mimic draw_geometries() with Visualizer class
#           . Change field of view
#           . Callback function
#           . Capture images in a customized animation
#   written by jslee
#   date : 2000.00.00
#####################################################################




import numpy as np
import matplotlib.pyplot as plt
import os

import open3d as o3d



# Callback functions
def rotate_view(vis):
    ctr = vis.get_view_control()
    ctr.rotate(10.0, 0.0)
    return False


"""
def custom_draw_geometry_with_key_callback(pcd):

    def change_background_to_black(vis):
        opt = vis.get_render_option()
        opt.background_color = np.asarray([0, 0, 0])
        return False

    def load_render_option(vis):
        vis.get_render_option().load_from_json(
            os.path.join(test_data_path, 'renderoption.json'))
        return False

    def capture_depth(vis):
        depth = vis.capture_depth_float_buffer()
        plt.imshow(np.asarray(depth))
        plt.show()
        return False

    def capture_image(vis):
        image = vis.capture_screen_float_buffer()
        plt.imshow(np.asarray(image))
        plt.show()
        return False

    key_to_callback = {}
    key_to_callback[ord("K")] = change_background_to_black
    key_to_callback[ord("R")] = load_render_option
    key_to_callback[ord(",")] = capture_depth
    key_to_callback[ord(".")] = capture_image
    o3d.visualization.draw_geometries_with_key_callbacks([pcd], key_to_callback)
"""


"""
# Capture images in a customized animation
def custom_draw_geometry_with_camera_trajectory(pcd):
    custom_draw_geometry_with_camera_trajectory.index = -1
    custom_draw_geometry_with_camera_trajectory.trajectory =\
            o3d.io.read_pinhole_camera_trajectory(
                os.path.join(test_data_path, 'camera_trajectory.json'))
    custom_draw_geometry_with_camera_trajectory.vis = o3d.visualization.Visualizer(
    )
    image_path = os.path.join(test_data_path, 'image')
    if not os.path.exists(image_path):
        os.makedirs(image_path)
    depth_path = os.path.join(test_data_path, 'depth')
    if not os.path.exists(depth_path):
        os.makedirs(depth_path)
    render_option_path = os.path.join(test_data_path, 'renderoption.json')

    def move_forward(vis):
        # This function is called within the o3d.visualization.Visualizer::run() loop
        # The run loop calls the function, then re-render
        # So the sequence in this function is to:
        # 1. Capture frame
        # 2. index++, check ending criteria
        # 3. Set camera
        # 4. (Re-render)
        ctr = vis.get_view_control()
        glb = custom_draw_geometry_with_camera_trajectory
        if glb.index >= 0:
            print("Capture image {:05d}".format(glb.index))
            depth = vis.capture_depth_float_buffer(False)
            image = vis.capture_screen_float_buffer(False)
            plt.imsave(os.path.join(depth_path, '{:05d}.png'.format(glb.index)),\
                    np.asarray(depth), dpi = 1)
            plt.imsave(os.path.join(image_path, '{:05d}.png'.format(glb.index)),\
                    np.asarray(image), dpi = 1)
            # vis.capture_depth_image("depth/{:05d}.png".format(glb.index), False)
            # vis.capture_screen_image("image/{:05d}.png".format(glb.index), False)
        glb.index = glb.index + 1
        if glb.index < len(glb.trajectory.parameters):
            ctr.convert_from_pinhole_camera_parameters(
                glb.trajectory.parameters[glb.index], allow_arbitrary=True)
        else:
            custom_draw_geometry_with_camera_trajectory.vis.\
                    register_animation_callback(None)
        return False
"""




if __name__ == '__main__':


    ######################################################################################
    # Mimic draw_geometries() with Visualizer class
    ######################################################################################
    # The following code achieves the same effect as :
    # o3d.visualization.draw_geometries([pcd])
    sample_ply_data = o3d.data.PLYPointCloud()
    pcd = o3d.io.read_point_cloud(sample_ply_data.path)

    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()


    # vis = o3d.visualization.Visualizer()
    # vis.create_window()
    # vis.add_geometry(pcd)
    # vis.get_render_option().load_from_json(os.path.join(test_data_path, 'renderoption.json'))
    # vis.run()
    # vis.destroy_window()
    ######################################################################################



    ######################################################################################
    # Change field of view
    ######################################################################################
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    ctr = vis.get_view_control()
    print('Field of view (before change) %.2f' % ctr.get_field_of_view())
    # ctr.change_field_of_view(step=fov_step)

    print('Field of view (after changing %.2f' % ctr.get_field_of_view())
    vis.run()
    vis.destroy_window()

    # custom_draw_geometry_with_custom_fov(pcd, 90.0)
    # custom_draw_geometry_with_custom_fov(pcd, -90.0)
    ######################################################################################




    ######################################################################################
    # Callback function
    ######################################################################################
    o3d.visualization.draw_geometries_with_animation_callback([pcd], rotate_view)
    ######################################################################################

    ######################################################################################
    # Capture images in a customized animation
    ######################################################################################
    # vis = custom_draw_geometry_with_camera_trajectory.vis
    # vis.create_window()
    # vis.add_geometry(pcd)
    # vis.get_render_option().load_from_json(render_option_path)
    # vis.register_animation_callback(move_forward)
    # vis.run()
    # vis.destroy_window()
    ######################################################################################

    print(f'Done')