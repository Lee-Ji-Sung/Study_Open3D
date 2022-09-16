#####################################################################
#   04_RGBDImage.py
#       - This shows how to use RGBDImage in Open3D
#           . SampleRedwoodRGBDImages
#           . SampleFountainRGBDImages
#           . SampleNYURGBDImages
#           . SampleSUNRGBDImages
#           . SampleTUMRGBDImages
#   written by jslee
#   date : 2000.00.00
#####################################################################





import numpy as np
import matplotlib.image as mpimg
import re


import open3d as o3d



# SampleNYURGBDImages
def read_nyu_pgm(filename, byteorder='>'):
    with open(filename, 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()

    except AttributeError:
        raise ValueError('Not a raw PGM file : "%s" ' % filename )

    img = np.frombuffer(buffer,
                        dtype=byteorder + 'u2',
                        count=int(width) * int(height),
                        offset=len(header)).reshape((int(height), int(width)))

    img_out = img.astype('u2')
    return img_out





if __name__ == '__main__':


    ######################################################################################
    # SampleRedwoodRGBDImages - 5 images & 5 depth image
    ######################################################################################
    dataset = o3d.data.SampleRedwoodRGBDImages()

    rgbd_images = []
    for i in range(len(dataset.depth_paths)):
        color_raw = o3d.io.read_image(dataset.color_paths[i])
        depth_raw = o3d.io.read_image(dataset.depth_paths[i])
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw)
        rgbd_images.append(rgbd_image)


    pcd = o3d.io.read_point_cloud(dataset.reconstruction_path)
    o3d.visualization.draw_geometries([pcd])
    ######################################################################################



    ######################################################################################
    # SampleFountainRGBDImages - 33 images & 33 depth image
    ######################################################################################
    """
    datasset = o3d.data.SampleFountainRGBDImages()


    rgbd_images = []
    for i in range(len(dataset.depth_paths)):
        depth = o3d.io.read_image(dataset.depth_paths[i])
        color = o3d.io.read_image(dataset.color_paths[i])
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, convert_rgb_to_intensity=False)
        rgbd_images.append(rgbd_image)

    # camera_trajectory = o3d.io.read_pinhole_camera_trajectory(dataset.keyframe_poses_log_path)

    mesh = o3d.io.read_triangle_mesh(dataset.reconstruction_path)

    o3d.visualization.draw_geometries([mesh])
    """
    ######################################################################################



    ######################################################################################
    # SampleNYURGBDImages
    ######################################################################################
    dataset = o3d.data.SampleNYURGBDImage()
    color_raw = mpimg.imread(dataset.color_path)
    depth_raw = read_nyu_pgm(dataset.depth_path)
    color = o3d.geometry.Image(color_raw)
    depth = o3d.geometry.Image(depth_raw)
    rgbd_image = o3d.geometry.RGBDImage.create_from_nyu_format(color, depth, convert_rgb_to_intensity=False)

    o3d.visualization.draw_geometries([rgbd_image])
    ######################################################################################



    ######################################################################################
    # SampleSUNRGBDImages
    ######################################################################################
    dataset = o3d.data.SampleSUNRGBDImage()
    color_raw = o3d.io.read_image(dataset.color_path)
    depth_raw = o3d.io.read_image(dataset.depth_path)
    rgbd_image = o3d.geometry.RGBDImage.create_from_sun_format(color_raw, depth_raw, convert_rgb_to_intensity=False)

    o3d.visualization.draw_geometries([rgbd_image])
    ######################################################################################



    ######################################################################################
    # SampleTUMRGBDImages
    ######################################################################################
    dataset = o3d.data.SampleTUMRGBDImage()
    color_raw = o3d.io.read_image(dataset.color_path)
    depth_raw = o3d.io.read_image(dataset.depth_path)
    rgbd_image = o3d.geometry.RGBDImage.create_from_tum_format(color_raw, depth_raw, convert_rgb_to_intensity=False)

    o3d.visualization.draw_geometries([rgbd_image])
    ######################################################################################


    print(f'Done')