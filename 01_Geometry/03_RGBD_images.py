#####################################################################
#   03_RGBD_images.py
#       - This shows how to use RGBD images in Open3D
#           . Redwood dataset
#           . SUN dataset
#           . NYU dataset
#           . TUM dataset
#       - open3d version : 0.15.1
#   written by jslee
#   date : 2000.00.00
#####################################################################




import numpy as np
import matplotlib.pyplot as plt

import matplotlib.image as mpimg
import re

import open3d as o3d



# This is special function used for reading NYU pgm format
# as it is written in big endian byte order.
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
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    img = np.frombuffer(buffer,
                        dtype=byteorder + 'u2',
                        count=int(width) * int(height),
                        offset=len(header)).reshape((int(height), int(width)))
    img_out = img.astype('u2')
    return img_out




if __name__ == '__main__':


    """
    ######################################################################################
    # Redwood dataset
    ######################################################################################
    print(f'Read Redwood datasset')
    redwood_rgbd = o3d.data.SampleRedwoodRGBDImages()
    color_raw = o3d.io.read_image(redwood_rgbd.color_paths[0])
    depth_raw = o3d.io.read_image(redwood_rgbd.depth_paths[0])
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw)
    print(rgbd_image)


    plt.subplot(1, 2, 1)
    plt.title('Redwood grayscale image')
    plt.imshow(rgbd_image.color)
    plt.subplot(1, 2, 2)
    plt.title('Redwood depth image')
    plt.imshow(rgbd_image.depth)
    plt.show()


    # point cloud
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, o3d.camera.PinholeCameraIntrinsic(
        o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    # flip it otherwise the point cloud will be upside down
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    o3d.visualization.draw_geometries([pcd])
    ######################################################################################
    """


    """
    ######################################################################################
    # SUN dataset
    ######################################################################################
    print(f'Read SUN dataset')
    sun_rgbd = o3d.data.SampleSUNRGBDImage()
    color_raw = o3d.io.read_image(sun_rgbd.color_path)
    depth_raw = o3d.io.read_image(sun_rgbd.depth_path)
    rgbd_image = o3d.geometry.RGBDImage.create_from_sun_format(color_raw, depth_raw)
    print(rgbd_image)


    plt.subplot(1, 2, 1)
    plt.title('SUN grayscale image')
    plt.imshow(rgbd_image.color)
    plt.subplot(1, 2, 2)
    plt.title('SUN depth image')
    plt.imshow(rgbd_image.depth)
    plt.show()


    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, o3d.camera.PinholeCameraIntrinsic(
        o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    # flip it, otherwise the pointcloud will be upside down
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    o3d.visualization.draw_geometries([pcd])
    ######################################################################################
    """


    """
    ######################################################################################
    # NYU dataset
    ######################################################################################
    print(f'Read NYU dataset')
    # Open3D does not support ppm/pgm file yet. Not using o3d.io.read_image here.
    # MathplotImage having some ISSUE with NYU pgm file. Not using imread for pgm.
    nyu_rgbd = o3d.data.SampleNYURGBDImage()
    color_raw = mpimg.imread(nyu_rgbd.color_path)
    depth_raw = read_nyu_pgm(nyu_rgbd.depth_path)
    color = o3d.geometry.Image(color_raw)
    depth = o3d.geometry.Image(depth_raw)
    rgbd_image = o3d.geometry.RGBDImage.create_from_nyu_format(color, depth)
    print(rgbd_image)

    plt.subplot(1, 2, 1)
    plt.title('NYU grayscale image')
    plt.imshow(rgbd_image.color)
    plt.subplot(1, 2, 2)
    plt.title('NYU depth image')
    plt.imshow(rgbd_image.depth)
    plt.show()



    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image,
        o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    # Flip it, otherwise the pointcloud will be upside down
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    o3d.visualization.draw_geometries([pcd])
    ######################################################################################
    """



    ######################################################################################
    # TUM dataset
    ######################################################################################
    print("Read TUM dataset")
    tum_rgbd = o3d.data.SampleSUNRGBDImage()
    color_raw = o3d.io.read_image(tum_rgbd.color_path)
    depth_raw = o3d.io.read_image(tum_rgbd.depth_path)
    rgbd_image = o3d.geometry.RGBDImage.create_from_tum_format(color_raw, depth_raw)
    print(rgbd_image)



    plt.subplot(1, 2, 1)
    plt.title('TUM grayscale image')
    plt.imshow(rgbd_image.color)
    plt.subplot(1, 2, 2)
    plt.title('TUM depth image')
    plt.imshow(rgbd_image.depth)
    plt.show()




    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image,
        o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    # Flip it, otherwise the pointcloud will be upside down
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    o3d.visualization.draw_geometries([pcd])
    ######################################################################################

    print('Done')