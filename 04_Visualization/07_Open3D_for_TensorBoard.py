#####################################################################
#   07_Open3D_for_TensorBoard.py
#       - This shows how to use Open3D_for_TensorBoard in Open3D
#           . Simple geometry sequences
#               - TensorFlow users
#           . Rich 3D models with PBR materials
#           . 3DML models training and inference
#           . Custom properties and semantic segmentation
#           . 3D object detection
#   written by jslee
#   date : 2000.00.00
#####################################################################




import numpy as np


import open3d as o3d

# Monkey-patch torch.utils.tensorboard.SummaryWriter
from open3d.visualization.tensorboard_plugin import summary

# Utility function to convert Open3D geometry to a dictionary format
from open3d.visualization.tensorboard_plugin.util import to_dict_batch
from torch.utils.tensorboard import SummaryWriter

import tensorflow as tf




if __name__ == '__main__':


    ######################################################################################
    # Simple geometry sequences
    #   - TensorFlow users
    ######################################################################################
    cube = o3d.geometry.TriangleMesh.create_box(1, 2, 4)
    cube.compute_vertex_normals()
    cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=1.0, height=2.0, resolution=20, split=4)

    cylinder.compute_vertex_normals()
    colors = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]


    logdir = "demo_logs/pytorch/small_scale"
    writer = SummaryWriter(logdir)
    for step in range(3):
        cube.paint_uniform_color(colors[step])
        writer.add_3d('cube', to_dict_batch([cube]), step=step)
        cylinder.paint_uniform_color(colors[step])
        writer.add_3d('cylinder', to_dict_batch([cylinder]), step=step)



    # TensorFlow users
    # ... geometry creation code as above ...
    logdir = "demo_logs/tf/small_scale"
    writer = tf.summary.create_file_writer(logdir)
    with writer.as_default():
        for step in range(3):
            cube.paint_uniform_color(colors[step])
            summary.add_3d('cube', to_dict_batch([cube]), step=step, logdir=logdir)
            cylinder.paint_uniform_color(colors[step])
            summary.add_3d('cylinder', to_dict_batch([cylinder]), step=step,
                           logdir=logdir)
    ######################################################################################


    print(f'Done')