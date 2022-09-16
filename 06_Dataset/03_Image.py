#####################################################################
#   03_Image.py
#       - This shows how to use Image in Open3D
#           . JuneauImage
#   written by jslee
#   date : 2000.00.00
#####################################################################





import numpy as np
import matplotlib.pyplot as plt

import open3d as o3d


if __name__ == '__main__':


    ######################################################################################
    # JuneauImage
    ######################################################################################
    img_data = o3d.data.JuneauImage()
    img = o3d.io.read_image(img_data.path)

    # plot
    plt.figure()
    plt.imshow(img)
    plt.show()
    ######################################################################################


    print(f'Done')