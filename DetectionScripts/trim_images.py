import os
import cv2
import numpy as np
import pandas as pd

#input_images = '../Example_Images/With_Square'
#output_image_folder = '../Trimmed_Images/With_Square'

input_images = '../Example_Images/Without_Square'
output_image_folder = '../Trimmed_Images/Without_Square'

def trim_img(image_path):
    width = 512 
    img = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if not hasattr(img, 'shape'):
        print("WARNING: Unable to load %s" %image_path)
        return None

    #rescale to 'width' while preserving aspect ratio
    scale = width / img.shape[1]
    height = int(img.shape[0] * scale)

    resized = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)
    image_name = image_path.split('/')[-1]
    image_name = output_image_folder + '/trimmed_' + image_name
    if os.path.isfile(image_name):
        print("NOTE: File %s already exists, deleting." % image_name)
        os.remove(image_name)
    
    cv2.imwrite(image_name,resized)

def trim_images_in_folder(folder_path):
    for image_path in ['/'.join((folder_path, i)) for i in os.listdir(folder_path)]:
        trim_img(image_path)

if __name__ == '__main__':
    trim_images_in_folder(input_images)