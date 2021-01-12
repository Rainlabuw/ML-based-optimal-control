# -*- coding: utf-8 -*-
"""
Created on Tue Nov 25 16:43:11 2020

@author: Niyousha Rahimi
"""

import os
import sys
import random
import math
import numpy as np
import skimage.io
import matplotlib
import matplotlib.pyplot as plt


# Root directory of the project
ROOT_DIR = os.path.abspath("C:/Users/newsh/Desktop/Motion_lanning/semantic_segmentation/Mask_RCNN-master")

# Import Mask RCNN
sys.path.append(ROOT_DIR)  # To find local version of the library
from mrcnn import utils
import mrcnn.model as modellib
from mrcnn import image_classification
# Import COCO config
sys.path.append(os.path.join(ROOT_DIR, "samples/coco/"))  # To find local version
import coco


''' ###################################'''

class InferenceConfig(coco.CocoConfig):
    # Set batch size to 1 since we'll be running inference on
    # one image at a time. Batch size = GPU_COUNT * IMAGES_PER_GPU
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1

class Mask_RCNN:
    
    def __init__ (self):
        # Directory to save logs and trained model
        MODEL_DIR = os.path.join(ROOT_DIR, "logs")
        
        # Local path to trained weights file
        COCO_MODEL_PATH = os.path.join(ROOT_DIR, "mask_rcnn_coco.h5")
        # Download COCO trained weights from Releases if needed
        if not os.path.exists(COCO_MODEL_PATH):
            utils.download_trained_weights(COCO_MODEL_PATH)
        
       
    
    
        
        config = InferenceConfig()
        # config.display()
        
        
        
        
        ''' ############## Create Model and Load Trained Weights #####################'''
        # Create model object in inference mode.
        self.model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR, config=config)
        
        # Load weights trained on MS-COCO
        self.model.load_weights(COCO_MODEL_PATH, by_name=True)
        
        
        ''' ###################################'''
        # COCO Class names
        # Index of the class in the list is its ID. For example, to get ID of
        # the teddy bear class, use: class_names.index('teddy bear')
        self.class_names = ['BG', 'person', 'bicycle', 'car', 'motorcycle', 'airplane',
                        'bus', 'train', 'truck', 'boat', 'traffic light',
                        'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird',
                        'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear',
                        'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie',
                        'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
                        'kite', 'baseball bat', 'baseball glove', 'skateboard',
                        'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
                        'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
                        'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
                        'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed',
                        'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote',
                        'keyboard', 'cell phone', 'microwave', 'oven', 'toaster',
                        'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors',
                        'teddy bear', 'hair drier', 'toothbrush']
        # self.class_names = ['BG', 'person', 'car', 'airplane',
        #                 'truck',  'traffic light']
        
    def object_pose_estimate(self, got_depth, object_semantics):
        """
        description
        
        """
        
        # getting rid of nan depth
        object_pos_points = got_depth[object_semantics[0], object_semantics[1], :]
        indice_ = [s for s in object_pos_points if s[0] != 0 ] #np.isnan
        
        obj_pose = np.sum(indice_,0)/(np.shape(indice_)[0])
        
        return obj_pose
    
    
    def point_cloud(self, depth, cam_pose):
        """Transform a depth image into a point cloud with one point for each
        pixel in the image, using the camera transform for a camera
        centred at cx, cy with field of view fx, fy.
    
        depth is a 2-D ndarray with shape (rows, cols) containing
        depths from 1 to 254 inclusive. The result is a 3-D array with
        shape (rows, cols, 3). Pixels with invalid depth in the input have
        NaN for the z-coordinate in the result.
    
        """
        
        Width=256
        Height=144
        factor = 2.0 #* np.tan(90/2.0)
        focal_length=Width/2
        rows, cols = depth.shape
        ratio = max(rows,cols)
        print(rows, cols)
        c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
        valid = (depth > 0) & (depth < 255)
        z = np.where(valid, depth / 256.0, np.nan)
        x = np.where(valid, cam_pose[0] - 100 * factor * z * (c - (cols / 2)) / ratio , 0) #focal_length
        y = np.where(valid, cam_pose[1] - 100 * factor * z * (r - (rows / 2)) / ratio  , 0)
        
        return np.dstack((x, y, z))

    
    def Object_detection(self, IMAGE_DIR):
        
        
        
        ''' ############# Run Object Detection ######################'''
        
        image = IMAGE_DIR#skimage.io.imread(IMAGE_DIR)
        
        results = self.model.detect([image], verbose=1)
        
        # Visualize results
        r = results[0]
        
        
        # image_classification.display_instances(image, r['rois'], r['masks'], r['class_ids'], 
        #                             self.class_names, r['scores'])
        
        oo = image_classification.identify_object(image, r['rois'], r['masks'], r['class_ids'], 
                                    self.class_names, r['scores'])
        # print(np.shape(oo))
        if not oo:
            return []
        else:
            return oo
        










