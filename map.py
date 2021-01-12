# -*- coding: utf-8 -*-
"""
Created on Tue Oct  6 19:58:54 2020

@author: niyousha
"""



import airsim
import cv2
import numpy as np
import os

import time
import numpy as np
from matplotlib import pyplot as plt
from PIL import Image
import imageio
import pprint

pp = pprint.PrettyPrinter(indent=4)



## updating the map for known buildings
road_map = np.ones([300,350])*255
road_map[0:50,0:250]=0
road_map[250:300,50:250]=0



# connect to the AirSim simulator 
client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = airsim.CarControls()

unreal_origin = [client.simGetObjectPose('SM_TerminalBlockout_18').position.x_val, client.simGetObjectPose('SM_TerminalBlockout_18').position.y_val]


## updating the map for SM_TerminalBlockouts
object_list = ['SM_TerminalBlockout_18','SM_TerminalBlockout3','SM_TerminalBlockout2','SM_TerminalBlockout10','SM_TerminalBlockout7','SM_TerminalBlockout8','SM_TerminalBlockout9']

for item in object_list:
    object_pose = [(client.simGetObjectPose(item).position.x_val-unreal_origin[0])*(-1)+75, (client.simGetObjectPose(item).position.y_val-unreal_origin[1])+125]
    if client.simGetObjectPose(item).orientation.z_val + client.simGetObjectPose('SM_TerminalBlockout_18').orientation.z_val !=0:
        road_map[int(object_pose[0])-25:int(object_pose[0])+1+3, int(object_pose[1])-7:int(object_pose[1])+1+7]=0
    else:
        road_map[int(object_pose[0])-3:int(object_pose[0])+1+25, int(object_pose[1])-7:int(object_pose[1])+1+7]=0


## updating the map for SM_PassengerBridge_bend
object_pose = [(client.simGetObjectPose('SM_PassengerBridge_bend_74').position.x_val-unreal_origin[0])*(-1)+75, (client.simGetObjectPose('SM_PassengerBridge_bend_74').position.y_val-unreal_origin[1])+125]
road_map[int(object_pose[0])-9:int(object_pose[0])+1+6, int(object_pose[1])-3:int(object_pose[1])+1+10]=0



## updating the map for SM_PassengerBridge
object_list = ['SM_PassengerBridge_116','SM_PassengerBridge2']

for item in object_list:
    object_pose = [(client.simGetObjectPose(item).position.x_val-unreal_origin[0])*(-1)+75, (client.simGetObjectPose(item).position.y_val-unreal_origin[1])+125]
    road_map[int(object_pose[0]-8.5):int(object_pose[0]+11.5), int(object_pose[1]-4):int(object_pose[1]+1)]=0
    


## updating the map for SM_AirbusA320_0
object_pose = [(client.simGetObjectPose('SM_AirbusA320_0').position.x_val-unreal_origin[0])*(-1)+75, (client.simGetObjectPose('SM_AirbusA320_0').position.y_val-unreal_origin[1])+125]
road_map[int(object_pose[0]-21):int(object_pose[0]+29), int(object_pose[1]-19):int(object_pose[1]+19)]=0


## updating the map for SM_AirbusA320_1'
object_pose = [(client.simGetObjectPose('SM_AirbusA320_1').position.x_val-unreal_origin[0])*(-1)+75, (client.simGetObjectPose('SM_AirbusA320_1').position.y_val-unreal_origin[1])+125]
road_map[int(object_pose[0]-29):int(object_pose[0]+21), int(object_pose[1]-19):int(object_pose[1]+19)]=0


print(client.simGetObjectPose('SM_AirbusA320_2'))
print(client.simGetObjectPose('SM_AirbusA320_0'))
print(client.simGetObjectPose('SM_TerminalBlockout_18').orientation.z_val)
print(client.simGetObjectPose('SM_TerminalBlockout2').orientation.z_val*2)



## updating the map for SM_AirbusA320_1'
object_pose = [(client.simGetObjectPose('SM_AirbusA320_2').position.x_val-unreal_origin[0])*(-1)+75, (client.simGetObjectPose('SM_AirbusA320_2').position.y_val-unreal_origin[1])+125]
road_map[int(object_pose[0]-29):int(object_pose[0]+21), int(object_pose[1]-19):int(object_pose[1]+19)]=0


plt.figure()
plt.imshow(road_map)

filename = 'savedImage.jpg'
  
# Using cv2.imwrite() method 
# Saving the image 
cv2.imwrite(filename, road_map) 
  