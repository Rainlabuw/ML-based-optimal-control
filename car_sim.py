# -*- coding: utf-8 -*-
"""
Created on Wed Oct  7 13:30:52 2020

@author: newsh
"""



import airsim
import cv2
import numpy as np
import os
import sys

import time
import numpy as np
from matplotlib import pyplot as plt
from PIL import Image
import imageio
import pprint

import argparse, numpy, time


# Root directory of the project
ROOT_DIR = os.path.abspath("C:/Users/newsh/Desktop/Motion_lanning/AirSimNH")
sys.path.append(ROOT_DIR)


from MapEnvironment import MapEnvironment
from RRTPlanner import RRTPlanner
from RRTStarPlanner import RRTStarPlanner
from AStarPlanner import AStarPlanner
from IPython import embed

# from Mask_RCNN import Mask_RCNN 


# MRN = Mask_RCNN()

def update_map(center, env_map):
    env_map[int(center[1]-7):int(center[1]+7), int(center[0]-7):int(center[0]+7)]=0
    return env_map


def proper_round(a):
    if int(a)+0.5 > a:
        return int(a)
    else:
        return int(a)+1


    
def proper_angle(dy, dx):
    if dx==0:
        return 0.0
    else:
        
        if dy>0 and dx<0:
            return np.arctan(dy/dx)
        elif dy<0 and dx<0:
            return np.arctan(dy/dx)
        elif dy>0 and dx>0:
            return -np.pi/2-np.arctan(dy/dx)
                
        else:
            return np.arctan(dy/dx)


# def follow_path(path, initial_heading):
#     if 



idx = 0
def save_images():
    global idx 
    global client
    idx += 1
    #get camera images from the car
    responses = client.simGetImages([
        # airsim.ImageRequest("1", airsim.ImageType.Segmentation, False, False)])
        airsim.ImageRequest("1", airsim.ImageType.Scene)])  #scene vision image in uncompressed RGB array
    # print('Retrieved images: %d', len(responses))
    
    # img_DIR = os.path.join(ROOT_DIR, "images")
    # filename = os.path.normpath('C:/Users/newsh/Desktop/Motion_lanning/AirSimNH/py' + str(idx) + '.png') 
    # if not os.path.exists(img_DIR):
    #     os.makedirs(img_DIR)
    
    
    # response = responses[0]
    # airsim.write_file(filename , response.image_data_uint8)
    # # img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8) # get numpy array
    # # img_rgb = img1d.reshape(response.height, response.width, 3) # reshape array to 3 channel image array H X W X 3
    # # cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png
    
    # get camera images from the car
    responses = client.simGetImages([
        # airsim.ImageRequest("1", airsim.ImageType.Segmentation, False, False)])
        airsim.ImageRequest("1", airsim.ImageType.Scene)])  #scene vision image in uncompressed RGB array
    # print('Retrieved images: %d', len(responses))
    
    filename = 'c:/temp/py' + str(idx)
    if not os.path.exists('c:/temp/'):
        os.makedirs('c:/temp/')
    
    
    response = responses[0]
    airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
    # MRN.Object_detection((os.path.normpath(filename + '.png')))
    
# def setCarControls():
    
#     if car_state.speed > 4:
#             car_controls.throttle = 0
#         else:
#             car_controls.throttle = 0.5
#         car_controls.steering = 0
#         client.setCarControls(car_controls)
#         time.sleep(.1)

pp = pprint.PrettyPrinter(indent=4)

seed = np.random.seed()
print(seed)

# connect to the AirSim simulator 
client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = airsim.CarControls()
client.reset()
# car_controls.brake = 1
# client.setCarControls(car_controls)
# time.sleep(.1)



print('initial heading = ' ,(client.simGetVehiclePose().orientation.z_val)*180/np.pi)

unreal_origin = [client.simGetObjectPose('SM_TerminalBlockout_18').position.x_val, client.simGetObjectPose('SM_TerminalBlockout_18').position.y_val]
vehicle_pose = [(client.simGetVehiclePose().position.y_val-unreal_origin[1])+125, (client.simGetVehiclePose().position.x_val-unreal_origin[0])*(-1)+75]

vehicle_path = []


start = [int(vehicle_pose[0]), int(vehicle_pose[1])]#[148, 321]
print('start = ', start)
goal = [25,250]

vehicle_path.append(start)

# First setup the environment and the robot.
img = cv2.imread('savedImage.jpg')
map_ = img[:,:,0]
planning_env = MapEnvironment(map_, start, goal, True) 

planner = 'rrtstar' #'' rrtstar''rrtstar

if planner == 'astar':
    planner = AStarPlanner(planning_env)
elif planner == 'rrt':
    planner = RRTPlanner(planning_env)
elif planner == 'rrtstar':
    planner = RRTStarPlanner(planning_env)
else:
    print('Unknown planner option: %s' % planner)
    exit(0)



plan = planner.Plan(start, goal)
# plan.reverse()

print(plan)

theta=proper_angle(plan[1][1]-plan[0][1], plan[1][0]-plan[0][0])

theta-=np.pi/2
print(theta*180/np.pi)
if theta*180/np.pi<-180.0:
    theta+=np.pi/2


vehicle_pose = [(client.simGetVehiclePose().position.y_val-unreal_origin[1])+125, (client.simGetVehiclePose().position.x_val-unreal_origin[0])*(-1)+75]
print(vehicle_pose)
position = airsim.Vector3r(0,0,0)
heading = airsim.utils.to_quaternion(0,0,theta)
pose = airsim.Pose(position, heading)
client.simSetVehiclePose(pose, True)

switch = 0

print('heading= %d, shib = %e', client.simGetVehiclePose().orientation.z_val, np.arctan((plan[1][1]-plan[0][1])/(plan[1][0]-plan[0][0]))-np.pi/2 )
i=1
while(np.sqrt( (vehicle_pose[0]-plan[-1][0])**2 + (vehicle_pose[1]-plan[-1][1])**2) > 10 ):
    
    while(np.sqrt( (vehicle_pose[0]-plan[i][0])**2 + (vehicle_pose[1]-plan[i][1])**2) > 13 ):
        car_state = client.getCarState()
        vehicle_pose = [ (client.simGetVehiclePose().position.y_val-unreal_origin[1])+125, (client.simGetVehiclePose().position.x_val-unreal_origin[0])*(-1)+75]
        print(vehicle_pose)
        vehicle_path.append(vehicle_pose)
        unknown_object_pose = [ (client.simGetObjectPose('Car_2').position.y_val-unreal_origin[1])+125, (client.simGetObjectPose('Car_2').position.x_val-unreal_origin[0])*(-1)+75]
        
        if car_state.speed > 4:
            car_controls.throttle = 0
        else:
            car_controls.throttle = 0.5
        car_controls.steering = 0
        client.setCarControls(car_controls)
        time.sleep(.1)
        save_images()
        
        if (np.sqrt((vehicle_pose[0]-unknown_object_pose[0])**2+(vehicle_pose[1]-unknown_object_pose[1])**2))<30 and switch==0:
            switch = 1
            print('HERE WE GO!')
            car_heading = proper_angle(-client.getCarState().kinematics_estimated.linear_velocity.x_val, client.getCarState().kinematics_estimated.linear_velocity.y_val)
            print('car_heading =' , car_heading, car_heading*180/np.pi)
            car_controls.throttle = 0
            # car_controls.brake = 1
            # client.setCarControls(car_controls)
            # time.sleep(.1)
            vehicle_pose = [ (client.simGetVehiclePose().position.y_val-unreal_origin[1])+125, (client.simGetVehiclePose().position.x_val-unreal_origin[0])*(-1)+75]
            print(vehicle_pose)
            print(unknown_object_pose)
            vehicle_path.append(vehicle_pose)
            
            map_ = update_map(unknown_object_pose, map_)
            start = [proper_round(vehicle_pose[0]), proper_round(vehicle_pose[1])]
            planning_env2 = MapEnvironment(map_, start, goal, True) 
            plan.clear()
            planner2 = RRTStarPlanner(planning_env2)
            plan = planner2.Plan(start, goal,1, 1)
            i = 1
            print(plan)
            
            path_heading = proper_angle((plan[i][1]-plan[i-1][1]), plan[i][0]-plan[i-1][0] )
            print('path_heading = ', path_heading, path_heading*180/np.pi)
            
            
            if car_heading > path_heading:
                car_controls.steering = -.25
            else:
                car_controls.steering = .25
            
            while(np.abs(car_heading- path_heading)>0.02):
                car_controls.throttle = 0.5
                client.setCarControls(car_controls)
                time.sleep(.1)
                save_images()
                
                car_heading = proper_angle(-client.getCarState().kinematics_estimated.linear_velocity.x_val, client.getCarState().kinematics_estimated.linear_velocity.y_val)
                print('car_heading =' , car_heading)
                vehicle_pose = [ (client.simGetVehiclePose().position.y_val-unreal_origin[1])+125, (client.simGetVehiclePose().position.x_val-unreal_origin[0])*(-1)+75]
                vehicle_path.append(vehicle_pose)
            
       
    print(vehicle_pose, plan[i])
    if (vehicle_pose[0]<35):
        break
    i+=1
    if i >= np.size(plan)-1:
        break
    path_heading = proper_angle((plan[i][1]-plan[i-1][1]), plan[i][0]-plan[i-1][0] )
    print('path_heading = ', path_heading, path_heading*180/np.pi)
    car_heading = proper_angle(-client.getCarState().kinematics_estimated.linear_velocity.x_val, client.getCarState().kinematics_estimated.linear_velocity.y_val)
    print('car_heading =' , car_heading, car_heading*180/np.pi)
    
    if car_heading > path_heading:
        car_controls.steering = -.25
    else:
        car_controls.steering = .25
    
    while(np.abs(car_heading- path_heading)>0.02):
        car_controls.throttle = 0.5
        client.setCarControls(car_controls)
        time.sleep(.1)
        save_images()
        
        
        car_heading = proper_angle(-client.getCarState().kinematics_estimated.linear_velocity.x_val, client.getCarState().kinematics_estimated.linear_velocity.y_val)
        print('car_heading =' , car_heading)
        vehicle_pose = [ (client.simGetVehiclePose().position.y_val-unreal_origin[1])+125, (client.simGetVehiclePose().position.x_val-unreal_origin[0])*(-1)+75]
        vehicle_path.append(vehicle_pose)
    vehicle_pose = [ (client.simGetVehiclePose().position.y_val-unreal_origin[1])+125, (client.simGetVehiclePose().position.x_val-unreal_origin[0])*(-1)+75]
    vehicle_path.append(vehicle_pose)
    
    
car_controls.brake = 1
client.setCarControls(car_controls)
time.sleep(.1)
save_images()

for j in range(np.shape(vehicle_path)[0]-1):
    x = [vehicle_path[j][0], vehicle_path[j+1][0]]
    y = [vehicle_path[j][1], vehicle_path[j+1][1]]
    plt.plot(x,y, 'r', linewidth=4)

#restore to original state
client.reset()
client.enableApiControl(False)





















