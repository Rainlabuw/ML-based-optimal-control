# -*- coding: utf-8 -*-
"""
Created on Wed Oct  7 13:30:52 2020

@author: Niyousha Rahimi
"""



import airsim
import cv2
import numpy as np
import os
import sys
import time
from matplotlib import pyplot as plt
import math

# Root directory of the project. Please change accordingly
ROOT_DIR = os.path.abspath("C:/Users/newsh/Desktop/Motion_lanning/AirSimNH")
sys.path.append(ROOT_DIR)



from MapEnvironment import MapEnvironment
from RRTStarPlanner import RRTStarPlanner
from AStarPlanner import AStarPlanner

from Mask_RCNN import Mask_RCNN 





def test_():
    global client
    responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
    response = responses[0]
    img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8) 
        
    # reshape array to 3 channel image array H X W X 3
    img_rgb = img1d.reshape(response.height, response.width, 3)

    # Instant segmentation using Mask-RCNN
    unknown_object_semantics = MRN.Object_detection(img_rgb)



def update_map(center, env_map):
    ##### Updating the map when an object is detected ######
    # It will put a box of 10 by 10 at the center mass of the object.
    env_map[int(center[1]-7):int(center[1]+7), int(center[0]-7):int(center[0]+7)]=0
    return env_map


def proper_round(a):
    if int(a)+0.5 > a:
        return int(a)
    else:
        return int(a)+1


    
def proper_angle(dx, dy):
    dy *= -1
    
    if dx <= 0 and dy >= 0:
        return 5*np.pi/2 - np.arctan2(dy, dx)
    else:
        return np.pi/2 - np.arctan2(dy, dx)


def setCarSteering(car_heading, path_heading):
    
    if car_heading >= 3*np.pi/2 and car_heading < 2*np.pi:
        if path_heading >= 0 and path_heading <= np.pi/2:
            steering = 0.45
        elif car_heading > path_heading:
            steering = -.45
        else:
            steering = .45    
    elif car_heading >= 0 and car_heading <= np.pi/2:
        if path_heading >= 3*np.pi/2 and path_heading < 2*np.pi:
            steering = -0.45
        elif car_heading > path_heading:
            steering = -.45
        else:
            steering = .45
    elif car_heading > path_heading:
        steering = -.45
    else:
        steering = .45
    return steering





def CheckForObstacles(unknown_object_semantics, vehicle_pose, car_heading):
    global client
    global MRN
     
    responses = client.simGetImages([airsim.ImageRequest(0, airsim.ImageType.DepthPerspective, pixels_as_float=True)])
    response = responses[0]
    img1d = np.array(response.image_data_float, dtype=np.float)
    img1d = img1d * 3.5 + 30
    img1d[img1d > 255] = 255
    img2d = np.fliplr(np.reshape(img1d, (responses[0].height, responses[0].width)))
    depth = np.array(img2d, dtype=np.uint8)
    
    
    savePointCloud(depth)
    
    got_depth = MRN.point_cloud(depth, vehicle_pose, car_heading)
    
    
    return MRN.object_pose_estimate(got_depth, unknown_object_semantics[0])





idx = 0
def save_images():
    ### Use this function to save Scene images from camera 1 ###
    ### Camera settings can be changed through settings.jason (it is usually under Documents/airsim) ###
    global idx 
    global client
    idx += 1
    #get camera images from the car
    responses = client.simGetImages([
        airsim.ImageRequest("1", airsim.ImageType.Scene)])  #scene vision image in uncompressed RGB array
    
    filename = 'c:/temp/py' + str(idx)
    if not os.path.exists('c:/temp/'):
        os.makedirs('c:/temp/')
    
    response = responses[0]
    airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
    

def savePointCloud(depth):
    color = (0, 255, 0)
    rgb = "%d %d %d" % color
    Width=256
    Height=144
    focal_length=Width/2
    B=20
    
    projectionMatrix =  np.array([
            [1, 0, 0, -Width/2],
            [0, 1, 0, -Height/2],
            [0, 0, 0, focal_length],
            [0, 0, -10, 0]])
    
    Image3D = cv2.reprojectImageTo3D(depth, projectionMatrix)
    
    outputFile = "cloud_01.asc"
    f = open(outputFile, "w")
    for x in range(Image3D.shape[0]):
        for y in range(Image3D.shape[1]):
            pt = Image3D[x, y]
            if (math.isinf(pt[0]) or math.isnan(pt[0])):
                # skip it
                None
            else:
                f.write("%f %f %f %s\n" % (pt[0], pt[1], pt[2] - 1, rgb))
    f.close()




'''
######################### MAIN ##############################
'''
MRN = Mask_RCNN()

# connect to the AirSim simulator 
client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = airsim.CarControls()
# client.reset()
# Loading the NN weights
test_()





#################### Getting unreal engine origin for change of frame ####################
unreal_origin = [client.simGetObjectPose('SM_TerminalBlockout_18').position.x_val, client.simGetObjectPose('SM_TerminalBlockout_18').position.y_val]



#################### The starting point of the simulator is the airsim origin ####################
vehicle_pose = [(client.simGetVehiclePose().position.y_val-unreal_origin[1])+125, (client.simGetVehiclePose().position.x_val-unreal_origin[0])*(-1)+75]
vehicle_path = []



#################### Initial path planning ####################
start = [300, 50]
print('start = ', start)
goal = [25,250]

vehicle_path.append(start)

# Loading the original MAP
img = cv2.imread('savedImage.jpg')
map_ = img[:,:,0]

planning_env = MapEnvironment(map_, start, goal, True) 

# Please avoid astar planner for now, as it has some issues
planner = 'rrtstar' 

if planner == 'astar':
    planner = AStarPlanner(planning_env)
elif planner == 'rrtstar':
    planner = RRTStarPlanner(planning_env)
else:
    print('Unknown planner option: %s' % planner)
    exit(0)


plan = planner.Plan(start, goal)
print('original plan check points: ', plan)





#################### Setting the car location to the start point ####################
theta=proper_angle(plan[1][0]-plan[0][0], plan[1][1]-plan[0][1])

position = airsim.Vector3r(round(unreal_origin[0])+25, round(unreal_origin[1])+175, 0)
heading = airsim.utils.to_quaternion(0,0,theta)
pose = airsim.Pose(position, heading)
client.simSetVehiclePose(pose, True)


# True position of the unknown obstacle (Car_2)
unknown_object_pose = [ (client.simGetObjectPose('Car_2').position.y_val-unreal_origin[1])+125, (client.simGetObjectPose('Car_2').position.x_val-unreal_origin[0])*(-1)+75]


switch = 0      
i=1
while(np.sqrt( (vehicle_pose[0]-plan[-1][0])**2 + (vehicle_pose[1]-plan[-1][1])**2) > 10 ):
    
    while(np.sqrt( (vehicle_pose[0]-plan[i][0])**2 + (vehicle_pose[1]-plan[i][1])**2) > 10 ):
        car_state = client.getCarState()
        vehicle_pose = [ (client.simGetVehiclePose().position.y_val-unreal_origin[1])+125, (client.simGetVehiclePose().position.x_val-unreal_origin[0])*(-1)+75]
        print('vehicle_pose = ',vehicle_pose)
        print(plan[i])
        print(i)
        
        vehicle_path.append(vehicle_pose)
        
        if car_state.speed > 5:
            car_controls.throttle = 0
        else:
            car_controls.throttle = 0.5
        car_controls.steering = 0
        client.setCarControls(car_controls)
        time.sleep(.1)
        
        
        if (np.sqrt((vehicle_pose[0]-unknown_object_pose[0])**2+(vehicle_pose[1]-unknown_object_pose[1])**2))<25 and switch==0: #
            
            car_heading = proper_angle(client.getCarState().kinematics_estimated.linear_velocity.y_val, -client.getCarState().kinematics_estimated.linear_velocity.x_val)
            vehicle_pose = [ (client.simGetVehiclePose().position.y_val-unreal_origin[1])+125, (client.simGetVehiclePose().position.x_val-unreal_origin[0])*(-1)+75]
            vehicle_path.append(vehicle_pose)
            
            car_controls.brake = 1
            client.setCarControls(car_controls)
            time.sleep(.1)
            
            responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
            response = responses[0]
            img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8) 
            
            
            
            # reshape array to 3 channel image array H X W X 3
            img_rgb = img1d.reshape(response.height, response.width, 3)
            # plt.imshow(img_rgb)
            
            
            # Instant segmentation using Mask-RCNN
            unknown_object_semantics = MRN.Object_detection(img_rgb)
            
            
            if  len(unknown_object_semantics) > 0: 
            
                switch = 1
                print('Replanning')
                 
                
                vehicle_pose = [ (client.simGetVehiclePose().position.y_val-unreal_origin[1])+125, (client.simGetVehiclePose().position.x_val-unreal_origin[0])*(-1)+75]
                vehicle_path.append(vehicle_pose)
                obs_pose = CheckForObstacles(unknown_object_semantics, vehicle_pose, car_heading)
    
                print('')
                print('#########################')
                print('vehicle pose = ', vehicle_pose)
                print('obstacle pose = ', obs_pose)
                print('True obstacle pose = ', unknown_object_pose)
                print('#########################')
                print('')
                
                map_ = update_map(obs_pose[0:2], map_) #obs_pose[0:2]
                start = [proper_round(vehicle_pose[0]), proper_round(vehicle_pose[1])]
                planning_env2 = MapEnvironment(map_, start, goal, True) 
                plan.clear()
                planner2 = RRTStarPlanner(planning_env2)
                plan = planner2.Plan(start, goal,1, 1)
                i = 1
                print('new plan check points: ', plan)
                
                path_heading = proper_angle(plan[i][0]-plan[i-1][0], (plan[i][1]-plan[i-1][1]) )
                
                car_controls.brake = 0  
                car_controls.steering = setCarSteering(car_heading, path_heading)
                print('Initiate steering')
                while(np.abs(car_heading- path_heading)>0.03):
                    car_controls.throttle = 0.5
                    client.setCarControls(car_controls)
                    time.sleep(.1)
                    
                    
                    car_heading = proper_angle(client.getCarState().kinematics_estimated.linear_velocity.y_val, -client.getCarState().kinematics_estimated.linear_velocity.x_val)
                    # print('car_heading =' , car_heading)
                    # print('path_heading = ', path_heading)
                    vehicle_pose = [ (client.simGetVehiclePose().position.y_val-unreal_origin[1])+125, (client.simGetVehiclePose().position.x_val-unreal_origin[0])*(-1)+75]
                    vehicle_path.append(vehicle_pose)
                    # print('vehicle pose = ', vehicle_pose)
                print('Done')
                print('')
                
            if switch == 0: 
                car_controls.brake = 0
                car_controls.throttle = 0.5
                car_controls.steering = 0
                client.setCarControls(car_controls)
                time.sleep(.1)
                vehicle_pose = [ (client.simGetVehiclePose().position.y_val-unreal_origin[1])+125, (client.simGetVehiclePose().position.x_val-unreal_origin[0])*(-1)+75]
                vehicle_path.append(vehicle_pose)
                
    
    if (vehicle_pose[0]<35 ):
        break
    i+=1
    if i >= np.size(plan)-1:
        break
    path_heading = proper_angle(plan[i][0]-plan[i-1][0], (plan[i][1]-plan[i-1][1]))
    car_heading = proper_angle(client.getCarState().kinematics_estimated.linear_velocity.y_val, -client.getCarState().kinematics_estimated.linear_velocity.x_val)
    
    
    car_controls.steering = setCarSteering(car_heading, path_heading)
    print('Initiate steering')
    while(np.abs(car_heading- path_heading)>0.03):
        car_controls.throttle = 0.5
        client.setCarControls(car_controls)
        time.sleep(.1)
        # save_images()
        
        
        car_heading = proper_angle(client.getCarState().kinematics_estimated.linear_velocity.y_val, -client.getCarState().kinematics_estimated.linear_velocity.x_val)
        
        vehicle_pose = [ (client.simGetVehiclePose().position.y_val-unreal_origin[1])+125, (client.simGetVehiclePose().position.x_val-unreal_origin[0])*(-1)+75]
        vehicle_path.append(vehicle_pose)
    print('Done')
    print('')
    
    vehicle_pose = [ (client.simGetVehiclePose().position.y_val-unreal_origin[1])+125, (client.simGetVehiclePose().position.x_val-unreal_origin[0])*(-1)+75]
    vehicle_path.append(vehicle_pose)
    
    
car_controls.brake = 1
client.setCarControls(car_controls)
time.sleep(.1)




#################### Visualizing vehicle's path ####################
for j in range(np.shape(vehicle_path)[0]-1):
    x = [vehicle_path[j][0], vehicle_path[j+1][0]]
    y = [vehicle_path[j][1], vehicle_path[j+1][1]]
    plt.plot(x,y, 'r', linewidth=4)


#restore to original state
client.reset()
client.enableApiControl(False)





















