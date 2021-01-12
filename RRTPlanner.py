
"""
Created on Wed Feb  12 21:23:42 2020

@author: Niyousha Rahimi
"""


import numpy
from RRTTree import RRTTree
from random import randint
import time

from matplotlib import pyplot as plt

class RRTPlanner(object):

    def __init__(self, planning_env):
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)
        self.Trnear = []
        self.Trnew = []

    def Plan(self, start_config, goal_config, epsilon = 1):
        start = time.time()
        # Initialize an empty plan.
        plan = []
        
        # Start with adding the start configuration to the tree.
        self.tree.AddVertex(start_config)
        
        
        
        
        plan.append(start_config)
        plan.append(goal_config)
        
        ii = 0
        for i in range(1,40000):
            #print("i=",i)
            ii = i
            self.extend(start_config, goal_config,epsilon)
            
            if len(self.tree.vertices)>3:
                if (int(self.tree.vertices[-2][0]) in range(int(goal_config[0])-3,int(goal_config[0])+3) and
                    int(self.tree.vertices[-2][1]) in range(int(goal_config[1])-3,int(goal_config[1])+3)):
                    break
            
                

        #near, new = self.ShortenPath(self.Trnear, self.Trnew)
        print("number of vertices", len(self.tree.vertices))
        
        
        plt.imshow(self.planning_env.map , interpolation='nearest')
        for i in range(numpy.shape(self.Trnew)[0] - 1):
            
            x = [self.Trnear[i][0], self.Trnew[i][0]]
            y = [self.Trnear[i][1], self.Trnew[i][1]]
            
            plt.plot(x,y, 'k')
       
        
        
        cost = 0
        
        vertices, edge = self.ShortenPath(goal_config)
        plt.imshow(self.planning_env.map , interpolation='nearest')
        for i in edge:
            
            x = [vertices[edge[i]][0], vertices[i][0]]
            y = [vertices[edge[i]][1], vertices[i][1]]
            cost += numpy.sqrt(numpy.power(x[1]-x[0],2)+numpy.power(y[1]-y[0],2))
            plt.plot(x,y, 'r')
        plt.show()
        
        end = time.time()
        print("cost=",cost)
        print("time=",end-start)
        print("number of iterations=",ii)
        
        return numpy.array(plan)

        
    
    def extend(self,start_config, goal_config,epsilon):
        
        rand = numpy.random.randint(1,101)
        if rand<=5:
            s_rand = goal_config
        else:
            s_rand = [randint(numpy.minimum(start_config[0], goal_config[0]), numpy.maximum(start_config[0], goal_config[0])
            ),randint(numpy.minimum(start_config[1], goal_config[1]),numpy.maximum(start_config[1], goal_config[1]))]
            
            
        vid_snear, s_near = self.tree.GetNearestVertex(s_rand)
            
            
        xs_new = self.findu(s_near, s_rand, epsilon)
        
        
        #if xs_new not in self.tree.vertices:
        if xs_new not in self.tree.vertices:
            if (self.planning_env.state_validity_checker(xs_new)==True and 
                self.planning_env.edge_validity_checker(s_near,xs_new)==True):
                vid_xsnew = self.tree.AddVertex(xs_new)
                self.tree.AddEdge(vid_snear,vid_xsnew)
                
                self.Trnear.append(s_near)
                self.Trnew.append(xs_new)
        

        pass
    
    def extend2(self,start_config, goal_config,epsilon):
        # Implement an extend logic.
        rand = numpy.random.randint(1,101)
        if rand<=50:
            s_rand = goal_config
        else:
            s_rand = [randint(numpy.minimum(start_config[0], goal_config[0]), numpy.maximum(start_config[0], goal_config[0])
            ),randint(numpy.minimum(start_config[1], goal_config[1]),numpy.maximum(start_config[1], goal_config[1]))]
            
            
        vid_snear, s_near = self.tree.GetNearestVertex(s_rand)
            
        
        if s_rand not in self.tree.vertices:
            if (self.planning_env.state_validity_checker(s_rand)==True and 
                self.planning_env.edge_validity_checker(s_near,s_rand)==True):
                vid_xsnew = self.tree.AddVertex(s_rand)
                
                self.tree.AddEdge(vid_snear,vid_xsnew)
                
                self.Trnear.append(s_near)
                self.Trnew.append(s_rand)
        
        

        pass

    def ShortenPath(self, goal):#, new):
        # Postprocessing of the plan.
        vertices = self.tree.vertices
        edge = self.tree.edges
        i=1
        while i>0:
            i=0
            for v in vertices:
                if vertices.index(v) not in edge.values():
                    if int(v[0]) not in range(int(goal[0])-3,int(goal[0])+3) or int(v[1]) not in range(int(goal[1])-3,int(goal[1])+3) :
                        if vertices.index(v) in edge:
                            del edge[vertices.index(v)]
                            i+=1 
        return vertices, edge
        
        
        
    def findu(self, near, rand, epsilon):
        if self.planning_env.state_validity_checker(near):
            u = numpy.array([rand[0]-near[0],rand[1]-near[1]])
            
            #u = u/(numpy.linalg.norm(u))
            new = near + epsilon*u
            
        
        return [new[0],new[1]]
        
        
  
