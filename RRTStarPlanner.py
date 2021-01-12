
"""
Created on Wed Feb  2 11:43:11 2020

@author: Niyousha Rahimi
"""




import numpy
from RRTTree import RRTTree
from random import randint
from matplotlib import pyplot as plt
import time

class RRTStarPlanner(object):

    def __init__(self, planning_env):
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)
        self.Trnear = []
        self.Trnew = []
        

    def Plan(self, start_config, goal_config, epsilon = 1, color=None):
        start = time.time()
        # Initialize an empty plan.
        plan = []

        # Start with adding the start configuration to the tree.
        self.tree.AddVertex(start_config)

        
        plan.append(start_config)
        plan.append(goal_config)
        ii=0
        
        print('#### Path Planning using RRT* method ####')
        print('...')
        print('It will take a few seconds...')
        print('')
        
        for i in range(1,5000):
            # print("i=",i)
            ii=i
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
            
            # plt.plot(x,y, 'k')
        
        
        
        cost = 0
        
        vertices, edge = self.ShortenPath(goal_config)
        plt.imshow(self.planning_env.map , interpolation='nearest')
        for i in edge:
            
            x = [vertices[edge[i]][0], vertices[i][0]]
            y = [vertices[edge[i]][1], vertices[i][1]]
            cost += numpy.sqrt(numpy.power(x[1]-x[0],2)+numpy.power(y[1]-y[0],2))
            if color==1:
                plt.plot(x,y, 'g', linewidth=4)
            else:
                plt.plot(x,y, 'b', linewidth=4)
            plan.append([x[0],y[0]])
        plt.show()
        
        end = time.time()
        print("cost=",cost)
        print("time=",end-start)
        print("number of iterations=",ii)
        #plan = vertices 
        plan.pop(0)
        p = plan.pop(0)
        plan.append(p)
        return plan
        
        

    def extend(self,start_config, goal_config,epsilon):
        
        #Implementing an extend logic.
        rand = numpy.random.randint(1,101)
        if rand<=20:
            s_rand = goal_config
        else:
            s_rand = [randint(numpy.minimum(start_config[0], goal_config[0]), numpy.maximum(start_config[0], goal_config[0])+10
            ),randint(numpy.minimum(start_config[1], goal_config[1]),numpy.maximum(start_config[1], goal_config[1]))]
            
            
        vid_snear, s_near = self.tree.GetNearestVertex(s_rand)
            
            
        xs_new = self.findu(s_near, s_rand, epsilon)
        # print(xs_new)
        # print(self.tree.vertices[0])
        #if xs_new not in self.tree.vertices:
        if xs_new not in self.tree.vertices:
            if (self.planning_env.state_validity_checker(xs_new)==True and 
                self.planning_env.edge_validity_checker(s_near,xs_new)==True):
                vid_xsnew = self.tree.AddVertex(xs_new)
                self.tree.AddEdge(vid_snear,vid_xsnew)
                
                
                self.Trnew.append(xs_new)
                self.Trnear.append(s_near)
                
                if len(self.tree.vertices)>3:
                    
                    knnId, knnVertices = self.tree.GetKNN(xs_new,3)
                
                    for knid in knnId:
                        if knid != vid_snear:
                            if self.planning_env.edge_validity_checker(self.tree.vertices[knid],xs_new)==True:
                                parent = []
                                new_parent = []

                                
                                if knid in self.tree.edges:
                                    new_parent.append(self.tree.edges[knid])
                                else:
                                    new_parent.append(knid)
                                parent.append(vid_snear)
                                while parent[-1] not in new_parent:
                                    if new_parent[-1] in self.tree.edges:
                                        new_parent.append(self.tree.edges[new_parent[-1]])
                                    if parent[-1] in self.tree.edges:
                                        parent.append(self.tree.edges[parent[-1]])
                                n = new_parent.index(parent[-1])
                                new_parent = new_parent[:n+1]
                                
                                if len(new_parent) < len(parent):
                                    del self.tree.edges[vid_xsnew]
                                    self.tree.AddEdge(knid,vid_xsnew)
                                    del self.Trnear[-1]
                                    self.Trnear.append(self.tree.vertices[knid])
                                    vid_snear = knid

                               
        pass
        
    
    def findu(self, near, rand, epsilon):
        if self.planning_env.state_validity_checker(near):
            u = numpy.array([rand[0]-near[0],rand[1]-near[1]])
            # print("u=",u)
            #u = u/(numpy.linalg.norm(u))
            new = near + epsilon*u
            
        
        return [new[0],new[1]]


    
    def ShortenPath(self, goal):
        #Postprocessing of the plan.
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
                    
