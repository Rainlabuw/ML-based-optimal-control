import time
import numpy

from matplotlib import pyplot as plt

class AStarPlanner(object):    
    def __init__(self, planning_env):
        self.planning_env = planning_env
        self.nodes = dict()

        self.vertices = []
        self.g = dict()
        self.h = dict()
        self.edges = dict()
        self.Closed = []
        self.Open = []
        
    def Plan(self, start_config, goal_config):
        start = time.time()
        plan = []
        

        # TODO (student): Implement your planner here.

        plan.append(start_config)
        plan.append(goal_config)
        
        for i in range(numpy.minimum(start_config[0], goal_config[0])-1, numpy.maximum(start_config[0], goal_config[0])+1):
            for j in range(numpy.minimum(start_config[1], goal_config[1])-1,numpy.maximum(start_config[1], goal_config[1])+1):
                if self.planning_env.state_validity_checker([i,j])==True:
                    vID = self.AddVertex([i,j])
                    self.Add_gFunction(vID,1000000)
                    self.Add_hFunction(vID,self.planning_env.compute_heuristic(self.vertices[vID],goal_config))
                j+=3
            i+=3
        
                
        vID_start = self.vertices.index(start_config)
        self.g[vID_start]=0
        self.Open.append(vID_start)
              
        
        while self.vertices.index(goal_config) not in self.Closed:
           print(len(self.Closed))
           vID_expand = self.Pick_expand(epsilon=20)
           
           self.Open.remove(vID_expand)
           self.Closed.append(vID_expand)
           print("close",self.vertices[self.Closed[-1]])
           
           successors = self.Get_Successors(self.vertices[vID_expand])
           
           for v in successors :
               
               if v not in self.Closed:
                   if v not in self.Open:
                       self.Open.append(v)
                       self.AddEdge(vID_expand,v)
                       
                   if self.g[v] > self.g[vID_expand] + self.planning_env.compute_distance(self.vertices[v],self.vertices[vID_expand]):
                       self.g[v] = self.g[vID_expand] + self.planning_env.compute_distance(self.vertices[v],self.vertices[vID_expand])
                   
        
        plt.imshow(self.planning_env.map , interpolation='nearest')
        for i in self.edges:
            
            x = [self.vertices[self.edges[i]][0], self.vertices[i][0]]
            y = [self.vertices[self.edges[i]][1], self.vertices[i][1]]
            #cost += numpy.sqrt(numpy.power(x[1]-x[0],2)+numpy.power(y[1]-y[0],2))
            plt.plot(x,y, 'k')
        
                       
        cost = 0
        plan = self.ShortenPath(start_config,goal_config, epsilon=20)
        plt.imshow(self.planning_env.map, interpolation='nearest')
        for i in range(numpy.shape(plan)[0] - 1):
            x = [plan[i][0], plan[i+1][ 0]]
            y = [plan[i][1], plan[i+1][ 1]]
            
            cost += numpy.sqrt((x[0]-x[1])**2+(y[0]-y[1])**2)
            plt.plot(x, y, 'r')
        
        plt.show()
        
        end = time.time()
        print("cost=",cost)
        print("time=",end-start)
        
        
        return plan

        
        
        
        
        
        
    def ShortenPath(self, start, goal, epsilon):
        # TODO (student): Postprocessing of the plan.    
        path = []
        path.append(goal)
        
        while start not in path:
            print("path",path[-1])
            successors = self.Get_Successors(path[-1])
            f = dict()
            
            for s in successors:
                print("successor", self.vertices[s])
                if s in self.Closed and self.vertices[s] not in path:
                    f[s] = self.g[s] + epsilon*(self.planning_env.compute_heuristic(self.vertices[s],start))
                    print("f=",f[s])
                else:
                    continue 
            if len(f)==1:
                for key in f:
                    v_next = key
            else:
                v_next = min(f, key = lambda x: f.get(x) )
                
            path.append(self.vertices[v_next])
                
        return path
        
        
    def AddVertex(self, config):
       
        vid = len(self.vertices)
        self.vertices.append(config)
        return vid

    def Add_gFunction(self, vID, g):
        self.g[vID] = g
    
    def Add_hFunction(self, vID, h):
        self.h[vID] = h

    def Get_Successors(self,config):
        successors = []
        for i in range(config[0]-1,config[0]+2):
            for j in range(config[1]-1,config[1]+2):
                if i == config[0] and j == config[1]:
                    continue
                else:
                    if self.planning_env.state_validity_checker([i,j])==True:
                        if [i,j] in self.vertices:
                            successors.append(self.vertices.index([i,j]))
        return successors
    

    def Pick_expand(self, epsilon):
        
        if len(self.Open)==1:
            return self.Open[0]
        else:
            f = dict()
            for v in self.Open:
                f[v] = self.g[v] + epsilon*self.h[v]
                
            return min(f, key = lambda x: f.get(x) )
            
    
    def AddEdge(self, sid, eid):
        self.edges[eid] = sid
        