import numpy
import cv2

from matplotlib import pyplot as plt

class MapEnvironment(object):
    
    def __init__(self, mapfile, start, goal, im):

        # Obtain the boundary limits.
        # Check if file exists.
        if im==True:
            # img = cv2.imread(mapfile)
            self.map = mapfile
            print('shape of map = ',numpy.shape(self.map))
            plt.imshow(self.map, interpolation='nearest')
            
            
        else:
            self.map = numpy.loadtxt(mapfile)
            
        self.ylimit = [0, numpy.shape(self.map)[0]] # TODO (avk): Check if this needs to flip.
        self.xlimit = [0, numpy.shape(self.map)[1]]
        
        print(self.xlimit, self.ylimit)

        
        # Check if start and goal are within limits and collision free
        if not self.state_validity_checker(start) or not self.state_validity_checker(goal):
            raise ValueError('Start and Goal state must be within the map limits');
            exit(0)

        # Display the map
        plt.imshow(self.map, interpolation='nearest')
        
    def compute_distance(self, start_config, end_config):
        # TODO: Implement a function which computes the distance between
        # two configurations.
        #
        dx = end_config[0]-start_config[0]
        dy = end_config[1]-start_config[1]
        d = numpy.sqrt(dx**2+dy**2)
        
        return d


    def state_validity_checker(self, config):
        # TODO: Implement a state validity checker
        # Return true if valid.
        
        #print('config= ',config )
        
        if (config[0]>=self.xlimit[0] and config[0]<=self.xlimit[1] and
            config[1]>=self.ylimit[0] and config[1]<=self.ylimit[1]):
            
            if config[0]>numpy.ceil(config[0])-0.5:
                x = numpy.ceil(config[0])
            else:
                x = numpy.floor(config[0])
                
            if config[1]>numpy.ceil(config[1])-0.5:
                y = numpy.ceil(config[1])
            else:
                y = numpy.floor(config[1])
            
            y=int(y)
            x=int(x)
            #print(y,x)
            
            if self.map[y][x]==255 :
                
                return True
            else:
                return False
        else:
            return False

    
    def edge_validity_checker(self, config1, config2):

        #
        # TODO: Implement an edge validity checker
        x1 = numpy.minimum(config1[0],config2[0])
        x2 = numpy.maximum(config1[0],config2[0])
        y1 = numpy.minimum(config1[1],config2[1])
        y2 = numpy.maximum(config1[1],config2[1])
        
        for i in range(int(x1), int(x2)+1):
            for j in range(int(y1), int(y2)+1):
                if self.map[j][i]==0:
                    return False
          
        
        else:
            return True    
        
        pass

    def compute_heuristic(self, config, goal):
        # TODO: Implement a function to compute heuristic.
        dx = config[0]-goal[0]
        dy = config[1]-goal[1]
        h = numpy.sqrt(dx**2+dy**2)
        return h
        
        

    def visualize_plan(self, plan):
        '''
        Visualize the final path
        @param plan Sequence of states defining the plan.
        '''
        plt.imshow(self.map, interpolation='nearest')
        for i in range(numpy.shape(plan)[0] - 1):
            x = [plan[i,0], plan[i+1, 0]]
            y = [plan[i,1], plan[i+1, 1]]
            
            plt.plot(y, x, 'k')
        plt.show()
        
    def visualize_Tree(self, plan):
        '''
        Visualize the final path
        @param plan Sequence of states defining the plan.
        '''
        plt.imshow(self.map, interpolation='nearest')
        for i in range(numpy.shape(plan)[0] - 1):
            x = [plan[i,0], plan[i+1, 0]]
            y = [plan[i,1], plan[i+1, 1]]
            
            plt.plot(y, x, 'k')
        plt.show()