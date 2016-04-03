import numpy

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution
        self.resolution = resolution

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        for idx in range(self.dimension):
            self.num_cells[idx] = numpy.ceil((upper_limits[idx] - lower_limits[idx])/resolution)


    def ConfigurationToNodeId(self, config):
        
        # TODO:
        # This function maps a node configuration in full configuration
        # space to a node in discrete space
        #
        coord = [0]*self.dimension
        coord = self.ConfigurationToGridCoord(config)
        node_id = self.GridCoordToNodeId(coord)
        return node_id

    def NodeIdToConfiguration(self, nid):
        
        # TODO:
        # This function maps a node in discrete space to a configuraiton
        # in the full configuration space
        #
        config = [0] * self.dimension
        coord = [0]*self.dimension
        coord = self.NodeIdToGridCoord(nid)
        config = self.GridCoordToConfiguration(coord)
        return config
        
    def ConfigurationToGridCoord(self, config): #DONE
        
        # TODO:
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #
        coord = [0] * self.dimension
       	for i in range(self.dimension):
       		l = int((config[i]-self.lower_limits[i])/self.resolution)
       		coord[i] = l
       	return coord

    def GridCoordToConfiguration(self, coord): #Done
        
        # TODO:
        # This function smaps a grid coordinate in discrete space
        # to a configuration in the full configuration space
        #
        config = [0] * self.dimension
        for i in range(self.dimension):
       		config[i] = coord[i]*self.resolution + 0.5*self.resolution + self.lower_limits[i]

        return config

    def GridCoordToNodeId(self,coord):
        
        # TODO:
        # This function maps a grid coordinate to the associated
        # node id 
        node_id = 0
        for i in range(self.dimension):
        	mul = 1
        	for j in range(self.dimension - i-1):
        		mul = mul*self.num_cells[j]
        	node_id = node_id + coord[self.dimension - i-1]*mul
        	node_id = int(node_id)

        return node_id

    def NodeIdToGridCoord(self, node_id):
        
        # TODO:
        # This function maps a node id to the associated
        # grid coordinate
        coord = [0] * self.dimension
        newnode = node_id
        for i in range(self.dimension):
        	div = 1
        	for j in range(self.dimension - i-1):
        		div = div*self.num_cells[j]
        	l = (int)(newnode/div)

        	coord[self.dimension - i-1] = l 
        	newnode =  newnode%div

        return coord
        
        
        
