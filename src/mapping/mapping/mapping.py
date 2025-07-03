

# Python standard library
from math import cos, sin, atan2, fabs

# Numpy
import numpy as np

from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import LaserScan
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import Header


from . grid_map import GridMap


class Mapping:
    def __init__(self, unknown_space, free_space, c_space, occupied_space,boundary,
                 radius, workspace, optional=None):
        self.ws = workspace
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.boundary = boundary
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space,
                                      "self.boundary":self.boundary}
        self.radius = radius
        self.__optional = optional

    def set_workspace(self,grid_map:GridMap):
        origin = grid_map.get_origin()
        resolution = grid_map.get_resolution()
        l = self.ws.shape[0]
        for indx, row in self.ws.iterrows():
            if indx<l-1:
                end_x = int((self.ws.iloc[indx+1].x-origin.position.x)/resolution)
                end_y = int((self.ws.iloc[indx+1].y-origin.position.y)/resolution)
                
                start_x = int((row.x-origin.position.x)/resolution)
                start_y = int((row.y-origin.position.y)/resolution)
                traversed = self.raytrace([start_x,start_y],[end_x,end_y])
                
                for point in traversed:
                    self.add_to_map(grid_map,point[0],point[1],self.boundary)
        
    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx
            

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map.__setitem__([x, y], value)
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False

    def update_map(self, grid_map:GridMap, pose, scan):
        """Updates the grid_map with the data from the laser scan and the pose.
        
        For E: 
            Update the grid_map with self.occupied_space.

            Return the updated grid_map.

            You should use:
                self.occupied_space  # For occupied space

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        For C:
            Update the grid_map with self.occupied_space and self.free_space. Use
            the raytracing function found in this file to calculate free space.

            You should also fill in the update (OccupancyGridUpdate()) found at
            the bottom of this function. It should contain only the rectangle area
            of the grid_map which has been updated.

            Return both the updated grid_map and the update.

            You should use:
                self.occupied_space  # For occupied space
                self.free_space      # For free space

                To calculate the free space you should use the raytracing function
                found in this file.

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        :type grid_map: GridMap
        :type pose: PoseStamped
        :type scan: LaserScan
        """

        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = grid_map.get_origin()
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()
        
        """
        Fill in your solution here
        """
        robot_x,robot_y = pose.pose.position.x, pose.pose.position.y
        ranges = scan.ranges
        range_min = scan.range_min
        range_max = scan.range_max
        start_angle = scan.angle_min
        angle_inc = scan.angle_increment
        min_x = grid_map.get_width()
        max_x =0
        min_y = grid_map.get_height()
        max_y=0
        occupied=[]
        for i,r in enumerate(ranges):
            angle= robot_yaw+start_angle+i*angle_inc
            if r<=range_min or r>=range_max:
                continue
            else:
                
                x = int((robot_x+r*cos(angle)-origin.position.x)/resolution)
                y = int((robot_y+r*sin(angle)-origin.position.y)/resolution)
                
                start_x = int((robot_x-origin.position.x)/resolution)
                start_y = int((robot_y-origin.position.y)/resolution)
            
                traversed = self.raytrace([start_x,start_y],[x,y])
                in_bounds=True
                #if grid_map[x,y]!=self.free_space:
                for point in traversed:
                    if not self.is_in_bounds(grid_map,point[0],point[1]):
                        in_bounds=False
                        break
                    elif grid_map.__getitem__(point)==self.boundary:
                        in_bounds=False
                        break
                    else:
                        self.add_to_map(grid_map,point[0],point[1],self.free_space)
                        if point[0]>max_x:
                            max_x=point[0]
                        if point[0]<min_x:
                            min_x=point[0]
                        if point[1]>max_y:
                            max_y=point[1]
                        if point[1]<min_y:
                            min_y=point[1]
                #if grid_map[x,y]!=self.free_space:
                if in_bounds and self.is_in_bounds(grid_map,x,y):
                    #occupied.append([x,y])
                    self.add_to_map(grid_map,x,y,self.occupied_space)
                    if x>max_x:
                        max_x=x
                    if x<min_x:
                        min_x=x
                    if y>max_y:
                        max_y=y
                    if y<min_y:
                        min_y=y
        '''for p in occupied:
            self.add_to_map(grid_map,p[0],p[1],self.occupied_space)'''
            
                

        """
        For C only!
        Fill in the update correctly below.
        """ 
        # Only get the part that has been updated
        update = None
        if max_x!=0 and max_y!=0 and min_x!=grid_map.get_width() and min_y!=grid_map.get_height():
            update = OccupancyGridUpdate()
            update.header = Header()
            update.header.frame_id=scan.header.frame_id
            update.header.stamp = scan.header.stamp
            # The minimum x index in 'grid_map' that has been updated
            update.x = int(min_x)
            # The minimum y index in 'grid_map' that has been updated
            update.y = int(min_y)
            # Maximum x index - minimum x index + 1
            update.width = int(max_x-min_x+1)
            # Maximum y index - minimum y index + 1
            update.height = int(max_y-min_y+1)
            # The map data inside the rectangle, in row-major order.
            update.data = []

            for j in range(min_y, max_y+1):
                for i in range(min_x,max_x+1):
                    #if self.is_in_bounds(grid_map,i,j):
                    update.data.append(grid_map.__getitem__([i,j]))

        # Return the updated map together with only the
        # part of the map that has been updated
        return grid_map, update

    def inflate_map(self, grid_map):
        """For C only!
        Inflate the map with self.c_space assuming the robot
        has a radius of self.radius.
        
        Returns the inflated grid_map.

        Inflating the grid_map means that for each self.occupied_space
        you calculate and fill in self.c_space. Make sure to not overwrite
        something that you do not want to.


        You should use:
            self.c_space  # For C space (inflated space).
            self.radius   # To know how much to inflate.

            You can use the function add_to_map to be sure that you add
            values correctly to the map.

            You can use the function is_in_bounds to check if a coordinate
            is inside the map.

        :type grid_map: GridMap
        """


        """
        Fill in your solution here
        """

        width= int(grid_map.get_width())
        height = int(grid_map.get_height())

        for i in range(width):
            for j in range(height):
                if grid_map[i,j]==self.occupied_space or grid_map[i,j]==self.boundary:
                    indices = self.find_radius_ind(width,height, [i,j] ,self.radius)
                
                    for ind in indices:
                        if self.is_in_bounds(grid_map,ind[0],ind[1]) and grid_map[ind[0],ind[1]]!=self.occupied_space:
                            self.add_to_map(grid_map,ind[0],ind[1],self.c_space)
            
        
        # Return the inflated map
        return grid_map

    def find_radius_ind(self,width,height,center,radius):
        indices= np.indices([width,height]).reshape(2,-1).T
        return indices[np.square(indices-np.array(center)).sum(1)<=radius**2]