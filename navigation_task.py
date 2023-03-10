# node that sends position to reach for move_base
# if position is an obstacle, send a new position in a radius around the current position
# check if robot is stuck, in case it active a recovery behavior.


'''
###############################
###### HOW WORK MOVE BASE #####
###############################


look at the graphic on documentation.
When I start "move_base" with rtamap started, 
I have this situation:

N.B: difference between grid_map and cost_map ( stupid explanation )
        - grid_map: cells can have only value 0 ( free ) or 100 ( occupied )
        - cost_map: cells have all integer value between 0 and 100


- User has a map in world_coordinate ( with rviz I send a goal in world_coordinate to the rover )
- Rover has a map in cell_coordinate ( grid_map / cost_map. This two map have the "same" meaning)
- global planner:   a process which calculate a path from start_position of the rover 
                    to the goal_position sent with rviz ( or with code in the main)
                    using the cost_map knew by the rover ( I'm not sure it use the cost_map, but this is the idea)
- local planner: a process the calculate costanly what the rover should do sending cmd_vel

knowing that, I thought this:
If my rover is in [8,8] and is going to [10,10].
If is stuck in cell [10,10] for unknow reason
I say to the rover to go back in a previous position, for example in [8,8] and meawhile sign
cell [10, 10] as occupied. In that way local planner ( or global planner, I don't understood) 
should calculate a new path considering that the cell [10,10] as occupied ( before was free ).
Repeat this steps if I can't find a path from [8,8], maybe going back to [6,5].

We could improve this mechanis signing as occupied all cell crossed by the rover while going from 
[8,8] to [10,10], so cell crossed before getting stuck.
In general cells cross from [10,10] to [8,8] can be different

'''

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid,Odometry
import numpy as np
from itertools import product   # for cartesian product

from collections import deque

import math
import random


# a data structure which able me to discard oldest element ( if len = 60, and I append the 61??, the first one is discarded automatically)
# when I want to add a new one into the array
# I keep there data structure outside the class in a way that, If the program
# is spinning, all times I create a new object NavigationManager with an empty structure
old_odom_msg = deque(maxlen=60)

# dizionary <coordinate cell, original value>, used to save original value of cell 
# I change value of some cells during the recovery behavior
temp_occupied_cell = {}

# This class is the one which ear the message and 
# manage the rover's behavior.
class NavigationManager(object):
    def __init__(self, topic, my_OccupancyGridManager):

        self.my_OccupancyGridManager = my_OccupancyGridManager
        self.first_state = None
        # OccupancyGrid starts on lower left corner
        self.odom_msg= None
        self.goal = None
        self.recovery_position = None   # used when robot is stuck to move it in and old position, NOT USED
        self.is_stuck = False      


        self.my_sub = rospy.Subscriber(topic, Odometry,self.my_callback_nav,queue_size=1)    
        
        self.distance=0
        self.recovery_distance = None

        self.prev_distance=0
        self.stuck_threshold=0.5      # how much distance to consider as not stuck
        self.max_stuck_time=60
        self.stuck_time=rospy.get_time()    #start timer

        print("Waiting for '" + str(self.my_sub.resolved_name) + "'...")

        while self.odom_msg is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        print("NavigationManager for '" + str(self.my_sub.resolved_name) + "' initialized!")

    # function to active when read a new message on the topic
    def my_callback_nav(self, data):

        #if first_state == None:
         #   first_state = self.odom_msg

        # I keep a record of some odometry_msg
        previous_odom_msg = self.odom_msg
        
        # I record message with a distance of 2 second, with a maxium record 2*60
        # if max_stuck_time is 60, in the last minute I have same poses, so I record last 2 minutes
        # should store states / messages every 2 seconds
        if len(old_odom_msg) > 0:
            time_distance = int( round(( old_odom_msg[-1].header.stamp.nsec - data.header.stamp.nsec )) )
        
        if len(old_odom_msg) > 0 and  time_distance >= 2.0 and self.is_stuck==False:
            self.old_odom_msg.append( previous_odom_msg )

        # self.distance it was the distance calculate of the previous msf
        # now I have to calculate the new distance
        if self.distance: self.prev_distance = self.distance

        # I put new message into the class
        self.odom_msg = data

        if self.goal:
            pose_x=self.odom_msg.pose.pose.position.x
            pose_y=self.odom_msg.pose.pose.position.y
            goal_x=self.goal.target_pose.pose.position.x
            goal_y=self.goal.target_pose.pose.position.y

            self.distance= math.sqrt((pose_x-goal_x)**2+(pose_y-goal_y)**2)

            # robot not stuck and I recived at least 2 msg
            # I consider my robot stuck-state if the difference is not changed
            # of a certain threshold between two message
            if self.prev_distance:
                
                if abs(self.prev_distance-self.distance)>self.stuck_threshold:
                    print("stuck time: ",abs(rospy.get_time()-self.stuck_time))
                    self.stuck_time=rospy.get_time()
                
                #if robot stuck for some time, do something
                if abs(rospy.get_time()-self.stuck_time)>self.max_stuck_time:
                    print("ROBOT IS STUCK!!")  
                    
                    self.is_stuck = True
                    self.recovery_behavior()
                    self.is_struck = False

    def set_recovery_goal(self, recovery_goal):
        self.recovery_goal = recovery_goal

    def set_goal(self, goal):
        self.goal=goal  
    
    def get_position(self):
        if self.odometry_msg is None:
            return None
        x=self.odometry_msg.pose.pose.position.x
        y=self.odometry_msg.pose.pose.position.y
        return (x,y)
    
    def get_stuck_time(self):
        return abs(rospy.get_time()-self.stuck_time)
   
    def set_recovery_position(self, recovery_position):
        self.recovery_position = recovery_position

    def set_recovery_distance(self, recovery_distance):
        self.recovery_distance = recovery_distance

    # a simply clear
    def clear_recovery_state(self):
        self.recovery_position = None
        self.recovery_distance = None
        self.is_struck = False

    # main function for recovery behavior
    def recovery_behavior(self):
        '''
        # start a routine which examine all old_odom_msg
        # starting from the most recent

        # I must set self.recovery_goal = old_odom_msg.pose
        # with the right structure
        # and say to rover to go there and try a new path

        # one idea: If my robot is stuck and can't reach the goal
        # I can try to analize the robot's state 10 second before and look for another point
        # or for another path.
        # In the first case, I can select a point near the goal randomly.
        # In the second case, I can try to find a new path without crossing
        # cells bring the robot in stuck-state

        # If one of two cases is succesfull, 
        # I say to rover to go in (x,y) of the previous state and execute the action
        # Else I can back 15 second before and repeat everything.

        # If nothing work -> manual command
        # or
        # Go back in the first state ( where rover start )
        # and make new a new plan without crossing cells that
        # he already crossed if this cells are 2 meters near
        # to the cell that bring him to stuck-state
        '''

        # If I have recorded 120 sec and my stuck-time is 60 sec
        # half of message have the same position / state 

        print("original goal: {}".format(self.goal))

        effective_len = len(old_odom_msg) / 2
        recovery_result = False
        for i in range( round( effective_len )):
            
            # I start from the most recent message
            msg = old_odom_msg[effective_len - i]   

            pos_x = msg.pose.pose.position.x
            pos_y = msg.pose.pose.position.y

            # old_state is the previous position where robot should go
            previous_position = self.my_OccupancyGridManager.world2grid(pos_x, pos_y)

            print("previous_pose: {}".format(previous_position))
            result = self.send_recovery_position(previous_position)
            
            if result == True:
                print("recovery position procedure: SUCCESS")
            else:
                print("recovery position procedure: FAILED")

            # save original value of the cell into the dictionary and set it as occupied
            # to let the planner find a new path without the crossed cell
            my_grid_cell = self.my_OccupancyGridManager[previous_position[0]][previous_position[1]]
            temp_occupied_cell[previous_position] = my_grid_cell

            # Do not assign a cell value over 100
            if my_grid_cell < 50:
                my_grid_cell += 50
            else:
                my_grid_cell = 100

            if result == False:
                continue
            else:
                recovery_result = True
                break
            # now I must sign as occupied cells, all cells which are crossed
            # by the rover between actual_position and old_state

        # restore original value

        # if I robot is no longer stuck -> find a new path
        if recovery_result == True:

            # I say to the planner to calculate a path to the original goal
            print("Recovery result success! Going to: ".format(self.goal))
            self.send_recovery_position(self.goal)
        
        else:
            print("!!! NOTHING WORKED -> go in manual !!!")

        #### IMPORTANT ###
        # create a mechanism to understand if robot is already stuck
        self.clear_recovery_state()
        print("resuming.")
        return

    # auxiliary function for recovery_behavior
    # it send the new point to be reached ( at least should send it ) 
    def send_recovery_position(self, recovery_position):

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        ogm = self.my_OccupancyGridManager

        print("\trecovery position: {}".format(recovery_position))

        goal_x = recovery_position[0]
        goal_y = recovery_position[1]
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(goal_x,goal_y,0.0)
        goal.target_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    
        ogm.set_recovery_position(goal)   
        self.set_recovery_position(goal)

        #print goal
        goal_x=goal.target_pose.pose.position.x
        goal_y=goal.target_pose.pose.position.y

        print("\tsending new point to reach: {}".format(goal))
        client.send_goal(goal)

        # this cicle wait the rover to reach the position "goal" ?
        while not client.wait_for_result(rospy.Duration.from_sec(1.0)):

            #print("Waiting for result...")
            goal_x = goal.target_pose.pose.position.x
            goal_y = goal.target_pose.pose.position.y
            
        
        #check if waypoint reached
        actual_pose_x = self.origin.position.x
        actual_pose_y = self.origin.position.y

        recovery_distance = math.sqrt( (goal_x - actual_pose_x)**2 + (goal_y - actual_pose_y)**2 )

        if recovery_distance < 2.5:
            print("\tPOINT REACHED! distance from point:",recovery_distance)
            print("\tLet the planner calculate a new path ... ")
            return True
        else:
            print("\tPOINT NOT REACHED!! distance from point is:",recovery_distance)
            print("\t!! NOT ABLE RO REACH PREVIOUS POSITION -> trying another older position !!!")
            return False


# This class is the one that modify the GridMap
class OccupancyGridManager(object):
    def __init__(self, topic, subscribe_to_updates=False):
        # OccupancyGrid starts on lower left corner
        self.occ_grid_msg = None
        self.grid_map = None
        self.metadata = None
        self.sub_occ_grid = rospy.Subscriber(topic, OccupancyGrid,
                                     self.my_callback_occ,
                                     queue_size=1)
        self.goal=None
        self.recovery_position = None

        print("Waiting for '" +
                      str(self.sub_occ_grid.resolved_name) + "'...")
        
        while self.metadata is None and \
                self.grid_map is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

    @property
    def resolution(self):
        return self.metadata.resolution

    @property
    def width(self):
        return self.metadata.width

    @property
    def height(self):
        return self.metadata.height

    @property
    def origin(self):
        return self.metadata.origin

    @property
    def reference_frame(self):
        return self.occ_grid_msg.header.frame_id

    def set_goal(self,goal):
        self.goal=goal
        self.update_goal()

    def set_recovery_position(self, recovery_position):
        self.recovery_position = recovery_position

    def get_goal_coord(self):
        x=self.goal.target_pose.pose.position.x
        y=self.goal.target_pose.pose.position.y
        return x,y

    #function called when ear a message: update occupancy grid and goal when new map received
    def my_callback_occ(self, data):
        
        # save whole message
        self.occ_grid_msg = data

        # map_load_time, resolution, width, height, origin (geometry_msgs/Pose)
        self.metadata = data.info

        # np.set_printoptions(threshold=99999999999, linewidth=200)
        # data comes in row-major order http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
        # first index is the row, second index the column

        #self.grid_map = np.array(data.data,dtype=np.int8).reshape(data.info.height,data.info.width)

        self.grid_map = data.data

        if self.goal:
            self.goal=self.update_goal()

        print("[OGM] width: {}, height: {}, origin: [{}, {}], resolution: {}".format(self.width, self.height, self.origin.position.x, self.origin.position.y, self.metadata.resolution))

    #world coordinates from costmap cells
    def cost2world(self, costmap_x, costmap_y):
        world_x = costmap_x * self.resolution + self.origin.position.x
        world_y = costmap_y * self.resolution + self.origin.position.y
        return world_x, world_y

    #costmap cells from world coordinates
    def world2cost(self, world_x, world_y):
        costmap_x = int(round((world_x - self.origin.position.x) / self.resolution))
        costmap_y = int(round((world_y - self.origin.position.y) / self.resolution))
        print("[DEBUG] world2cost: [{}. {}] -> [{}, {}]".format(world_x, world_y, costmap_x, costmap_y))
        
        costmap_x = int( round(world_x/self.resolution) )
        costmap_y = int( round(world_y/self.resolution) )

        print("[DEBUG] world2cost: [{}. {}] -> [{}, {}]".format(world_x, world_y, costmap_x, costmap_y))
        return costmap_x, costmap_y

    # x, y are in grid_map coordinate
    def get_cost_point(self,grid_x,grid_y):
        if self.is_in_gridmap(grid_x,grid_y):
            return self.grid_map[grid_x*self.width + grid_y]
        else:
            return None
    
    # x, y are in grid coordinate
    def get_world_point(self,grid_x,grid_y):
        world_x, world_y = self.cost2world(grid_x, grid_y)
        if self.is_in_map(world_x, world_y):
            return world_x, world_y
        else:
            return None

    # given a point in the world map, return his value in the costmap ( cell value )
    def get_cell_value_from_world_x_y(self, x, y):
        cx, cy = self.world2cost(x, y)
        print("[OGM] wordl: [{}, {}] -> cost: [{},{}]".format(x,y,cx,cy))
        try:
            return self.get_cell_value_from_costmap_x_y(cx, cy)
        except IndexError as e:
            raise IndexError("\nCoordinates out of grid (in frame: {}) x: {}, y: {} must be in between: [{}, {}], [{}, {}]. Internal error: {}".format(
                self.reference_frame, x, y,
                self.origin.position.x,
                self.origin.position.x + self.height * self.resolution,
                self.origin.position.y,
                self.origin.position.y + self.width * self.resolution,
                e))

    # given a point in the costmap / gridmap, return his value in the costmap ( cell value )
    def get_cell_value_from_costmap_x_y(self, x, y):
        if self.is_in_gridmap(x, y):
            # data comes in row-major order http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
            # first index is the row, second index the column
            print("[OGM] [{},{}] is in grid map".format(x,y))
            return self.grid_map[x*self.width + y]      # should not be [x][y] ? x = rows, y = cols?
        else:
            raise IndexError(
                "\n[OGM] Coordinates out of gridmap, x: {}, y: {} must be in between: [0, {}], [0, {}]".format(
                    x, y, self.height, self.width))
    

    def is_in_gridmap(self, grid_x, grid_y):
        if -1 < grid_x < self.width and -1 < grid_y < self.height:
            return True
        else:
            return False

    def is_in_map(self,world_x,world_y):
        i,j=self.get_costmap_x_y(world_x,world_y)
        return self.is_in_gridmap(i,j)


    # when a goal for Occupacy grid is setted, this function will search for
    # another point if the goal-point is an obstacle
    def update_goal(self,min_radius=3,max_radius=5000):

        goal=self.goal
        goal_x=goal.target_pose.pose.position.x
        goal_y=goal.target_pose.pose.position.y

        # check if the goal is an obstacle
        goal_value = self.get_cell_value_from_world_x_y(goal_x,goal_y)
        if goal_value > 0:
            print("the goal {:.2f},{:.2f} is an obstacle, search for near alternative...".format(goal_x,goal_y))

            # I extract coordinate in the costmap from the goal-coordinate expressed in world coordinate
            cell_x,cell_y=self.world2cost(goal_x,goal_y)

            # search for a new cell in a radial way from the goal-coordiante cell 
            new_x,new_y,_=self.get_closest_cell_under_cost( cell_x, cell_y, 1,min_radius, max_radius)

            if new_x<0 and new_y<0:
                print("!!! feasible goal not found !!!")

                # I set as goal the first position of the rover, maybe wrong because
                # may be repeat same steps that bring to robot to stuck-state
                goal.target_pose.pose.position.x = first_state.pose.pose.position.x
                goal.target_pose.pose.position.y = first_state.pose.pose.position.y
                return None
            
            # I convert goal-coordinate-cell in world coordinate and set it as goal
            new_goal_x,new_goal_y= self.cost2world(new_x,new_y)
            goal.target_pose.pose.position.x=new_goal_x
            goal.target_pose.pose.position.y=new_goal_y

            dist= math.sqrt( (goal_x-new_goal_x)**2 +  (goal_y-new_goal_y)**2)
            print("new goal found: {:.2f},{:.2f}\n\t at distance {:.2f} from previous goal".format(new_goal_x,new_goal_y,dist))

            self.goal=goal
            return goal
        
        # goal is not obstacle
        self.goal=goal
        return goal

    # not used
    def check_goal(self):
        goal=self.goal
        goal_x=goal.target_pose.pose.position.x
        goal_y=goal.target_pose.pose.position.y

        goal_value = self.get_cost_point(goal_x,goal_y)

        if goal_value == None:
            print("Not a right point: probably outside of the map")
        elif goal_value >0: 
            print("The goal {:.2f},{:.2f} is an obstacle".format(goal_x,goal_y))
            return False
        else:
            print("The goal {:.2f},{:.2f} is OK!".format(goal_x,goal_y))
            return True

    # this functions are called on "update_goal", no worry about them
    def get_closest_cell_under_cost(self, x, y, cost_threshold,min_radius, max_radius):
        """
        Looks from closest to furthest in a circular way for the first cell
        with a cost under cost_threshold up until a distance of max_radius,
        useful to find closest free cell.
        returns -1, -1 , -1 if it was not found.
        :param x int: x coordinate to look from
        :param y int: y coordinate to look from
        :param cost_threshold int: maximum threshold to look for
        :param max_radius int: maximum number of cells around to check
        """
        return self.get_closest_cell_arbitrary_cost(
            x, y, cost_threshold,min_radius, max_radius, bigger_than=False)
    
    def get_closest_cell_over_cost(self, x, y, cost_threshold, max_radius):
        """
        Looks from closest to furthest in a circular way for the first cell
        with a cost over cost_threshold up until a distance of max_radius,
        useful to find closest obstacle.
        returns -1, -1, -1 if it was not found.
        :param x int: x coordinate to look from
        :param y int: y coordinate to look from
        :param cost_threshold int: minimum threshold to look for
        :param max_radius int: maximum number of cells around to check
        """
        return self.get_closest_cell_arbitrary_cost(
            x, y, cost_threshold, max_radius, bigger_than=True)

    # main function to search for a new cell-point in costmap around (x,y)
    def get_closest_cell_arbitrary_cost(self, x, y,
                                         cost_threshold,min_radius=1, max_radius=100000,
                                         bigger_than=False):

        # Check the actual goal cell
        try:
            cost = self.get_cell_value_from_costmap_x_y(x, y)
        except IndexError:
            return None

        if bigger_than:
            if cost > cost_threshold:
                return x, y, cost
        else:
            if cost < cost_threshold:
                return x, y, cost

        def create_radial_offsets_coords(min_radius,max_radius):
            """
            Creates an ordered by radius (without repetition)
            generator of coordinates to explore around an initial point 0, 0
            For example, radius 2 looks like:
            [(-1, -1), (-1, 0), (-1, 1), (0, -1),  # from radius 1
            (0, 1), (1, -1), (1, 0), (1, 1),  # from radius 1
            (-2, -2), (-2, -1), (-2, 0), (-2, 1),
            (-2, 2), (-1, -2), (-1, 2), (0, -2),
            (0, 2), (1, -2), (1, 2), (2, -2),
            (2, -1), (2, 0), (2, 1), (2, 2)]
            """
            # We store the previously given coordinates to not repeat them
            # we use a Dict as to take advantage of its hash table to make it more efficient
            coords = {}
            # iterate increasing over every radius value...
            for r in range(min_radius, max_radius + 1):
                # for this radius value... (both product and range are generators too)
                tmp_coords = product(range(-r, r + 1), repeat=2)
                # only yield new coordinates
                for i, j in tmp_coords:
                    if (i, j) != (0, 0) and not coords.get((i, j), False):
                        coords[(i, j)] = True
                        yield (i, j)

        coords_to_explore = create_radial_offsets_coords(min_radius,max_radius)

        for idx, radius_coords in enumerate(coords_to_explore):
            # for coords in radius_coords:
            tmp_x, tmp_y = radius_coords
            # print("Checking coords: " +
            #       str((x + tmp_x, y + tmp_y)) +
            #       " (" + str(idx) + " / " + str(len(coords_to_explore)) + ")")
            try:
                cost = self.get_cell_value_from_costmap_x_y(x + tmp_x, y + tmp_y)
            # If accessing out of grid, just ignore
            except IndexError:
                pass
            if bigger_than:
                if cost > cost_threshold:
                    return (x + tmp_x, y + tmp_y, cost)

            else:
                if cost < cost_threshold:
                    return (x + tmp_x, y + tmp_y, cost)

        return -1, -1, -1



### just some functions ###
def get_new_goal(goal_x,goal_y,radius=1,angle=0):
    # return a new goal in a radius around the current goal
    # get random angle
    
    # get point on circumference
    x = goal_x + radius*math.cos(angle)
    y = goal_y + radius*math.sin(angle)
    return x,y

def search_feasible_goal(ogm,goal_x,goal_y):
    print("original goal: ",goal_x,goal_y)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position = Point(goal_x,goal_y,0.0)
    goal.target_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

    goal_x=goal.target_pose.pose.position.x
    goal_y=goal.target_pose.pose.position.y
    radius=0.1
    while ogm.get_cell_value_from_world_x_y(goal_x,goal_y)>0:
        #print("costmap value",ogm.get_cell_value_from_world_x_y(goal_x,goal_y))

        #try 5 points around the circle
        attempts=10
        for i in range(attempts):
            #get angle based on i
            angle = i* math.pi*2  / attempts
            goal_x,goal_y=get_new_goal(goal_x,goal_y,radius,angle)

            #update goal
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = goal_x
            goal.target_pose.pose.position.y = goal_y

            print("attempting goal: ",goal_x,goal_y)


            if ogm.get_cell_value_from_world_x_y(goal_x,goal_y)<=0: break
        
        radius+= 0.2

    print("new goal: ",goal.target_pose.pose.position.x,goal.target_pose.pose.position.y)
    

    return goal

#get x,y coordinates between two points
def get_intermediate_waypoint(w1,w2,step):
    #step=[0,1] is how much near to x1 it is
    x1=w1[0]
    y1=w1[1]
    x2=w2[0]
    y2=w2[1]
    x=x1+(x2-x1)*step
    y=y1+(y2-y1)*step
    return x,y

def main():
    rospy.init_node('navigation_goals')
    
    #ogm = OccupancyGridManager('/rtabmap/grid_map')
    ogm=OccupancyGridManager("/move_base/local_costmap/costmap")
    nvm= NavigationManager("/odom", ogm)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    waypoints = [ 
        [-15.0, 7.5],       # TEST_POINT: 2 meters away from leo initial position
        [-12.6, 7.87],      # TEST_POINT: 5 meters away from leo initial position
        [7.22, -6.36],      # for positioning
        [7.62, -11.17],     # encounter first obstacle
        [-0.34, -16,70]     # should need to recalculate another path to reach this point
          ]

    WAYPOINT_REACHED=False
    w_idx = 0
    while w_idx < len(waypoints):
        waypoint=waypoints[w_idx]
        #print
        
        
        cell_x,cell_y=ogm.world2cost(waypoint[0],waypoint[1])

        print("[MAIN] verifing: [{}, {}]".format(cell_x, cell_y))
        if not ogm.is_in_gridmap(cell_x, cell_y): 
            print("[MAIN] WAYPOINT [{:.2f},{:.2f}] OUTSIDE OF MAP! Search for nearest one".format(waypoint[0],waypoint[1]))
            #this has to be reached yet, current one will change
            waypoints.append(waypoint)
        
        
        print("[MAIN] REACHING WAYPOINT: " + str(waypoint))
        goal_x,goal_y = waypoint
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = Point(goal_x,goal_y,0.0)
        goal.target_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        ogm.set_goal(goal)
        
        nvm.set_goal(goal)

        #print goal
        goal_x=goal.target_pose.pose.position.x
        goal_y=goal.target_pose.pose.position.y

        print("[MAIN] sending goal: {} {}".format(goal_x,goal_y))
        #ogm.check_goal()

        client.send_goal(goal)

        while not client.wait_for_result(rospy.Duration.from_sec(1.0)):

            print("[MAIN] Waiting for result...")
            goal_x=goal.target_pose.pose.position.x
            goal_y=goal.target_pose.pose.position.y

            #check fi goal has been updated
            if ogm.get_goal_coord() != (goal_x,goal_y):
                print("[MAIN] goal has been changed")
                client.send_goal(ogm.goal)
                nvm.set_goal(ogm.goal)
        
        #check if waypoint reached
        distance= nvm.distance
        if distance<2.5:
            print("[MAIN] GOAL REACHED! distance from goal:",distance)
            WAYPOINT_REACHED=True
            #proceed with next waypoint
            w_idx+=1
        else:
            print("[MAIN] GOAL NOT REACHED!! distance from goal is:",distance)
            #remain on current keypoint
        
        if nvm.get_stuck_time()>30:
            client.send_goal(ogm.goal)
            nvm.set_goal(ogm.goal)

        #if reached waypoint do an operation
        if WAYPOINT_REACHED:
            print("\n\n[MAIN] PERFORMING AN OPERATION\n\n")
            rospy.sleep(10)
            
            ###### HERE SHOULD CALL THE RECOVERY BEHAVIOR OR SEND A NEW FEASIBLE GOAL ####




        
    print("[MAIN] NAVIGATION TASK FINISHED")
    print("[MAIN] waypoits reached:",waypoints)

if __name__ == '__main__':
    print("Starting navigation task...")
    main()
    rospy.spin()

