#node that sends position to reach for move_base
#if position is an obstacle, send a new position in a radius around the current position
#check if robot is stuck
# when a goal is reached, send a new goal
#not used in the project yet, to be improved
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid,Odometry
import numpy as np
from itertools import product

import math
import random


class NavigationManager(object):
    def __init__(self, topic):
        # OccupancyGrid starts on lower left corner
        self._pose_data = None
        self._pose_metadata = None
        self._reference_frame = None
        self._sub = rospy.Subscriber(topic, Odometry,
                                     self._pose_cb,
                                     queue_size=1)
        self.goal=None
        
        self.distance=0
        self.prev_distance=0
        #how mmuch distance to consider as not stuck
        self.threshold=0.5
        self.max_stuck_time=60
        #start timer
        self.stuck_time=rospy.get_time()

        self.startPose=None

        print("Waiting for '" +
                      str(self._sub.resolved_name) + "'...")
        while self._pose_metadata is None and \
                self._pose_data is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        print("NavigationManager for '" +
                      str(self._sub.resolved_name) +
                      "' initialized!")

    def _pose_cb(self, data):
        self._pose_data = data
        self._pose_metadata = data.header  
        self._reference_frame = data.header.frame_id

        if not self.startPose: self.startPose=data

        #distance between the robot pose and the goal
        if self.distance: self.prev_distance=self.distance

        if self.goal:
            pose_x=self._pose_data.pose.pose.position.x
            pose_y=self._pose_data.pose.pose.position.y
            goal_x=self.goal.target_pose.pose.position.x
            goal_y=self.goal.target_pose.pose.position.y

            self.distance= math.sqrt((pose_x-goal_x)**2+(pose_y-goal_y)**2)

            #robot not stuck 
            if self.prev_distance:
                if abs(self.prev_distance-self.distance)>self.threshold:
                    print("stuck time: ",abs(rospy.get_time()-self.stuck_time))
                    self.stuck_time=rospy.get_time()
                
                #if robot stuck for some time, do something
                if abs(rospy.get_time()-self.stuck_time)>self.max_stuck_time:
                    print("ROBOT IS STUCK!!")  

    def set_goal(self, goal):
        self.goal=goal  
    
    def get_position(self):
        if self._pose_data is None:
            return None
        x=self._pose_data.pose.pose.position.x
        y=self._pose_data.pose.pose.position.y
        return (x,y)
    
    def get_stuck_time(self):
        return abs(rospy.get_time()-self.stuck_time)
   

class OccupancyGridManager(object):
    def __init__(self, topic, subscribe_to_updates=False):
        # OccupancyGrid starts on lower left corner
        self._grid_data = None
        self._occ_grid_metadata = None
        self._reference_frame = None
        self._sub = rospy.Subscriber(topic, OccupancyGrid,
                                     self._occ_grid_cb,
                                     queue_size=1)
        self.goal=None
        print("Waiting for '" +
                      str(self._sub.resolved_name) + "'...")
        while self._occ_grid_metadata is None and \
                self._grid_data is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        print("OccupancyGridManager for '" +
                      str(self._sub.resolved_name) +
                      "' initialized!")
        print("Height (y / rows): " + str(self.height) +
                      ", Width (x / columns): " + str(self.width) +
                      ", starting from bottom left corner of the grid. " + 
                      " Reference_frame: " + str(self.reference_frame) +
                      " origin: " + str(self.origin))


    @property
    def resolution(self):
        return self._occ_grid_metadata.resolution

    @property
    def width(self):
        return self._occ_grid_metadata.width

    @property
    def height(self):
        return self._occ_grid_metadata.height

    @property
    def origin(self):
        return self._occ_grid_metadata.origin

    @property
    def reference_frame(self):
        return self._reference_frame

    def set_goal(self,goal):
        self.goal=goal
        self.update_goal()

    def get_goal_coord(self):
        x=self.goal.target_pose.pose.position.x
        y=self.goal.target_pose.pose.position.y
        return x,y

    #update occupancy grid and goal when new map received
    def _occ_grid_cb(self, data):
        #print("Got a full OccupancyGrid update")
        self._occ_grid_metadata = data.info
        # Contains resolution, width & height
        # np.set_printoptions(threshold=99999999999, linewidth=200)
        # data comes in row-major order http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
        # first index is the row, second index the column
        self._grid_data = np.array(data.data,
                                   dtype=np.int8).reshape(data.info.height,
                                                          data.info.width)
        self._reference_frame = data.header.frame_id

        if self.goal:
            self.goal=self.update_goal()
        # print(self._grid_data)

    #world coordinates from costmap cells
    def get_world_x_y(self, costmap_x, costmap_y):
        world_x = costmap_x * self.resolution + self.origin.position.x
        world_y = costmap_y * self.resolution + self.origin.position.y
        return world_x, world_y

    #costmap cells from world coordinates
    def get_costmap_x_y(self, world_x, world_y):
        costmap_x = int(
            round((world_x - self.origin.position.x) / self.resolution))
        costmap_y = int(
            round((world_y - self.origin.position.y) / self.resolution))
        return costmap_x, costmap_y

    def get_cost_from_world_x_y(self, x, y):
        cx, cy = self.get_costmap_x_y(x, y)
        try:
            return self.get_cost_from_costmap_x_y(cx, cy)
        except IndexError as e:
            raise IndexError("Coordinates out of grid (in frame: {}) x: {}, y: {} must be in between: [{}, {}], [{}, {}]. Internal error: {}".format(
                self.reference_frame, x, y,
                self.origin.position.x,
                self.origin.position.x + self.height * self.resolution,
                self.origin.position.y,
                self.origin.position.y + self.width * self.resolution,
                e))

    def get_cost_from_costmap_x_y(self, x, y):
        if self.is_in_gridmap(x, y):
            # data comes in row-major order http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
            # first index is the row, second index the column
            return self._grid_data[y][x]
        else:
            raise IndexError(
                "Coordinates out of gridmap, x: {}, y: {} must be in between: [0, {}], [0, {}]".format(
                    x, y, self.height, self.width))

    def is_in_gridmap(self, grid_x, grid_y):
        if -1 < grid_x < self.width and -1 < grid_y < self.height:
            return True
        else:
            return False

    def is_in_map(self,world_x,world_y):
        i,j=self.get_costmap_x_y(world_x,world_y)
        return self.is_in_gridmap(i,j)

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
        return self._get_closest_cell_arbitrary_cost(
            x, y, cost_threshold,min_radius, max_radius, bigger_than=False)

    def update_goal(self,min_radius=3,max_radius=5000):
        goal=self.goal
        goal_x=goal.target_pose.pose.position.x
        goal_y=goal.target_pose.pose.position.y
        if self.get_cost_from_world_x_y(goal_x,goal_y)>0:
            print("the goal {:.2f},{:.2f} is an obstacle, search for near alternative...".format(goal_x,goal_y))
            cell_x,cell_y=self.get_costmap_x_y(goal_x,goal_y)

            new_x,new_y,_=self.get_closest_cell_under_cost( cell_x, cell_y, 1,min_radius, max_radius)
            if new_x<0 and new_y<0:
                print("feasible goal not found")
                return None
            new_goal_x,new_goal_y= self.get_world_x_y(new_x,new_y)
            goal.target_pose.pose.position.x=new_goal_x
            goal.target_pose.pose.position.y=new_goal_y

            dist= ( (goal_x-new_goal_x)**2 +  (goal_y-new_goal_y)**2)**0.5
            print("new goal found: {:.2f},{:.2f}\n\t at distance {:.2f} from previous goal".format(
                new_goal_x,new_goal_y,dist))

            self.goal=goal
            return goal
        #goal not obstacle
        self.goal=goal
        return goal

    def check_goal(self):
        goal=self.goal
        goal_x=goal.target_pose.pose.position.x
        goal_y=goal.target_pose.pose.position.y
        if self.get_cost_from_world_x_y(goal_x,goal_y)>0: 
            print("The goal {:.2f},{:.2f} is an obstacle".format(goal_x,goal_y))
            return False
        print("The goal {:.2f},{:.2f} is OK!".format(goal_x,goal_y))
        return True

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
        return self._get_closest_cell_arbitrary_cost(
            x, y, cost_threshold, max_radius, bigger_than=True)

    def _get_closest_cell_arbitrary_cost(self, x, y,
                                         cost_threshold,min_radius=1, max_radius=100000,
                                         bigger_than=False):

        # Check the actual goal cell
        try:
            cost = self.get_cost_from_costmap_x_y(x, y)
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
                cost = self.get_cost_from_costmap_x_y(x + tmp_x, y + tmp_y)
            # If accessing out of grid, just ignore
            except IndexError:
                pass
            if bigger_than:
                if cost > cost_threshold:
                    return x + tmp_x, y + tmp_y, cost

            else:
                if cost < cost_threshold:
                    return x + tmp_x, y + tmp_y, cost

        return -1, -1, -1


def get_new_goal(goal_x,goal_y,radius=1,angle=0):
    #return a new goal in a radius around the current goal
    #get random angle
    
    #get point on circumference
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
    while ogm.get_cost_from_world_x_y(goal_x,goal_y)>0:
        #print("costmap value",ogm.get_cost_from_world_x_y(goal_x,goal_y))

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


            if ogm.get_cost_from_world_x_y(goal_x,goal_y)<=0: break
        
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
    ogm = OccupancyGridManager('/rtabmap/grid_map')
    #ogm=OccupancyGridManager("/move_base/local_costmap/costmap")
    nvm= NavigationManager("/odom")

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    
    #waypoints to reach
    waypoints=[
        [-0,34, -16,70]
        [9.822, 0.231],
        [9.692, 3.729],
        [33.932, 2.475]]
    WAYPOINT_REACHED=False
    w_idx=0
    while w_idx < len(waypoints):
        waypoint=waypoints[w_idx]
        #print
        
        
        cell_x,cell_y=ogm.get_costmap_x_y(waypoint[0],waypoint[1])

        if not ogm.is_in_gridmap(cell_x, cell_y): 
            print("WAYPOINT {:.2f},{:.2f} OUTSIDE OF MAP! Search for nearest one".format(waypoint[0],waypoint[1]))
            #this has to be reached yet, current one will change
            waypoints.append(waypoint)
        while not ogm.is_in_map(waypoint[0], waypoint[1]): 
            #create intermediate waypoint
            waypoint=get_intermediate_waypoint(nvm.get_position(),waypoint,0.5)
            print("intermediate waypoint: ",waypoint)
        
        print("\nREACHING WAYPOINT: " + str(waypoint))
        goal_x,goal_y=waypoint

        #search for a feasible goal
        #goal=search_feasible_goal(ogm,goal_x,goal_y)

        
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
        #print("sending goal: {} {}".format(goal_x,goal_y))
        ogm.check_goal()

        client.send_goal(goal)

        while not client.wait_for_result(rospy.Duration.from_sec(1.0)):

            #print("Waiting for result...")
            goal_x=goal.target_pose.pose.position.x
            goal_y=goal.target_pose.pose.position.y

            #check fi goal has been updated
            if ogm.get_goal_coord() != (goal_x,goal_y):
                print("goal has been changed")
                client.send_goal(ogm.goal)
                nvm.set_goal(ogm.goal)

            '''
            #custom search for goal
            value=ogm.get_cost_from_world_x_y(goal_x,goal_y)
            if value>0:
                print("goal not valid")
                goal=search_feasible_goal(ogm,goal_x,goal_y)
                nvm.set_goal(goal)
                print("new_goal:",goal.target_pose.pose.position.x,goal.target_pose.pose.position.y)
            #print("check for costmap update: ", goal_x, goal_y,"value: ", value)
            '''
        
        #check if waypoint reached
        distance= nvm.distance
        if distance<2.5:
            print("GOAL REACHED! distance from goal:",distance)
            WAYPOINT_REACHED=True
            #proceed with next waypoint
            w_idx+=1
        else:
            print("GOAL NOT REACHED!! distance from goal is:",distance)
            #remain on current keypoint
        
        if nvm.get_stuck_time()>30:
            client.send_goal(ogm.goal)
            nvm.set_goal(ogm.goal)

        #if reached waypoint do an operation
        if WAYPOINT_REACHED:
            print("\n\nPERFORMING AN OPERATION\n\n")
            rospy.sleep(10)
            #Do operation


        
    print("NAVIGATION TASK FINISHED")
    print("waypoits reached:",waypoints)



if __name__ == '__main__':
    print("Starting navigation task...")
    main()
    rospy.spin()

