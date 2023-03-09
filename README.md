# sasa---stt_nav
Read file "guida - LEGGI". 
I had problem to read the right topic, but I think you can run the code and check simply for rover's behavior


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
