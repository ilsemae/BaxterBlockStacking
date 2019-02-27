Code for Baxter block stacking.

In your first terminal:
	. ~/ros_ws/devel/setup.bash

then
	roslaunch group7_proj1 proj1.launch total_blocks:=5 configu:=stacked_ascending gripper_loc:=5 real_or_sym:=[sym or baxter] hand:=[left, right, or both]


You should now see periodic updates of the world state in this window.

Now open another terminal, and run the controller by typing:

	run rostopic pub /command group7_proj1/Command [Open OR Close OR MoveTo OR MoveOver OR Scatter OR Stack_Ascending OR Stack_Descending] [int target_block]
