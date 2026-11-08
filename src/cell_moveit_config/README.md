# cell_moveit_config

Hand-written reference of what the MoveIt Setup Assistant generates in Ch 6 for `cell_control.urdf.xacro`.
Generating your own with the assistant is the exercise; use this package to compare, or as a fallback.
The self-collision list in `config/cell.srdf` only disables pairs that are adjacent or can never touch;
the assistant samples thousands of poses and usually disables more, which only makes planning faster.
