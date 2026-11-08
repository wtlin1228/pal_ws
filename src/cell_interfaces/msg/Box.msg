string id                        # unique, e.g. "box_007"
string sku                       # "A", "B" ... or a barcode
geometry_msgs/PoseStamped pose   # center of the box; pose.header.frame_id names the frame
geometry_msgs/Vector3 size       # x, y, z extents [m] along the box's own axes
string location                  # "pallet" | "conveyor" | "gripper" | "unknown"
