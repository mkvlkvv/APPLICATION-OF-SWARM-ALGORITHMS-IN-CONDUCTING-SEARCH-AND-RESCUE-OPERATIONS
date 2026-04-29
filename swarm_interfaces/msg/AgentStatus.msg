uint8 agent_id
uint8 state              # 0=IDLE, 1=SEARCHING, 2=TRACKING, 3=RTL, 4=DEAD
geometry_msgs/Point position
float32 battery
uint8[] assigned_zone_ids
builtin_interfaces/Time stamp