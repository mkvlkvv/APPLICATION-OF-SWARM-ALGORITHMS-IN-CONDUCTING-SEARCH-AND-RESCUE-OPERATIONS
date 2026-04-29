uint8 agent_id           # кто обнаружил
uint8 target_id          # хеш позиции или инкрементальный
geometry_msgs/Point world_position
string color_label       # "red", "blue", "green"
float32 confidence
builtin_interfaces/Time stamp