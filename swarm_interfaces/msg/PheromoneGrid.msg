# Феромонная карта арены, flat row-major
std_msgs/Header header
float32 origin_x           # мировые координаты левого-нижнего угла (NED), м
float32 origin_y
float32 cell_size          # размер клетки, м
uint16 width               # число клеток по X
uint16 height              # число клеток по Y
float32[] data             # width*height значений [0..1], row-major