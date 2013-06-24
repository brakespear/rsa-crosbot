Header header
uint8 type
int8 status
uint32 id
string description
geometry_msgs/Pose robot    # Global to given frame
geometry_msgs/Pose pose     # Robot relative

sensor_msgs/Image[] images
crosbot/PointCloudMsg[] clouds
#crosbot/ColouredCloudMsg[] colouredClouds