Header header
uint8 type
uint8 status
string description
geometry_msgs/Pose robot    # Global to given frame
geometry_msgs/Pose pose     # Robot relative

sensor_msgs/Image[] images
crosbot/PointCloudMsg[] clouds
#crosbot/ColouredCloudMsg[] colouredClouds