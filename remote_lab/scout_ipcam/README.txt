*Authors*

IPCAM node was created by Rodrigo Ventura and adapted by Joao Mendes. If any problem is found please contact mendes.joao.p@gmail.com


*Purpose*

To aquire data from an ipcamera and publish it on ROS. This node acts as a simple webgrabber that publishes the aquired images on a topic.


*Running*

This node was tested on the following camera models: Vivotek FE8174 and FE8171v, and Axis P1344. Small changes may have to occur in different models.
A launch file (streamer_v2.launch) is already provided and expects two args (ip,stream_num) respectively the IP of the camera and the desired stream (considering that the camera can stream more that one more simultaneously). Notice that the range of IPs where the cameras are working  is hardcoded in the launch.

An example of how streamer_v2.launch can be used is found on main_monarch_iw.launch (used throughout the monarch IW in IST). 

In order to run the code:

- cd /monarch/code/trunk/rosbuild_ws/ipcam
- roslaunch main_monarch_iw.launch 

2 nodes will be launched for camera 92 and 93. Both on stream 3. Check the launch to change the IP or stream of the camera.

Please notice that along with the driver a republisher is also launched. If your software does not required the republisher please edit the launch to reduce computational effort. 


* nodes.yaml *

For each set of desired IP and stream a node is launched. The names are /streamer_XX_Y where XX is the last digits of the IP and Y is the number of the stream.

for example: camera with IP 10.10.2.99 publishing stream 3 would be:
/streamer_99_3

similarly the republisher will be named:

/republisher_99_3


* topics.yaml *

The launched topics also follow the same set of name, i.e., /camera_driver_XX_Y/(...)

Where, for exaple, /camera_driver_99_3/stream/compressed is the topic published by /streamer_99_3


** type of message ** 

The type of the published message is sensor_msgs/CompressedImage. The republisher publishes the messages described in the ros website.


