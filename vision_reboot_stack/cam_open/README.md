# Camera Opener

Uses cv-bridge and image-transport packages to capture the camera frames and publish them
on a topic. The subscriber then subsribes to this topic and shows this in a open cv window.

## Subscriber : my_subscriber.cpp
## Publisher : my_publisher.cpp

**Topics** : "camera/image"
<br>

**To Run :**
* In the image_transport_ws folder, do `$catkin_make`
* Source the setup.bash : `$ source devel/setup.bash` | While being in the image_transport_ws folder
* Run roscore : `$ roscore`
* Run the publisher to take frames from webcam and publish it to **camera/image**
* `$ rosrun image_transport_tutorial my_publisher.cpp`
* Run the subscriber to take frames from the topic **camera/image** and show it in new openCV window 
* `$ rosrun image_transport_tutorial my_publisher.cpp`
