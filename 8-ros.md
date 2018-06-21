You can also use this library with ROS sensor_msgs, but it is not a dependency. You don't need to install this package with catkin -- using pip should be fine -- but if you want to it is possible:

Steps:
```
# you need to do this manually in this case
pip install python-lzf
cd your_workspace/src
git clone https://github.com/dimatura/pypcd
mv setup_ros.py setup.py
catkin build pypcd
source ../devel/setup.bash
```
Then you can do something like this:
```
import pypcd
import rospy
from sensor_msgs.msg import PointCloud2


def cb(msg):
    pc = PointCloud.from_msg(msg)
    pc.save('foo.pcd', compression='binary_compressed')
    # maybe manipulate your pointcloud
    pc.pc_data['x'] *= -1
    outmsg = pc.to_msg()
    # you'll probably need to set the header
    outmsg.header = msg.header
    pub.publish(outmsg)

# ...
sub = rospy.Subscriber('incloud', PointCloud2)
pub = rospy.Publisher('outcloud', PointCloud2, cb)
rospy.init('pypcd_node')
rospy.spin()
```