# [Open3D-ROS 연동 ](https://github.com/karaage0703/open3d_ros)

- [How to set up Open3D and ROS linkage by "open3d_ros](https://karaage.hatenadiary.jp/entry/2018/03/12/073000): 중간 부분 부터 

- [Util.py](https://gist.github.com/adioshun/17dd3e4f4351bf422830282781a644b7)


## 1. ROS to PCL

```python 
def convert_pcl(data):
    """     
    convert_pcl(data):
    - input " pointcloud2 ROS mgs.
    - output : pcd
    """
    header = '''# .PCD v0.7 - Point Cloud Data file format
    VERSION 0.7
    FIELDS x y z rgb
    SIZE 4 4 4 4
    TYPE F F F F
    COUNT 1 1 1 1
    WIDTH %d
    HEIGHT %d
    VIEWPOINT 0 0 0 1 0 0 0
    POINTS %d
    DATA ascii'''

    with open(tmp_pcd_name, 'w') as f:
        f.write(header % (data.width, data.height, data.width*data.height))
        f.write("\n")

        for p in pc2.read_points(data, skip_nans=True):
            f.write('%f %f %f %e' % (p[0], p[1], p[2], p[3]))
            f.write("\n")

        cloud_list = []
        for p in pc2.read_points(data, skip_nans=False):
            cloud_list.append(p[0])
            cloud_list.append(p[1])
            cloud_list.append(p[2])
            cloud_list.append(p[3])

        f.write("\n")

    pcd = py3d.read_point_cloud(tmp_pcd_name)

    return pcd

def callback(data):
    result_pcl = convert_pcl(data)
    print(result_pcl)

    publish_pointcloud(result_pcl, data)

    # publish_testcloud(data)

if __name__ == "__main__":
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('input',
                     PointCloud2, callback)

    rospy.spin()

```


## 2. ROS
publish_pointcloud(output_data, input_data)
- input 
    - output_data : 송신 point cloud topic
    - input_data : 