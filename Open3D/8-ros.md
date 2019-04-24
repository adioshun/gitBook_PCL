# [Open3D-ROS 연동 ](https://github.com/karaage0703/open3d_ros)

- [How to set up Open3D and ROS linkage by "open3d_ros](https://karaage.hatenadiary.jp/entry/2018/03/12/073000): 중간 부분 부터 

- [Util.py](https://gist.github.com/adioshun/17dd3e4f4351bf422830282781a644b7)
    
    
    
convert_pcl(data):
- input 
    - data : pointcloud2 topic
- output : pcd

publish_pointcloud(output_data, input_data)
- input 
    - output_data : 송신 point cloud topic
    - input_data : 