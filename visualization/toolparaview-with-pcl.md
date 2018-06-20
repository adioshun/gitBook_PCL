https://www.paraview.org/



[ParaView/PCL Plugin/Download And Build Instructions](https://www.paraview.org/Wiki/ParaView/PCL_Plugin/Download_And_Build_Instructions) : ParaView 3.14.1 + PCL Plugin v1.0


[ROS Manual](http://wiki.ros.org/Industrial/Tutorials/PCLParaview): Paraview 3.12 + PCL Plugin 1.0 (PCL 1.5)

A plugin to enable PCL functionality in ParaView: [Paraview 4.1](https://www.paraview.org/paraview-downloads/download.php?submit=Download&version=v4.1&type=binary&os=Linux&downloadFile=ParaView-4.1.0-Linux-64bit-glibc-2.3.6.tar.gz) + [PCL Plugin v1.1](https://github.com/Kitware/PCLPlugin) (PCL = 1.5.1)



[ROS Manual](http://wiki.ros.org/Industrial/Tutorials/PCLParaview): Paraview 3.12 + PCL Plugin 1.0 (PCL 1.5)



---

  ParaView v4.1.0
  PCL v1.5.1 and current master

paraview 다운로드 : https://www.paraview.org/download/


tar xvfz ParaView-3.14.1-Source.tar.gz

$cd /home/ParaView-3.12.0 
$mkdir build 
$cd build 
$ccmake .. #OR $cmake-gui Make sure that BUILD_SHARED_LIBS is set to ON Configure and generate files 
$make 


The executable paraview file will be in /home/ParaView-3.12.0/build/bin 


