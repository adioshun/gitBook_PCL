# [PCLVisualizer](http://pointclouds.org/documentation/tutorials/pcl_visualizer.php)

CloudViewer 보다 다양한 기능을 가진 뷰어 
- playing normals, 
- drawing shapes 
- multiple viewport


설치 : `sudo apt install pcl-tools`




pcl_viewer: a quick way for visualizing PCD (Point Cloud Data) files. More information about PCD files can be found in the PCD file format tutorial.

Syntax is: pcl_viewer <file_name 1..N>.<pcd or vtk> <options>, where options are:
```
adioshun@lidar0:/workspace$ pcl_viewer
The viewer window provides interactive commands; for help, press 'h' or 'H' from within the window.
Syntax is: pcl_viewer <file_name 1..N>.<pcd or vtk> <options>
  where options are:
                     -bc r,g,b                = background color
                     -fc r,g,b                = foreground color
                     -ps X                    = point size (1..64) 
                     -opaque X                = rendered point cloud opacity (0..1)
                     -shading X               = rendered surface shading ('flat' (default), 'gouraud', 'phong')
                     -position x,y,z          = absolute point cloud position in metres
                     -orientation r,p,y       = absolute point cloud orientation (roll, pitch, yaw) in radians
                     -ax n                    = enable on-screen display of XYZ axes and scale them to n
                     -ax_pos X,Y,Z            = if axes are enabled, set their X,Y,Z position in space (default 0,0,0)

                     -cam (*)                 = use given camera settings as initial view
 (*) [Clipping Range / Focal Point / Position / ViewUp / Distance / Field of View Y / Window Size / Window Pos] or use a <filename.cam> that contains the same information.

                     -multiview 0/1           = enable/disable auto-multi viewport rendering (default disabled)


                     -normals 0/X             = disable/enable the display of every Xth point's surface normal as lines (default disabled)
                     -normals_scale X         = resize the normal unit vector size to X (default 0.02)

                     -pc 0/X                  = disable/enable the display of every Xth point's principal curvatures as lines (default disabled)
                     -pc_scale X              = resize the principal curvatures vectors size to X (default 0.02)

                     -immediate_rendering 0/1 = use immediate mode rendering to draw the data (default: disabled)
                                                Note: the use of immediate rendering will enable the visualization of larger datasets at the expense of extra RAM.
                                                See http://en.wikipedia.org/wiki/Immediate_mode for more information.
                     -vbo_rendering 0/1       = use OpenGL 1.4+ Vertex Buffer Objects for rendering (default: disabled)
                                                Note: the use of VBOs will enable the visualization of larger datasets at the expense of extra RAM.
                                                See http://en.wikipedia.org/wiki/Vertex_Buffer_Object for more information.

                     -use_point_picking       = enable the usage of picking points on screen (default disabled)

                     -optimal_label_colors    = maps existing labels to the optimal sequential glasbey colors, label_ids will not be mapped to fixed colors (default disabled)
```
(Note: for multiple .pcd files, provide multiple -{fc,ps,opaque} parameters; they will be automatically assigned to the right file)

Usage example:

> pcl_viewer -multiview 1 {AAA.pcd} {BBB.pcd} {CCC.pcd}

The above will load the partial_cup_model.pcd file 3 times, and will create a multi-viewport rendering (-multiview 1).
