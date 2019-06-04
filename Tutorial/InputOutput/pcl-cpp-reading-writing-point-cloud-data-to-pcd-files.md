
# [Common data structures in PCL](https://www.twblogs.net/a/5c27931ebd9eee16b3dbc3eb) 

> https://blog.csdn.net/qq_16481211/article/details/85332763#pclPCLPointCloud2_80

## 1. Template class : `Pcl::PointCloud`

member variables
- width 
    - unstructured : number of points in the cloud 
    - structured : number of points on **a line** of the point cloud data set, RGB-D등 이미지 camera 
- height 
    - unstructured : 항상 `1`, 따라서 1이 아니면 structured 
    - structured : the total number of rows of the point cloud
    
템플릿 
- PointXYZ : `pcl::PointCloud<pcl::PointXYZ> cloud;`


## 2. Structure : `PointXYZ`, `PointXYZI`, `PointXYZRGB`, `PCLPointCloud2`


### 2.1 PointXYZ

pcl::PointXYZ::PointXYZ 

```cpp
  pcl::PointXYZ::PointXYZ ( float_x,
                      float_y,
                      float_z
                        ) 
```

### 2.2 PointXYZI

Pcl::PointXYZI structure

```cpp

union {
   struct {
      float   intensity
   } 	
   float   data_c [4]
}; 

```

### 2.3 PointXYZRGB

Pcl::PointXYZRGB structure

```cpp

Eigen::Vector3i 	getRGBVector3i ()
const Eigen::Vector3i 	getRGBVector3i () const
Eigen::Vector4i 	getRGBVector4i ()
const Eigen::Vector4i 	getRGBVector4i () const
 	PointXYZRGB (const _PointXYZRGB &p)
 	PointXYZRGB ()
 	PointXYZRGB (uint8_t _r, uint8_t _g, uint8_t _b)
//其中PointXYZRGB	为
pcl::PointXYZRGB::PointXYZRGB	(	uint8_t 	_r,
uint8_t 	_g,
uint8_t 	_b 
)	
	
```



### 2.4 PCLPointCloud2

Pcl::PCLPointCloud2 structure

```cpp
struct PCLPointCloud2
         {
          PCLPointCloud2 () : header (), height (0), width (0), fields (),
         is_bigendian (false), point_step (0), row_step (0),
          data (), is_dense (false)
          {
         #if defined(BOOST_BIG_ENDIAN)
          is_bigendian = true;
         #elif defined(BOOST_LITTLE_ENDIAN)
          is_bigendian = false;
         #else
         #error "unable to determine system endianness"
         #endif
          } 
      ::pcl::PCLHeader header;
     
      pcl::uint32_t height;
      pcl::uint32_t width;
     
      std::vector< ::pcl::PCLPointField> fields;
     
      pcl::uint8_t is_bigendian;
      pcl::uint32_t point_step;
      pcl::uint32_t row_step; 
      std::vector<pcl::uint8_t> data; 
      pcl::uint8_t is_dense; 
      public:
      typedef boost::shared_ptr< ::pcl::PCLPointCloud2> Ptr;
      typedef boost::shared_ptr< ::pcl::PCLPointCloud2 const> ConstPtr;
      }; // struct PCLPointCloud2 

```


---

Replace each other : https://blog.csdn.net/qq_16481211/article/details/85332763#pclPCLPointCloud2_80


---
- [Reading Point Cloud data from PCD files](http://www.pointclouds.org/documentation/tutorials/reading_pcd.php#reading-pcd)

- [Writing Point Cloud data to PCD files](http://www.pointclouds.org/documentation/tutorials/writing_pcd.php#writing-pcd)








