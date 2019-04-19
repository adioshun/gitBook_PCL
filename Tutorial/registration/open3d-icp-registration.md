# [[Open3D] ICP registration](http://www.open3d.org/docs/tutorial/Basic/icp_registration.html)


- 입력 `The input are `
    - two point clouds 
    - an initial transformation : usually obtained by a [global registration algorithm](http://www.open3d.org/docs/tutorial/Advanced/global_registration.html#global-registration)
- 출력 `The output is `
    - a refined transformation 

        
- In this tutorial, we show two ICP variants, 
    - the point-to-point ICP 
    - the point-to-plane ICP