
# 설치 

```
$ pip install k3d
$ jupyter nbextension install --py --sys-prefix k3d
$ jupyter nbextension enable --py --sys-prefix k3d
```


```python 
#jupyter
pa = cloud.to_array()

plot = k3d.plot()
print(pa.shape[0])
points_number = pa.shape[0]
colors = np.random.randint(0, 0xFFFFFF, points_number)

points = k3d.points(pa, colors, point_size=0.01, shader='3d')
plot += points
plot.camera_auto_fit = False
plot.display()
```