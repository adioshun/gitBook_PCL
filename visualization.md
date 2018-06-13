# k3d를 이용한 시각화 

```python 

# 시각화 부분

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


# Matplot를 이용한 시각화 


```python

%matplotlib inline

X = data

# #############################################################################
# Plot result
import matplotlib.pyplot as plt

# Black removed and is used for noise instead.
unique_labels = set(labels)
colors = [plt.cm.Spectral(each)
          for each in np.linspace(0, 1, len(unique_labels))]
for k, col in zip(unique_labels, colors):
    if k == -1:
        # Black used for noise.
        col = [0, 0, 0, 1]

    class_member_mask = (labels == k)

    xy = X[class_member_mask & core_samples_mask]
    plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
             markeredgecolor='k', markersize=1)# 14

    xy = X[class_member_mask & ~core_samples_mask]
    plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
             markeredgecolor='k', markersize=1)

plt.title('Estimated number of clusters: %d' % n_clusters_)
plt.show()


```


---

https://www.paraview.org/

https://www.slicer.org/ : medical images


http://www.sci.utah.edu/software/imagevis3d.html

http://www.danielgm.net/cc/