3D NMS(Non-Maximum Suppression)

```python

# [Non-Maximum Suppression](https://www.pyimagesearch.com/2014/11/17/non-maximum-suppression-object-detection-python/)
import numpy as np
# [IoU](https://gist.github.com/daisenryaku/91e6f6d78f49f67602d21dc57d494c60)

def nms(x, nms_th):
  # x:[p, z, w, h, d]
  if len(x) == 0:
    return x
  x = x[np.argsort(-x[:, 0])]
  bboxes = [x[0]]
  for i in np.arange(1, len(x)):
    bbox = x[i]
    flag = 1
    for j in range(len(bboxes)):
      if iou(bbox[1:5], bboxes[j][1:5]) > nms_th:
        flag = -1
        break
      if flag == 1:
        bboxes.append(bbox)
  bboxes = np.asarray(bboxes, np.float32)
  return bboxes
```


3D IoU (Intersection over Union)

```python
# 3D IoU python Implementation
# [IoU](https://www.pyimagesearch.com/2016/11/07/intersection-over-union-iou-for-object-detection/)

def IoU(box0, box1):
  # box0: [x, y, z, d]
  r0 = box0[3] / 2
  s0 = box0[:3] - r0
  e0 = box0[:3] + r0
  r1 = box1[3] / 2
  s1 = box1[:3] - r1
  e1 = box1[:3] + r1
  
  overlap = [max(0, min(e0[i], e1[i]) - max(s0[i], s1[i])) for i in range(3)]
  intersection = reduce(lambda x,y:x*y, overlap)
  union = pow(box0[3], 3) + pow(box1[3], 3) - intersection
  return intersection / union


```