# 3Point Random Sample Consensus(RANSAC) for ROS2

![image](https://github.com/Minho-iNCSL/3p_RANSAC_ros2/assets/60316325/5aff65b3-b381-487d-9219-ea1c13eb8077)

Implementation of 3 point RANSAC with visualization (Rviz2)
Not fully complete detail about the code ( 2point, parameter configuration, others.. ) 

```
cd ~/ros2_ws/src/
git clone https://github.com/Minho-iNCSL/3p_RANSAC_ros2.git
colcon build
ros2 launch ransac ransac_node
```

#### How to change several parameter ? 

In `/param/ransac.yaml`, we just focus 3 parameter (success_probability, outlier_ratio, threshold).

Iteration($N$) is automatically calculated in code.

success_probability = $p$

outlier_ratio = $\epsilon$

subset_size = $s$

$$ N = \dfrac{log(1-p)}{log(1-(1-\epsilon)^s)} $$
