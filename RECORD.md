## RECORD

最近在重学视觉slam十四讲，然后开始看代码了，但是有一些环境没有装，特别是好像还不可以调用ros的opencv，正在重装。后面要把这里提到的一些代码彻底搞懂，并且新建一个叫SLAM-TOOLS的仓库，把这些代码整理改进之后放这个仓库，以后再用！

## 视觉SLAM十四讲中的重要代码

- ch3/examples/plotTrajectory.cpp：用于显示机器人轨迹，对SLAM做分析特别有用！！
- ch3/visualizeGeometry/visualizeGeometry .cpp：可视化，四元数、旋转矩阵可以直观的看到！
- ch4/examples/trajectoryError.cpp：可以直观的比较两个轨迹，并且计算误差，非常有用！
- ch5/imageBasics/undistortImage.cpp：图像去畸变的代码，SLAM里面会用上！
- ch5/rgbd/joinMap.cpp：根据rgbd生成点云，这个对后面建图的代码来说比较关键！！
- ch5/stereo/stereoVision.cpp：根据双目计算深度并且生成点云，也很关键！！
- 

