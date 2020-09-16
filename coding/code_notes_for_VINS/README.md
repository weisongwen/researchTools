# C++ code notes for VINS

## LK光流跟踪介绍
-   LK光流点不需要计算和匹配描述子，但是也需要一定的计算量。此外，LK光流跟踪能直接得到特征点的对应关系（划重点），不太会误匹配，但是光流必需要求相机的运动是微小的。

    
- Lucas-Kanade光流

    Lucas–Kanade光流算法是一种两帧差分的光流估计算法。在LK光流中，认为来自相机的图像是随时间变化的。图像可以看作时间的函数: I(t)，在t时刻，位于(x,y)处的像素的灰度可以写成 I(x,y,t) 。这种方式把图像看成了关于位置与时间的函数，它的值域就是图像中像素的灰度。现在考虑某个固定的空间点，它在t时刻的像索坐标为x,y。由于相机的运动，它的图像坐标将发生变化。我们希望估计这个空间点在其他时刻图像中位置。LK光流法有三个基本假设：
   1. 灰度不变：一个像素点随着时间的变化，其亮度值（像素灰度值）是恒定不变的。这是光流法的基本设定。所有光流法都必须满足。灰度不变假设是很强的假设， 实际当中很可能不成立。事实上，由于物体的材质不同，像素会出现高光和阴影部分；有时，相机会自动调整曝光参数，使得图像整体变亮或变暗。这此时候灰度不变假设都是不成立的，因此光流的结果也不一定可靠。
-           I (x+dx, y+dy, t+dt) = I (x,y,t). 
   2.  小运动： 时间的变化不会引起位置的剧烈变化。这样才能利用相邻帧之间的位置变化引起的灰度值变化，去求取灰度对位置的偏导数。所有光流法必须满足。

   3.  空间一致：假设某一个窗口内的像素具有相同的运动。这是LK光流法独有的假定。因为为了求取x,y方向的速度，需要建立多个方程联立求解。而空间一致假设就可以利用邻域n个像素点来建立n个方程。
```
//Opencv 代码
   void calcOpticalFlowPyrLK( InputArray prevImg, InputArray nextImg,
                           InputArray prevPts, CV_OUT InputOutputArray nextPts,
                           OutputArray status, OutputArray err,
                           Size winSize=Size(21,21), int maxLevel=3,
                           TermCriteria criteria=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),
                           int flags=0, double minEigThreshold=1e-4);
                           
prevImg – 第一个8位输入图像或者通过 buildOpticalFlowPyramid()建立的金字塔
nextImg – 第二个输入图像或者和prevImg相同尺寸和类型的金字塔
prevPts – 二维点向量存储找到的光流；点坐标必须是单精度浮点数
nextPts – 输出二维点向量（用单精度浮点坐标）包括第二幅图像中计算的输入特征的新点位置；当OPTFLOW_USE_INITIAL_FLOW 标志通过，向量必须有和输入一样的尺寸。
status – 输出状态向量（无符号char）；如果相应的流特征被发现，向量的每个元素被设置为1，否则，被置为0.
err – 输出错误向量；向量的每个元素被设为相应特征的一个错误，误差测量的类型可以在flags参数中设置；如果流不被发现然后错误未被定义（使用status（状态）参数找到此情形）。
winSize – 在每个金字塔水平搜寻窗口的尺寸。
maxLevel – 基于最大金字塔层次数。如果设置为0，则不使用金字塔（单级）；如果设置为1，则使用两个级别，等等。如果金字塔被传递到input，那么算法使用的级别与金字塔同级别但不大于MaxLevel。
criteria – 指定迭代搜索算法的终止准则（在指定的最大迭代次数标准值（criteria.maxCount）之后，或者当搜索窗口移动小于criteria.epsilon。）
flags – 操作标志，可选参数：
OPTFLOW_USE_INITIAL_FLOW – 使用初始估计，存储在nextPts中；如果未设置标志，则将prevPts复制到nextPts并被视为初始估计。
OPTFLOW_LK_GET_MIN_EIGENVALS – 使用最小本征值作为误差度量（见minEigThreshold描述）；如果未设置标志，则将原始周围的一小部分和移动的点之间的 L1 距离除以窗口中的像素数，作为误差度量。
minEigThreshold – 算法所计算的光流方程的2x2标准矩阵的最小本征值（该矩阵称为[Bouguet00]中的空间梯度矩阵）÷ 窗口中的像素数。如果该值小于MinEigThreshold，则过滤掉相应的特征，相应的流也不进行处理。因此可以移除不好的点并提升性能。
```

# 待解决的问题

1. 误匹配的特征点，错误的匹配特征数
2. 远处的特征点视差较小
3. 相机的频率待提高（目前，我们相机的频率是40，但实际应用中要更低）

### Reference
1. [feature_tracker.cpp in VINS-Fusion](https://github.com/kuankuan-yue/VINS-FUSION-leanrning/blob/master/vins_estimator/src/featureTracker/feature_tracker.cpp#L161)
<!-- 2. [Quick Intro to Git and GitHub](https://hplgit.github.io/teamods/bitgit/Langtangen_bitgit_4print.pdf) -->


### Contact
- Author: xiwei, PhD Candidate in Hong Kong Polytechnic University.
- Affiliation: [Intelligent Positioning and Navigation Laboratory](https://www.polyu-ipn-lab.com/)