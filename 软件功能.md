# 软件功能

## 一、加载模型、清除模型、模型运动->Robot父类

### 1.加载机械臂(一次性加载所有)

步骤：

0.所有关节文件放在一个文件夹(RobotModels)

![image-20250218124536216](D:\typora\软件功能.assets\image-20250218124536216.png)

1.设置模型相对路径

2.加载模型

3.jonts参数初始化(已知参数)

~~~c#
public virtual void RobotInit(string RelativeModelsPath)
~~~

加载到Robot的JointCollection的joints里；

依据机械臂参数加载到joints里的joint类的旋转轴和旋转位置

### 2.加载单个STL模型/工具

~~~c#
 public virtual void JointModel3DStlLoad(string path)
~~~

给出stl模型位置，加载到Robot的JointCollection的joints里

### 3.机械臂模型清除

~~~c#
public virtual void RobotModelClear()
~~~

清除Robot的JointCollection的joints

### 4.模型运动

#### 4.1点动

~~~c++
  public virtual void ForwardMove(double[] angles)
~~~

依据现有的Robot的JointCollection的joints里的joint数目，进行各个关节的移动

#### 4.2联动

#### 4.3关节空间运动

#### 4.4笛卡尔空间运动

#### 4.5工具坐标系运动

## 二、界面交互模型、轨迹点模型

### 1.BOX模型

~~~c#
 public void BoxModelInit(Point3D _point,double[] position)
~~~



![image-20250218131921400](D:\typora\软件功能.assets\image-20250218131921400.png)

### 2.frame模型

![image-20250218133305139](D:\typora\软件功能.assets\image-20250218133305139.png)

### 3.PointsModel模型

~~~c++
 public void PointsModelInit()
 {
     PointsVisual.Color = Colors.Red;
     PointsVisual.Size = 4;
 }
~~~



![image-20250218133420904](D:\typora\软件功能.assets\image-20250218133420904.png)

### 4.点模型

~~~c++
public class SphereModel
~~~

## 三、界面显示

![image-20250218133842165](D:\typora\软件功能.assets\image-20250218133842165.png)

## 四、加载机械臂参数

### 1.加载txt文件参数

~~~c#
 public virtual double[,] RobotSportsParamsTxtRead(string robotJointsParams)
~~~

### 2.将参数加载到模型的joint

~~~c++
public virtual void JointsParmasInit()
~~~

## 五、路径规划与轨迹规划

### 1.直线路径规划

![image-20250218134227826](D:\typora\软件功能.assets\image-20250218134227826.png)

### 2.多项式曲线轨迹规划

![image-20250218134306216](D:\typora\软件功能.assets\image-20250218134306216.png)

### 3.路径保存和读取(后面改成弹窗口选择文件)

![image-20250218134943025](D:\typora\软件功能.assets\image-20250218134943025.png)

![image-20250218135103044](D:\typora\软件功能.assets\image-20250218135103044.png)

### 4.路径清除

### 5.任意路径规划

空间点击点

![image-20250218135551047](D:\typora\软件功能.assets\image-20250218135551047.png)

## 六、作图与轨迹分析

![image-20250218140403369](D:\typora\软件功能.assets\image-20250218140403369.png)