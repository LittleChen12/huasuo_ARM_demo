using System;
using System.Drawing;
using System.Windows.Media.Media3D;

namespace RobotLibrary
{
    public class Joint:JointBase
    {
        /// <summary>
        /// 变量属性 变量名称 变量解释
        /// 只读Vector3D Axis 旋转轴
        /// 只读Point3D Center 旋转中心点
        /// double angle 旋转角度（角度制）
        /// JointLimit jointlimit 旋转角度范围
        /// 用于仿真模型的运动
        /// </summary> 
        private Vector3D axis;
        public Vector3D Axis { get { return axis; } }
        private Point3D center;
        public Point3D Center { get {return center ; }  }
        public double angle ;
        public JointLimit jointlimit;
        /// <summary>
        /// 空构造函数：Joint类
        /// 枚举变量和、Point3D变量的构造函数：Joint类
        /// </summary>
        public Joint()
        {
            jointlimit = new JointLimit();
            model3D = new GeometryModel3D();
            modelvisual3D = new ModelVisual3D();
            jointpath = "";
        }
        /// <summary>
        /// 构造函数：Joint类
        /// 参数：旋转轴枚举变量、旋转轴的3D坐标
        /// 设置旋转轴和旋转轴的3D坐标
        /// </summary>
        /// <param name="eRobotRotAxis"></param>
        /// <param name="_center"></param>
        public Joint(ERobotRotAxis eRobotRotAxis, Point3D _center)
        {
            switch (eRobotRotAxis)
            {
                case ERobotRotAxis.RotX:
                    axis.X = 1;
                    axis.Y = 0;
                    axis.Z = 0;
                    break;
                case ERobotRotAxis.RotY:
                    axis.Y = 1;
                    axis.Z = 0;
                    axis.X = 0; 
                    break;
                case ERobotRotAxis.RotZ:
                    axis.Z = 1;
                    axis.X = 0;
                    axis.Y = 0;
                    break;
                default:
                    break;
            }
            center= _center;
            jointlimit = new JointLimit();
        }
        /// <summary>
        /// 设置旋转轴
        /// </summary>
        /// <param name="_axis">旋转轴的轴向量</param>
        public void SetRot(Vector3D _axis)
        {
            axis=_axis; 
        }
        /// <summary>
        /// 设置旋转的位置
        /// </summary>
        /// <param name="Point3D">旋转轴的3D坐标</param>
        public void SetRotPoints(Point3D _center)
        {
            center = _center;
        }

    }
}