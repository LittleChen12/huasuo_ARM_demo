using Controller;
using Device;
using HelixToolkit.Wpf;
using Microsoft.Win32;
using Model;
using RobotLibrary;
using RobotLibraryAlgorithm;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media.Media3D;
using TrajectoryPositionList;

namespace Robots
{
    //机器人管理
    public class RobotManager
    {
        /// <summary>
        /// 变量属性 变量名称 变量解释
        /// string RobotType 机器人类型
        /// Robot robot 机器人父类对象。子类可以赋值作为统一
        /// Tool tool 工具
        /// Position RightNowPosition 当前位置
        /// AlgorithmManager algorithmManager 算法管理器
        /// </summary>
        public string RobotType;
        public Robot robot;
        private Tool tool;
        public Tool Tool
        {
            get;
            set;
        }
        public Position RightNowPosition;
        public AlgorithmManager algorithmManager;
        public MovePositionList movepositionlist;
        public CartesianPositionList cartesianpositionlist;

        public RobotManager()
        {
            tool = new Tool();
            RightNowPosition = new Position();
            algorithmManager = new AlgorithmManager(RobotType);
            movepositionlist = new MovePositionList();
            cartesianpositionlist = new CartesianPositionList();
        }
        /// <summary>
        /// 依据传入不同机器人类型，进行初始化
        /// 机器人初始化
        /// </summary>
        /// <param name="robottype"></param>
        public void RobotInit(string robottype)
        {
            RobotType = robottype;
            robot = RobotFactory.CreateRobot(RobotType);
            robot.RobotInit(RobotType);
            algorithmManager = new AlgorithmManager(RobotType);
            robot.MoveControlInit();
            robot.KinematicsInit();
        }
        public void RobotClear() 
        {
            robot.RobotModelClear();
        }
        /// <summary>
        /// 初始化工具
        /// 设置工具路径，加载工具模型，设置工具转换坐标系
        /// 
        /// </summary>
        /// <param name="ToolPath"></param>
        public void ToolInit(string ToolPath)
        {
            tool.SetJointPath(ToolPath);
            var load=new RobotStlLoad();
            tool.model3D=load.ModelSTLload(tool.jointpath);
            tool.modelvisual3D.Content=tool.model3D;
            double[] angles = new double[6];
            double[] JointRad = algorithmManager.algorithm.AngleToRad(angles);
            JointPosition jointposition = new JointPosition(JointRad);
            var path = new RobotLibraryAlgorithm.KinematicsAlgorithm.KinematicsHuaShu(jointposition);
            Matrix4x4 GripToTool = algorithmManager.algorithm.ToTrans(tool.CartesianPosition);
            tool.CartesianPosition= path.FkAngle(angles, GripToTool);
            double[] frame = new double[3] { tool.CartesianPosition.Point.X* 1000, tool.CartesianPosition.Point.Y * 1000, tool.CartesianPosition.Point.Z * 1000 };
            tool.ToolTd = new TranslateTransform3D(frame[0], frame[1], frame[2]);
        }
        public void ToolClear()
        {
            tool.modelvisual3D.Content=null;
        }

        //连续运动函数
        /// <summary>
        /// 传角度-》坐标
        /// </summary>
        /// <param name="angles">角度:°</param>
        public void FKShow(double[] angles)
        {
            //CartesianPosition carposition = algorithmManager.kinematicsAlgorithm.FkAngle(angles, algorithmManager.algorithm.ToTrans(tool.CartesianPosition));
            var p=new Matrix4x4();
            p.M11 = 1;p.M22 = 1;p.M33 = 1;p.M44 = 1;
       
            CartesianPosition carposition = algorithmManager.kinematicsAlgorithm.FkAngle(angles, p);
            double[] rad = algorithmManager.algorithm.AngleToRad(angles);
            JointPosition joint = new JointPosition(rad);
            RightNowPosition = new Position(carposition, joint);
        }

        /// <summary>
        /// 获取当前工作目录
        /// </summary>
        /// <returns></returns>
        private static string GetCurSourceFileName()
        {
            string path = Directory.GetCurrentDirectory();
            var p = Directory.GetParent(path);
            string s = p.Parent.Parent.FullName;
            return s;
        }

        /// <summary>
        /// 保存路径点->text
        /// 路径存放在 \PositionsSave文件夹下
        /// </summary>
        /// <param name="CartesianPositionlist"></param>
        public void SavePoseTxt(CartesianPositionList CartesianPositionlist)
        {
            SaveFileDialog saveFileDialog = new SaveFileDialog();

            // 设置文件类型过滤器
            saveFileDialog.Filter = "文本文件|*.txt|所有文件|*.*";
            saveFileDialog.Title = "保存文本文件";
            saveFileDialog.DefaultExt = "txt";    // 默认扩展名
            saveFileDialog.AddExtension = true;   // 自动添加扩展名
            saveFileDialog.OverwritePrompt = true;// 覆盖时提示
            List<string> contents = new List<string>();
            if (saveFileDialog.ShowDialog() == true)
            {
                try
                {
                    contents.Add("X(mm) Y(mm) Z(mm) RX(rad) RY(rad) RZ(rad)");
                    for (int i = 0; i < CartesianPositionlist.CartesianPositions.Count; i++)
                        contents.Add(CartesianPositionlist.CartesianPositions[i].Point.X.ToString() + " " + CartesianPositionlist.CartesianPositions[i].Point.Y.ToString() + " "
                            + CartesianPositionlist.CartesianPositions[i].Point.Z.ToString() + " " + CartesianPositionlist.CartesianPositions[i].Rx.ToString() + " "
                            + CartesianPositionlist.CartesianPositions[i].Ry.ToString() + " " + CartesianPositionlist.CartesianPositions[i].Rz.ToString());

                    File.WriteAllLines(saveFileDialog.FileName, contents);
                    Console.WriteLine("保存成功！ " + "文件路径：" + saveFileDialog.FileName);

                }
                catch (Exception ex)
                {
                    // 处理异常
                }
            }

           
        }
        /// <summary>
        /// 
        /// 加载路径点->text
        /// </summary>
        /// <returns>CartesianPositionList</returns>
        public CartesianPositionList LoadPoseTxt()
        {
            OpenFileDialog OpenFileDialog = new OpenFileDialog();
            // 设置文件类型过滤器
            OpenFileDialog.Filter = "文本文件|*.txt|所有文件|*.*";
            OpenFileDialog.Title = "读取文本文件";
            OpenFileDialog.DefaultExt = "txt";    // 默认扩展名
            OpenFileDialog.AddExtension = true;   // 自动添加扩展名
            CartesianPositionList CartesianPositionlist = new CartesianPositionList();
            string[] lines;
            if (OpenFileDialog.ShowDialog() == true)
            {
                try
                {
                     lines = File.ReadAllLines(OpenFileDialog.FileName);
                    foreach (string line in lines)
                    {
                        if (line.Contains("X"))
                            continue;
                        // 使用String.Split方法将每一行分割成字符串数组
                        string[] parts = line.Split(' ');
                        var position = new CartesianPosition();
                        Point3D point = new Point3D(double.Parse(parts[0]), double.Parse(parts[1]), double.Parse(parts[2]));
                        position.Point = point;
                        position.Rx = double.Parse(parts[3]);
                        position.Ry = double.Parse(parts[4]);
                        position.Rz = double.Parse(parts[5]);
                        CartesianPositionlist.CartesianPositions.Add(position);
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine("文件不存在");   
                }
            }
            return CartesianPositionlist;

        }

        /// <summary>
        /// 在界面显示路径点
        /// </summary>
        /// <param name="RightNowPosition">机械臂当前位姿 </param>
        /// <param name="CartesianPositions">一系列笛卡尔中间位姿 </param>
        /// <returns></returns>
        public PointsModel PathPointsShow(Position RightNowPosition, CartesianPositionList CartesianPositions)
        {
            PointsModel PathPoints=new PointsModel();
            PathPoints.PointsVisual.Size = 3;
            var Points = algorithmManager.MoveInterPolation.MoveL(RightNowPosition, CartesianPositions.CartesianPositions[0].mmTom(), 0.5, tool.GripToTool);
            for (int i = 1; i < CartesianPositions.CartesianPositions.Count; i++)
            {
                var list = algorithmManager.MoveInterPolation.MoveL(Points[Points.Count - 1], CartesianPositions.CartesianPositions[i].mmTom(), 0.5, tool.GripToTool);
                foreach (var p in list)
                {
                    Points.Add(p);
                }
            }
            foreach (var p in Points)
            {
                PathPoints.PointsVisual.Points.Add(new Point3D(p.Pose.Point.X * 1000, p.Pose.Point.Y * 1000, p.Pose.Point.Z * 1000));
            }
            return PathPoints;
        }


        //public double[,] ConvertToRobotMatrix()
        //{
        //    double[,] robotparams = new double[joints.Count, 6];
        //    for (int i = 0; i < joints.Count; i++)
        //    {
        //        robotparams[i, 0] = joints[i].Axis.X;
        //        robotparams[i, 1] = joints[i].Axis.Y;
        //        robotparams[i, 2] = joints[i].Axis.Z;
        //        robotparams[i, 3] = joints[i].Center.X;
        //        robotparams[i, 4] = joints[i].Center.Y;
        //        robotparams[i, 5] = joints[i].Center.Z;
        //    }
        //    return robotparams;
        //}
        //public double[,] ConvertToRobotLimitMartix()
        //{
        //    double[,] limitmartix = new double[4, joints.Count];

        //    return limitmartix;
        //}


    }
}
