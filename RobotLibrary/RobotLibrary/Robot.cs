using RobotLibraryAlgorithm;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Controls;
using System.Windows.Media.Media3D;

namespace RobotLibrary
{
    public abstract class Robot
    {
        /// <summary>
        /// 变量类型 变量名称 变量解释
        /// JointCollection Joints 机器人关节组
        /// double[,] RobotSportsParams  机器人参数 前三个参数是旋转轴xyz, 后三个参数是旋转中心坐标
        /// double[,] RobotLimParams 机器人软件限制参数
        /// </summary>
        private JointCollection joints=new JointCollection();
        public JointCollection Joints
        {
            get { return joints; }
            set { joints = value; }
        }
        public Tool? tool =null;
        //用于界面模型的运动
        public double[,] RobotSportsParams = new double[6, 6];
        //机器人软件限制参数
        public double[,] RobotLimParams = new double[4, 6];
        /// <summary>
        /// 机器人初始化
        /// //1.设置模型相对路径
        /// //2.加载模型
        /// //3.jonts参数初始化(已知参数)
        /// </summary>
        /// <param name="RelativeModelsPath"></param>
        public virtual void RobotInit(string RelativeModelsPath)
        {
            Joints = new JointCollection();
            if(joints.BasePath== "")
            { ChooseJointsPath("\\RobotModels\\" + RelativeModelsPath); }
            JointsModel3DSTLLoad();
            JointsParmasInit();
        }
        /// <summary>
        /// 获取当前程序集所在文件夹
        /// </summary>
        /// <returns></returns>
        private static string GetCurSourceFileName()
        {
            string path = Directory.GetCurrentDirectory();
            var p = Directory.GetParent(path);
            if (p?.Parent?.Parent == null)
            {
                throw new InvalidOperationException("无法获取当前程序集所在文件夹的路径。");
            }
            return p.Parent.Parent.FullName;
        }
        /// <summary>
        /// 相对路径
        /// 获取3D_Models\相对路径文件夹下的所有文件
        /// 并将关节组的每个关节设置绝对路径
        /// 如果原来关节组有文件，则删除
        /// </summary>
        private string[] ChooseJointsPath(string RelativePath)
        {
            joints.BasePath = GetCurSourceFileName()+RelativePath;
            string[] modelilst = Directory.GetFiles(joints.BasePath); 
            if(joints.Count!=0)
            {
                joints.Clear();
            }
            foreach (string model in modelilst)
            {
               joints.Add(new Joint());
               joints[joints.Count - 1].SetJointPath(model);
            }
            return modelilst;
        }
        /// <summary>
        /// 设置绝对路径同时把每个关节的绝对路径加入Joints集合
        /// 并将关节组的每个关节设置绝对路径
        /// 如果原来关节组有文件，则删除
        /// </summary>
        /// <param name="JointsPath">绝对路径</param>
        /// <returns>返回该文件下所有文件路径</returns>
        public virtual string[] SetJointsPath(string JointsPath)
        {
            joints.BasePath = JointsPath;
            string[] modelilst = Directory.GetFiles(joints.BasePath);
            if (joints.Count != 0)
            {
                joints.Clear();
            }
            foreach (string model in modelilst)
            {
                joints.Add(new Joint());
                joints[joints.Count - 1].SetJointPath(model);
            }
            return modelilst;
        }
        /// <summary>
        /// //STL文件加载到Joint组
        /// Joints每个Joint的Model3D属性也加载
        /// Joints的RobotModel与RobotModelVisual加载为整个机械臂
        /// </summary>
        private  void JointsModel3DSTLLoad()
        {
            RobotStlLoad load = new RobotStlLoad();
            Joints.RobotModel = load.JointsStlLoad(Joints.BasePath);
            for (int i=0;i< joints.Count; i++)
            {
                joints[i].model3D = joints.RobotModel.Children[i];
            }
            joints.RobotModelVisual.Content = Joints.RobotModel;
        }
        /// <summary>
        /// 
        /// 加载单个机器关节模型到整个机械臂RobotModel
        /// joints里的joint的model3D、modelvisual3D赋值
        /// </summary>
        /// <param name="path">绝对路径</param>
        public virtual void JointModel3DStlLoad(string path)
        {
            joints.Add(new Joint());
            joints[joints.Count - 1].SetJointPath(path);
            RobotStlLoad load = new RobotStlLoad();
            joints[joints.Count - 1].model3D= load.ModelSTLload(path);
            joints[joints.Count - 1].modelvisual3D.Content = joints[joints.Count - 1].model3D;
            joints.RobotModel.Children.Add(joints[joints.Count - 1].model3D);
            joints.RobotModelVisual.Children.Add(joints[joints.Count - 1].modelvisual3D);
        }

        /// <summary>
        /// 机器人模型清除
        /// 清空Joints里的所有关节
        /// 清空机械臂的RobotModelVisual
        /// </summary>
        public virtual void RobotModelClear()
        {
            //viewPort3d.Children.Remove(joints.RobotModelVisual);
            joints.RobotModelVisual.Children.Clear();
            joints.Clear();
        }
        /// <summary>
        /// 添加工具到机械臂
        /// </summary>
        /// <param name="_tool"></param>
        public virtual void RobotAddTool(Tool _tool)
        {
            tool=_tool; 
            joints.Add(new Joint());
            joints[joints.Count - 1].model3D= tool.model3D;
            joints[joints.Count - 1].modelvisual3D.Content = tool.model3D;
            joints.RobotModel.Children.Add(joints[joints.Count - 1].model3D);
            joints.RobotModelVisual.Children.Add(joints[joints.Count - 1].modelvisual3D);
        }

        /// <summary>
        /// 读取新机器人运动参数（txt）
        /// 加载到参数数组RobotSportsParams
        /// </summary>
        /// <param name="robotJointsParams"></param>
        /// <returns></returns>
        public virtual double[,] RobotSportsParamsTxtRead(string robotJointsParams)
        {
            string[] content = null;
            try
            {
                content = File.ReadAllLines(robotJointsParams);
            }
            catch (Exception ex)
            {
                Console.WriteLine("An error occurred: " + ex.Message);
            }
            for (int i = 0; i < content.Length; i++)
            {
                string line = content[i];
                int[] num = line.Split(' ').Select(int.Parse).ToArray();
                for (int j = 0; j < 6; j++)
                {
                    RobotSportsParams[i, j] = num[j];
                }
            }
            return RobotSportsParams;
        }
        /// <summary>
        /// 关节运动参数初始化
        /// 根据RobotSportsParams对关节组的各个关节设置旋转轴与旋转中心
        /// 底座固定因此参数少一个
        /// </summary>
        public virtual void JointsParmasInit()
        {
            for (int i = 0; i < joints.Count - 1; i++)
            {
                    joints[i].SetRot(new Vector3D(RobotSportsParams[i, 0], RobotSportsParams[i, 1], RobotSportsParams[i, 2]));
                    joints[i].SetRotPoints(new Point3D(RobotSportsParams[i, 3], RobotSportsParams[i, 4], RobotSportsParams[i, 5]));  
            }
        }

        /// <summary>
        /// 模型运动
        /// </summary>
        /// <param name="angles">角度制</param>
        public virtual void ForwardMove(double[] angles)
        {
            var TS = new List<Transform3DGroup>();
            var RS= new List<RotateTransform3D>();
            for (int i = 0;i<joints.Count-1;i++)
            {
                if (tool != null && i == joints.Count - 2)
                {
                    TS.Add(new Transform3DGroup());
                    TS[joints.Count - 2].Children.Add(tool.ToolTd);
                    TS[joints.Count - 2].Children.Add(TS[joints.Count - 3]);
                }
                else 
                {
                    RS.Add(new RotateTransform3D(new AxisAngleRotation3D(joints[i].Axis, angles[i]), joints[i].Center));
                    TS.Add(new Transform3DGroup());
                    TS[i].Children.Add(RS[i]);
                    if (i != 0)
                    TS[i].Children.Add(TS[i - 1]);
                }
            }
            //底座不变换
            for (int i = 0; i < TS.Count; i++)
            {
                joints.RobotModel.Children[i+1].Transform = TS[i];
            }
        }
        /// <summary>
        /// 实际移动控制初始化重写
        /// </summary>
        public abstract void MoveControlInit();
       /// <summary>
       /// 运动学初始化重写
       /// </summary>
       /// <param name="angles"></param>
        public abstract void KinematicsInit();
        /// <summary>
        /// 实际移动控制命令，控制器做父类
        /// </summary>
        public abstract void Command(List<string> CommandStrings);
    }
}
