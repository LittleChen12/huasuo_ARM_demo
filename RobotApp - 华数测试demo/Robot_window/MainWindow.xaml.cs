
using HelixToolkit.Wpf;
using Microsoft.VisualBasic;
using Microsoft.Win32;
//using PA.src;
using Robot_window.Control2183;
using Robot_window.MenuEvents;

using Robot_window.UI_Page;
using Robot_window.ViewModels;
using RobotLibrary;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Data;
using System.Drawing;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Numerics;
using System.Reflection;
using System.Reflection.Emit;
using System.ServiceProcess;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Shell;
using System.Windows.Threading;

namespace Robot_window
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    /// 

    //关节类：旋转轴和旋转中心
    public class Joint
    {
        public double angle = 0;
        public double rotPointX = 0;
        public double rotPointY = 0;
        public double rotPointZ = 0;
        public int rotAxisX = 0;
        public int rotAxisY = 0;
        public int rotAxisZ = 0;
        /*@name     LoadParams
         *@brief    用Joint类将6*6的机械臂参数按关节进行封装，
         *@param    double[,] robotparam,机械臂的旋转中心等
         *@rturn    List<Joint>:列表中有6个关节
         */
        public static List<Joint> LoadParams(double[,] robotparam)
        {
            
            List<Joint> ListJoints = new List<Joint>();
            for (int i = 0; i < robotparam.GetLength(0); i++)
            {
                Joint joint = new Joint();
                joint.rotAxisX = (int)robotparam[i, 0];
                joint.rotAxisY = (int)robotparam[i, 1];
                joint.rotAxisZ = (int)robotparam[i, 2];
                joint.rotPointX = robotparam[i, 3];
                joint.rotPointY = robotparam[i, 4];
                joint.rotPointZ = robotparam[i, 5];
                ListJoints.Add(joint);
            }
            return ListJoints;

        }

    }


    //窗口控制界面
    public partial class MainWindow : Window
    {
        private PortViewModel _viewModel = new PortViewModel();
        public static MainWindow mainwindow;
        public ModelVisual3D visual = new ModelVisual3D();
        public Model3DGroup RobotModel;
        public List<Joint> ListJoints;
        public Position RightNowPosition = new Position();
        public DispatcherTimer timer = new DispatcherTimer()
        {
            Interval = TimeSpan.FromMilliseconds(2)
        };

        int test_i = 0;
        public RectangleVisual3D rectangleVisual3D = new RectangleVisual3D();
        public SphereVisual3D sphere = new SphereVisual3D();
        public CoordinateSystemVisual3D coordinateSystem = new CoordinateSystemVisual3D
        {
            ArrowLengths = 50,
           
            
        };


        ////工具坐标系
        //public Matrix4x4 GripToTool = new Matrix4x4(
        //     1, 0, 0, (float)0.0736197,
        //     0, 1, 0, (float)-0.0034699,
        //     0, 0, 1, (float)-0.0250941,
        //     0, 0, 0, 1
        //);
        public Matrix4x4 GripToTool = new Matrix4x4(
     1, 0, 0, 0,//
     0, 1, 0, 0,
     0, 0, 1, 0,
     0, 0, 0, 1

);

        //public Matrix4x4 GripToTool = new Matrix4x4(
        //     1, 0, 0, 0,//
        //     0, 1, 0, 0.111f,
        //     0, 0, 1, 0.042f,
        //     0, 0, 0, 1

        //);
        //        public Matrix4x4 GripToTool = new Matrix4x4(
        //     0, 1, 0, 0,//
        //     -1, 0, 0, 0.111f,
        //     0, 0, 1, 0.042f,
        //     0, 0, 0, 1

        //);
        private MenuEvents.MenuEvents menuevents;
        public double[] OldAngles = new double[6];//辅助2183，增量控制
        public bool simulation = true;
        public int Control2183Speed = 100;//2183速度
        public Model3DGroup group = new Model3DGroup();//末端坐标系frame
        public TranslateTransform3D XYZtransform = new TranslateTransform3D();//末端变换
        CoordinateSystemVisual3D coordinateSystemVisual3D = new CoordinateSystemVisual3D();
        ArrowVisual3D arr= new ArrowVisual3D();

        //SerialPortUtils serialPortUtils = new SerialPortUtils();

        public bool Simulation
        {
            get { return simulation; }
            set { simulation = value; }
        }
        public bool SingleMove
        {
            get; set;
        }
        public bool NewIKFlag
        {
            get; set;
        }
        const bool IsConnectdSerialPort = true;


        public MainWindow()
        {
            InitControl2183();
            InitializeComponent();
            mainwindow = this;

            InitRobot();
            InitViewModels();
            this.DataContext = this;


            //Control2183.Control2183.control.control2183.command("BGS", "\r", ":", true); //开始运动

            if (IsConnectdSerialPort)
            {
                InitCom();
                InitCoder();
            }


        }

        /*@name    :InitRobot
         *@brief   :使用RobotLoad类加载STL模型并在界面显示模型
         *@param   :null  
         */
        private void InitRobot()
        {
            RobotParams robotParams = new RobotParams();
            robotParams.LinkBasePath = GetCurSourceFileName() + "\\3D_Models\\HuaShu";
            RobotLoad load = new RobotLoad();
            RobotModel = load.StlLoad(robotParams.LinkBasePath);

            visual.Content = RobotModel;
            viewPort3d.Children.Add(visual);

            ListJoints = Joint.LoadParams(robotParams.robotparams);
            InitFrame();

            //viewPort3d.RotateGesture = new MouseGesture(MouseAction.LeftClick);
            //viewPort3d.PanGesture = new MouseGesture(MouseAction.RightClick);
            viewPort3d.Camera.LookDirection = new Vector3D(2038, -5200, -2930);
            viewPort3d.Camera.UpDirection = new Vector3D(-0.145, 0.372, 0.917);
            viewPort3d.Camera.Position = new Point3D(-1571, 4801, 3774);
            double[] angles = { 0, 0, 0, 0, 0, 0 };

            ForwardMove(angles);
        }
        //显示机械臂末端位姿的坐标系
        private void InitFrame()
        {

            double[] angles = new double[6];
            double[] JointRad = BasicAlgorithm.AngleToRad(angles);
            JointPosition jointposition = new JointPosition(JointRad);
            PathClass path = new PathClass(jointposition);
            path.FK(GripToTool);
            double[] frame = new double[3] { path.Points.X * 1000, path.Points.Y * 1000, path.Points.Z * 1000 };

            /*法2/
            //XYZtransform = new TranslateTransform3D(frame[0], frame[1], frame[2]);
            //coordinateSystemVisual3D = new CoordinateSystemVisual3D
            //{
            //    ArrowLengths = 100,
            //};
            ////var translation = new TranslateTransform3D(frame[0], frame[1], frame[2]);  // 平移
            ////var rotation = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), 180));  // 围绕 Y 轴旋转 45 度

            //// 创建一个 Transform3DGroup，将多个变换组合起来

            ////group.Children.Add(rotation);
            ////group.Children.Add(translation);

            //// 将组合变换应用于坐标系
           
            //viewPort3d.Children.Add(coordinateSystemVisual3D);


            /*
             * 法一：圆柱和圆锥结合手写
             */
            var m = new ModelVisual3D();
            MeshBuilder builderx = new MeshBuilder(false, false);
            builderx.AddCylinder(new Point3D(frame[0], frame[1], frame[2]), new Point3D(frame[0], frame[1], frame[2] + 80), 5, 36);
            builderx.AddCone(new Point3D(frame[0], frame[1], frame[2] + 75), new Point3D(frame[0], frame[1], frame[2] + 85), 7.5, false, 36);
            var gx = new GeometryModel3D(builderx.ToMesh(), Materials.Red);
            MeshBuilder bulidery = new MeshBuilder(false, false);
            bulidery.AddCylinder(new Point3D(frame[0], frame[1], frame[2]), new Point3D(frame[0], frame[1] - 80, frame[2]), 5, 36);
            bulidery.AddCone(new Point3D(frame[0], frame[1] - 75, frame[2]), new Point3D(frame[0], frame[1] - 85, frame[2]), 7.5, false, 36);
            var gy = new GeometryModel3D(bulidery.ToMesh(), Materials.Green);
            MeshBuilder buliderz = new MeshBuilder(false, false);
            buliderz.AddCylinder(new Point3D(frame[0], frame[1], frame[2]), new Point3D(frame[0] + 80, frame[1], frame[2]), 5, 36);
            buliderz.AddCone(new Point3D(frame[0] + 75, frame[1], frame[2]), new Point3D(frame[0] + 85, frame[1], frame[2]), 7.5, false, 36);
            var gz = new GeometryModel3D(buliderz.ToMesh(), Materials.Blue);

            group = new Model3DGroup();
            group.Children.Add(gx);
            group.Children.Add(gy);
            group.Children.Add(gz);
            m.Content = group;
            viewPort3d.Children.Add(m);
        }


        /*@name     InitViewModels
         * @brief   初始化界面一些用户控件，订阅事件等
         */
        private void InitViewModels()
        {
            timer.Tick += RobotMoveTick;
            menuevents = new MenuEvents.MenuEvents();
            viewPort3d.MouseDown += menuevents.world3dclick;
            viewPort3d.KeyDown += menuevents.KeyDownSpace;

            comsearch.Click += menuevents.SearchCom;

            Specimens.Click += menuevents.SpecimensProduce;
            desk.Click += menuevents.DeskProduce;
            contentcon.Content = menuevents.frame1;
            worldclear.Click += menuevents.clearworld;
            JointMenu.Click += menuevents.JointSpace;
            CartesianMenu.Click += menuevents.CartesianSpace;
            FaLanMenu.Click += menuevents.FaLanSpace;
            LinePathButton.Click += menuevents.LinePathWindow_Click;
            PointToPathButton.Click += menuevents.PointToPath_Click;
            GridPathButton.Click += menuevents.GridPath_Click;
            RotunditPathButton.Click += menuevents.RotunditPathWindow_Click;
            SphericalPathButton.Click += menuevents.SphericalPathWindow_Click;
            Restbutton.Click += menuevents.ResetRobot;
            PathClickButton.Click += menuevents.PathClick;
            SerialPortButton.Click += menuevents.SerialPort;
            Controlbutton2183.Click += menuevents.Control2183_init;
            SpeedSet.Click += menuevents.Speedset;
            tool.Click += menuevents.toolset;

            clearrobot.Click += menuevents.clearrobot;
            robotload.Click += menuevents.LoadRobot;

            ChartButton.Click+= menuevents.ChartWindow_Click;
            ThreePathButton.Click+=menuevents.ThreePath_Click;
            Q6DrawButton.Click += menuevents.Q6Draw_Click;
            A6B6DrawButton.Click+=menuevents.A6B6Draw_Click;
            grep.Click += menuevents.Grep;
            grep2.Click += menuevents.Grep2;
        }
        public static string GetCurSourceFileName()

        {
            string path = Directory.GetCurrentDirectory();
            var p = Directory.GetParent(path);
            string s = p.Parent.Parent.FullName;
            string ss = s + "\\3D_Models";
            string[] modelilst = Directory.GetFiles(ss);
            return s;
        }


        private void InitCom()
        {
            string[] ports;
            if(menuevents.ports!=null&&menuevents.ports.Length>=0)
                ports=menuevents.ports;
            else
                ports = SerialPort.GetPortNames();
            menuevents.serialPortUtils.OpenClosePort(ports[0], 115200);
        }
        /*@name     InitCoder
        * @brief   上电获取各个关节角度编码器初始值
        */
        private void InitCoder()
        {

            List<int> deviceAddresses = new List<int> { 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6 };

            menuevents.serialPortUtils.LoadAddress(deviceAddresses);
            double[] angles = new double[6];

            for (int i = 1; i < 7; i++)
            {



                bool single = true;
                double angle = 0;
                int numcircle = 0;
                menuevents.serialPortUtils.StartPolling(single);
                Thread.Sleep(300);
                angle = menuevents.serialPortUtils.Angle;
                menuevents.serialPortUtils.Angle = 0;

                single = false;
                menuevents.serialPortUtils.StartPolling(single);
                Thread.Sleep(300);
                numcircle = menuevents.serialPortUtils.NumCircle;


                //if (MainWindow.mainwindow.simulation == false)
                angles[i - 1] = Control2183.Control2183.control.InitAngle(numcircle, angle, i);

            }



            angles[0] = angles[0] - 27.702064183091167;// 2 1 3 4 6 5
            angles[1] = angles[1] - 3.2096049802147761 + 0.0102 + 0.0022;
            angles[2] = -(angles[2] + 3.5820997933253178 + 3.8846 + 0.0446);
            angles[3] = -(angles[3] - 1.2262836216633357 + 0.423 - 0.4023);
            angles[4] = -(angles[4] + 51.505606840471515 + 4.832 + 0.157);
            angles[5] = -(angles[5] - 10.550751123530128);

            //耦合补偿
            angles[5] = angles[5] + 0.0125 * angles[4];


            MainWindow.mainwindow.OldAngles = angles;
            MainWindow.mainwindow.ForwardMove(angles);

            if (MainWindow.mainwindow.contentcon.Content == menuevents.frame1)
                JointMovePage.jointmovepage.Inint();
            else
                CartesianMovePage.cartesianmovpage.Inint();

        }
        private void InitControl2183()
        {

            var control = new Control2183.Control2183();
        }




        /*@name     ForwardMove
         *@brief    利用封装有各个关节的列表，生成Transform3DGroup，再对RobotModel的每个关节进行运动变换
         *          更新并存储在当前位置RightNowPosition的位姿。
         *@param    double[] angles:6个界面上的关节角度值（角度制）
         */
        public void ForwardMove(double[] angles)
        {
            var F1 = new Transform3DGroup();
            var R1 = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(ListJoints[0].rotAxisX, ListJoints[0].rotAxisY, ListJoints[0].rotAxisZ), angles[0]), new Point3D(ListJoints[0].rotPointX, ListJoints[0].rotPointY, ListJoints[0].rotPointZ));
            F1.Children.Add(R1);

            var F2 = new Transform3DGroup();
            var R2 = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(ListJoints[1].rotAxisX, ListJoints[1].rotAxisY, ListJoints[1].rotAxisZ), angles[1]), new Point3D(ListJoints[1].rotPointX, ListJoints[1].rotPointY, ListJoints[1].rotPointZ));
            F2.Children.Add(R2);
            F2.Children.Add(F1);

            var F3 = new Transform3DGroup();
            var R3 = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(ListJoints[2].rotAxisX, ListJoints[2].rotAxisY, ListJoints[2].rotAxisZ), angles[2]), new Point3D(ListJoints[2].rotPointX, ListJoints[2].rotPointY, ListJoints[2].rotPointZ));
            F3.Children.Add(R3);
            F3.Children.Add(F2);

            var F4 = new Transform3DGroup();
            var R4 = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(ListJoints[3].rotAxisX, ListJoints[3].rotAxisY, ListJoints[3].rotAxisZ), angles[3]), new Point3D(ListJoints[3].rotPointX, ListJoints[3].rotPointY, ListJoints[3].rotPointZ));
            F4.Children.Add(R4);
            F4.Children.Add(F3);

            var F5 = new Transform3DGroup();
            var R5 = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(ListJoints[4].rotAxisX, ListJoints[4].rotAxisY, ListJoints[4].rotAxisZ), angles[4]), new Point3D(ListJoints[4].rotPointX, ListJoints[4].rotPointY, ListJoints[4].rotPointZ));
            F5.Children.Add(R5);
            F5.Children.Add(F4);

            var F6 = new Transform3DGroup();
            var R6 = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(ListJoints[5].rotAxisX, ListJoints[5].rotAxisY, ListJoints[5].rotAxisZ), angles[5]), new Point3D(ListJoints[5].rotPointX, ListJoints[5].rotPointY, ListJoints[5].rotPointZ));
            F6.Children.Add(R6);
            F6.Children.Add(F5);



            group.Children[0].Transform = F6;
            group.Children[1].Transform = F6;
            group.Children[2].Transform = F6;


            //var F7 = new Transform3DGroup();
            // var rotation = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), 180));
            //F7.Children.Add(rotation);
            //F7.Children.Add(XYZtransform);
            //F7.Children.Add(F6);
            //coordinateSystemVisual3D.Transform = F7;


            List<Transform3DGroup> listTF = new List<Transform3DGroup>() { F1, F2, F3, F4, F5, F6 };

            for (int i = 0; i < 6; i++)
            {
                RobotModel.Children[i + 1].Transform = listTF[i];
            }
            if (menuevents != null && menuevents.toolmodel3d != null)
            {

                var f = new Transform3DGroup();
                f.Children.Add(menuevents.tooltd);
                f.Children.Add(F6);
                menuevents.toolmodel3d.Transform = f;
            }

            double[] JointRad = BasicAlgorithm.AngleToRad(angles);
            JointPosition jointposition = new JointPosition(JointRad);
            PathClass path = new PathClass(jointposition);
            path.FK(GripToTool);


            x.Content = (path.Points.X * 1000).ToString("0.00");
            y.Content = (path.Points.Y * 1000).ToString("0.00");
            z.Content = (path.Points.Z * 1000).ToString("0.00");
            A.Content = (path.Points.RX * 180 / Math.PI).ToString("0.00");
            B.Content = (path.Points.RY * 180 / Math.PI).ToString("0.00");
            c.Content = (path.Points.RZ * 180 / Math.PI).ToString("0.00");

            double[] Radangles = BasicAlgorithm.AngleToRad(angles);
            RightNowPosition.Joints = new JointPosition(Radangles);
            RightNowPosition.Pose = new CartesianPosition(PathClass.FK(Radangles, MainWindow.mainwindow.GripToTool));


        }


        /*@name CarPathPointsLocalSub
         *@brief   选择空间曲线中的上个点，与当前机械臂所在位姿点进行插值。记录插值生成的点：movepositions
         *         同时启用定时器，触发定时器的Tick事件，让机械臂根据插值的点进行运动
         */
        private void CarPathPointsLocalSub(object sender, RoutedEventArgs e)
        {
            timer.Stop();
            menuevents.MpCount = 0;

            if (menuevents.CpCount != 0)
            {
                menuevents.CpCount--;
            }
            else
            {
                menuevents.CpCount = menuevents.CartesianPositions.Count - 1;
            }


            menuevents.MovePositions = Position.MoveL(RightNowPosition, menuevents.CartesianPositions[menuevents.CpCount].mmTom(), 0.5, MainWindow.mainwindow.GripToTool);

            //if (menuevents.CpCount == menuevents.CartesianPositions.Count - 1) { menuevents.CpCount = 0; }
            if (simulation == false)
                Control2183.Control2183.control.Command_2183(menuevents.MovePositions, Control2183Speed);
            timer.Start();
        }
        /*@name CarPathPointsLocalAdd
         *@brief   选择空间曲线中的下个点，与当前机械臂所在位姿点进行插值。记录插值生成的点：movepositions
         *         同时启用定时器，触发定时器的Tick事件，让机械臂根据插值的点进行运动
         */
        private void CarPathPointsLocalAdd(object sender, RoutedEventArgs e)
        {
            timer.Stop();
            menuevents.MpCount = 0;
            menuevents.MovePositions = Position.MoveL(RightNowPosition, menuevents.CartesianPositions[menuevents.CpCount].mmTom(), 0.5, MainWindow.mainwindow.GripToTool);

            if (menuevents.CpCount != menuevents.CartesianPositions.Count)
                menuevents.CpCount++;
            if (menuevents.CpCount == menuevents.CartesianPositions.Count) { menuevents.CpCount = 0; }
            if (simulation == false)
                Control2183.Control2183.control.Command_2183(menuevents.MovePositions, Control2183Speed);
            timer.Start();
        }
        /*@name     RobotMoveTick
         *@brief    机械臂根据两点生成的movepositions进行运动
         */
        private void RobotMoveTick(object? sender, EventArgs e)
        {
            double[] rad = new double[6];
            double[] angles = new double[6];
            rad = menuevents.MovePositions[menuevents.MpCount].Joints.Joints.ToArray();
            angles = BasicAlgorithm.RadToAngle(rad);
            ForwardMove(angles);
            if (contentcon.Content == menuevents.frame1)
                JointMovePage.jointmovepage.Inint();
            else
                CartesianMovePage.cartesianmovpage.Inint();
            menuevents.MpCount++;
            if (menuevents.MpCount == menuevents.MovePositions.Count)
            {
                menuevents.MpCount = 0;
                timer.Stop();
            }

        }

        private void CarpathPointsContinous(object sender, RoutedEventArgs e)
        {

            if (menuevents.MovePositions.Count != 0)
                menuevents.MovePositions.Clear();
            int count = 0;
            List<Position> Movlposition = new List<Position>();
            foreach (var CartesianPosition in menuevents.CartesianPositions)
            {
                if (count == 0)
                    Movlposition = Position.MoveL(RightNowPosition, CartesianPosition.mmTom(), 0.5, MainWindow.mainwindow.GripToTool);
                else if (count != menuevents.CartesianPositions.Count - 1)
                    Movlposition = Position.MoveL(menuevents.MovePositions[menuevents.MovePositions.Count - 1], menuevents.CartesianPositions[count + 1].mmTom(), 0.5, MainWindow.mainwindow.GripToTool);
                foreach (var p in Movlposition)
                {
                    menuevents.MovePositions.Add(p);
                }

                count++;
            }
            Console.WriteLine(menuevents.MovePositions.Count);
            if (simulation == false)
                Control2183.Control2183.control.Command_2183(menuevents.MovePositions, Control2183Speed);
            timer.Start();


        }



        public void CartesianPositionList_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            timer.Stop();

            menuevents.MpCount = 0;
            bool flag = true;
            if (menuevents.MovePositions.Count != 0)
            { menuevents.MovePositions.Clear(); }
            if (menuevents.PathPoints != null)
            {
                menuevents.PathMovePoints.Children.Clear();
                menuevents.PathMovePoints.Points.Clear();
                viewPort3d.Children.Remove(menuevents.PathMovePoints);
            }


            ListBox listBox = sender as ListBox;
            if (SingleMove)
            {
                if (listBox != null && listBox.SelectedItem != null)
                {
                    var selectedItem = listBox.SelectedItem;
                    CartesianPosition ca = selectedItem as CartesianPosition;
                    menuevents.MovePositions = Position.MoveL(RightNowPosition, ca.mmTom(), 0.5, MainWindow.mainwindow.GripToTool);
                    //if(menuevents.CpTotalCount!=menuevents.CartesianPositions.Count)
                    timer.Start();
                    if (simulation == false)
                        Control2183.Control2183.control.Command_2183(menuevents.MovePositions, Control2183Speed);
                    menuevents.ListBoxLidex = listBox.SelectedIndex;
                }
            }
            else
            {
                if (listBox != null && listBox.SelectedItem != null && NewIKFlag == false)
                {
                    var f = Math.Abs(listBox.SelectedIndex - menuevents.ListBoxLidex) + 1;
                    while (f != 0)
                    {
                        CartesianPosition ca = listBox.Items[menuevents.ListBoxLidex] as CartesianPosition;
                        if (flag)
                        {
                            var list = Position.MoveL(RightNowPosition, ca.mmTom(), 0.5, MainWindow.mainwindow.GripToTool);
                            foreach (var p in list) { menuevents.MovePositions.Add(p); }
                            flag = false;
                        }
                        else
                        {
                            var list = Position.MoveL(menuevents.MovePositions[menuevents.MovePositions.Count - 1], ca.mmTom(), 0.5, MainWindow.mainwindow.GripToTool);
                            foreach (var p in list) { menuevents.MovePositions.Add(p); }
                        }
                        f--;
                        if (listBox.SelectedIndex > menuevents.ListBoxLidex)
                            menuevents.ListBoxLidex++;
                        else
                            menuevents.ListBoxLidex--;
                    }
                }
                else if (NewIKFlag == true)
                {
                    //CartesianPosition ca = listBox.Items[menuevents.ListBoxLidex] as CartesianPosition;
                    //var list = Position.MoveL(RightNowPosition, ca.mmTom(), 0.5, MainWindow.mainwindow.GripToTool);
                    //foreach (var p in list) { menuevents.MovePositions.Add(p); }
                    var f = Math.Abs(listBox.SelectedIndex - menuevents.ListBoxLidex) + 1;
                        while(f != 0)
                        {
                            menuevents.MovePositions.Add(NewIK.IK_New(menuevents.CartesianPositions[menuevents.ListBoxLidex].mmTom()));
                            if (menuevents.ListBoxLidex < listBox.SelectedIndex)
                                {
                                    menuevents.ListBoxLidex++;
                                }
                                else { menuevents.ListBoxLidex--; }
                            f--;   
                        }
                    
                 }
                
                menuevents.ListBoxLidex = listBox.SelectedIndex;
                // if (menuevents.CpTotalCount != menuevents.CartesianPositions.Count)
                timer.Start();

                if (simulation == false)
                    Control2183.Control2183.control.Command_2183(menuevents.MovePositions, Control2183Speed);
            }
            if (NewIKFlag == false)
            {
                foreach (var p in menuevents.MovePositions)
                {
                    // menuevents.PathMovePoints.Points.Add(new Point3D(p.Pose.X * 1000, p.Pose.Y * 1000, p.Pose.Z * 1000)); 
                    double[] JointRad = p.Joints.Joints.ToArray();
                    JointPosition jointposition = new JointPosition(JointRad);
                    PathClass path = new PathClass(jointposition);
                    path.FK(GripToTool);
                    menuevents.PathMovePoints.Points.Add(new Point3D(path.Points.X * 1000, path.Points.Y * 1000, path.Points.Z * 1000));
                }
            }
            else
            {
                for (int i = 0; i < NewIK.Q3Value.Length; i++)
                {
                    double[] JointRad = new double[6] { NewIK.Q1Value[i], NewIK.Q2Value[i], NewIK.Q3Value[i], NewIK.Q4Value[i], NewIK.Q5Value[i], NewIK.Q6Value[i] };
                    JointPosition jointposition = new JointPosition(JointRad);
                    var path = NewIK.FKNewRad(JointRad);
                    menuevents.PathMovePoints.Points.Add(new Point3D(path.X, path.Y, path.Z));
                }
            }
            viewPort3d.Children.Add(menuevents.PathMovePoints);
            //menuevents.CpTotalCount=menuevents.CartesianPositions.Count;
        }

        private void servo(object sender, RoutedEventArgs e)
        {

            if ((string)servoflag.Content == "伺服开")
            {
                servoflag.Content = "伺服关";
                servoflag.Background = new SolidColorBrush(Colors.Red);
                if (Control2183.Control2183.control.IsConnect)
                {
                    Control2183.Control2183.control.Open_servo();
                }
            }
            else if ((string)servoflag.Content == "伺服关")
            {
                servoflag.Content = "伺服开";
                servoflag.Background = new SolidColorBrush(Colors.AliceBlue);
                if (Control2183.Control2183.control.IsConnect)
                {
                    Control2183.Control2183.control.Close_servo();
                }

            }
        }

        private void Comtest(object sender, RoutedEventArgs e)
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
                    for (int i = 0; i < menuevents.CartesianPositions.Count; i++)
                        contents.Add(menuevents.CartesianPositions[i].X.ToString() + " " + menuevents.CartesianPositions[i].Y.ToString() + " "
                            + menuevents.CartesianPositions[i].Z.ToString() + " " + menuevents.CartesianPositions[i].RX.ToString() + " "
                            + menuevents.CartesianPositions[i].RY.ToString() + " " + menuevents.CartesianPositions[i].RZ.ToString());

                    File.WriteAllLines(saveFileDialog.FileName, contents);
                  
                }
                catch (Exception ex)
                {
                    // 处理异常
                }
            }
           
        }

        private void AddPositions(object sender, RoutedEventArgs e)
        {


            menuevents.CartesianPositions.Add(RightNowPosition.Pose.mTomm());

            CartesianPositionList.ItemsSource = null;
            CartesianPositionList.ItemsSource = menuevents.CartesianPositions;
            return;


            ////390,160,40,180,0,180
            ////CartesianPosition cartesianPosition = new CartesianPosition(390/1000,160/1000,180+450,Math.PI,0,0);

            //menuevents.CartesianPositions.Add(new CartesianPosition(340, 160, 40 + 456,   Math.PI ,  0,  0));
            //menuevents.CartesianPositions.Add(new CartesianPosition(340, -160, 40 + 456,  Math.PI ,  0,  0));
            //menuevents.CartesianPositions.Add(new CartesianPosition(660, -160, 360 + 456, Math.PI ,  0,  0));
            //menuevents.CartesianPositions.Add(new CartesianPosition(660, 160, 360 + 456,  Math.PI ,  0,  0));
            //menuevents.CartesianPositions.Add(new CartesianPosition(500, 0, 200 + 456,    Math.PI ,  0,  0));
            ////menuevents.CartesianPositions.Add(new CartesianPosition(500, 0, 200 + 456, Math.PI, 0, 0));

            //CartesianPositionList.ItemsSource = null;
            //CartesianPositionList.ItemsSource = menuevents.CartesianPositions;

            //return;
            ////menuevents.MovePositions.Add(RightNowPosition);





            ////if (MenuEvents.MenuEvents.Points != null)
            ////{
            ////    viewPort3d.Children.Remove(Points);
            ////    Points.Points.Clear();
            ////}
            ////foreach (CartesianPosition position in gridpathpoints)
            ////{
            ////    MenuEvents.MenuEvents.Points.Points.Add(new Point3D(position.X, position.Y, position.Z));
            ////}
            ////viewPort3d.Children.Add(Points);
            ////CartesianPositions = gridpathpoints;

            //double[,] datata = new double[50, 6]
            //    { { -0.0001695,-125.6360585,217.154152,-0.0002515,75.9607989,-0.0044495                    },
            //    { 0.0004312,-91.8415431,195.1566676,-0.0000266,76.6869311,0.0006443                        },
            //    { -9.387511,-75.8726421,189.4377216,-0.0004266,66.4371182,-9.3870035                       },
            //    { -19.6807845,-69.2123413,190.239836,-0.0012266,58.9763572,-19.6802436                     },
            //    { -17.7814674,-44.4378762,189.9376335,-0.0010414,21.1821301,-17.7829394                    },
            //    { 7.9197477,-58.0116834,189.0981092,0.0013867,48.9174205,23.659295                         },
            //    { 9.5604401,-73.4529676,211.3002415,0.00152,42.1564024,9.559394                            },
            //    { 26.1173295,-60.2002677,203.6985489,0.0035182,36.5107815,25.1837083                       },
            //    { 69.1446115,-59.5734082,197.9509144,-3.212946,29.7936734,81.3788493                       },
            //    { 10.2474561,-60.6202222,187.9090021,0.0012564,52.7178635,7.8125972                        },
            //    { -30.1520578,-47.0113493,166.4712685,-0.0029865,60.5449961,-60.3137885                    },
            //    { -37.7533611,-61.7370377,200.992273,-0.005114,40.7492952,-55.7476178                      },
            //    { -32.9159194,-79.7237035,225.9600027,-0.0049331,33.7683863,-32.9107666                    },
            //    { -66.5225334,-66.1164111,207.5724843,-0.0080622,38.5473448,-77.2471046                    },
            //    { -69.9349536,-60.7194912,182.5440125,-0.005713,58.1775425,-81.4726094                     },
            //    { -70.6319338,-59.5658935,199.4311272,-0.0083826,40.1367428,-82.2270489                    },
            //    { -81.9996274,-78.2183347,192.1971984,-0.0057931,66.0221178,-92.1998833                    },
            //    { -88.8047855,-99.9081252,228.7189509,-0.0069225,51.1911956,-101.0886638                   },
            //    { -56.687349,-92.390547,203.1806793,-0.0028832,69.2154269,-68.6627309                      },
            //    { 6.166783,-130.6717722,228.7447156,0.0038933,81.9349229,6.1690946                         },
            //    { 49.7277309,-104.4302506,213.5988219,0.0086522,70.837247,51.814556                        },
            //    { 42.9516825,-87.5441586,222.846325,0.0111294,44.7041116,52.759723                         },
            //    { 39.1551187,-72.1900363,221.2371549,0.0145401,30.9593205,48.7370227                       },
            //    { 30.4527848,-53.9426169,198.3032052,-38.5638373,35.6446942,69.7618811                     },
            //    { 28.4794685,-61.3329379,181.022178,0.0071084,60.3210303,37.5163639                        },
            //    { 34.6591317,-49.4863543,172.0274086,0.0085086,57.4688681,38.0479448                       },
            //    { 58.0915061,-85.2281824,221.598512,0.012386,43.6326098,68.3962334                         },
            //    { 65.3631405,-70.0091807,195.1549182,0.0106627,54.8581887,80.2939342                       },
            //    { 82.0685,-69.1729,210.69,-23.5174,38.4821,119.692                                         },
            //    { 74.3637605,-88.7727597,234.7564536,0.0156796,34.0157602,99.084744                        },
            //    { 42.2421182,-89.3778318,222.9682646,-15.4028749,59.4352429,60.5023363                     },
            //    { -16.5909169,-94.5181917,218.3335635,-18.364,49.2506447,-0.3230455                        },
            //    { -30.1202364,-100.7420109,218.5502631,-18.3632131,44.3920489,-32.3627523                  },
            //    { -20.7003762,-61.798567,194.8390871,-15.7838947,42.2799453,-15.9400757                    },
            //    { 6.7222769,-67.0343864,202.4833027,-9.9192306,44.7542677,26.2658358                       },
            //    { -3.410263,-79.788273,221.4636719,37.9720869,23.6319363,-23.4769192                       },
            //    { -3.956554,-75.7354476,235.9949603,67.7050722,-0.0785771,-66.9761305                      },
            //    { -2.7961964,-55.8824495,204.4320242,60.6836015,2.1729683,-60.9219022                      },
            //    { -13.5516125,-61.5978524,199.2032605,5.8738459,31.0681711,-36.9174601                     },
            //    { -45.2845116,-76.9184417,220.7343541,14.2236557,44.254509,-63.9841836                     },
            //    { -44.4912301,-75.8535095,206.0412702,-89.436985,19.7059364,37.3294731                     },
            //    { -61.6388087,-78.7796576,204.3659111,-117.9387387,-15.8485112,49.46672                    },
            //    { -85.2048526,-96.8378088,215.509718,-148.8868589,-25.0377761,60.3823445                   },
            //    { -53.3440819,-64.2051452,202.2477225,-120.5449791,-12.7453766,56.3022039                  },
            //    { -25.0589905,-61.0166656,202.2487898,-91.9685783,4.9551086,59.8936534                     },
            //    { 25.6904,-73.1048,216.267,-30.5297,36.3454,63.9292                                        },
            //    { 19.4440815,-84.7215215,222.6523817,-17.0824803,32.7998178,31.9950524                     },
            //    { 4.2986006,-89.5594833,216.6173133,-11.1089714,55.2390066,8.7984009                       },
            //    { -4.7288206,-72.0846875,221.9537121,-21.8746475,7.8094907,8.0313788                       },
            //    { 0,-90,180,0,90,0                                                                         }};


            //for(int i = 0; i <50;i++)
            //{
            //    double[] dddda = new double[6] { datata[i,0], datata[i, 1]+90, datata[i, 2]-180, datata[i, 3], datata[i, 4], datata[i, 5] };

            //    Position TPosition = new Position(BasicAlgorithm.AngleToRad(dddda));

            //    menuevents.CartesianPositions.Add(TPosition.Pose.mTomm());

            //    CartesianPositionList.ItemsSource = null;
            //    CartesianPositionList.ItemsSource = menuevents.CartesianPositions;

            //}



        }


        private void SpeedSetSilder(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            int max = (int)speed.Maximum;
            int value = (int)speed.Value;
            if (value <= 10)
            {
                speed.Value = 10;
                //RobotSpeed(10, 0.8);
                if (speedshow != null)
                    speedshow.Content = "Slow";
            }
            else if (value > 10 && value <= 35)
            {
                speed.Value = 35;
                //RobotSpeed(11, 0.4);
                if (speedshow != null)
                    speedshow.Content = "Medium-";
            }
            else if (value > 35 && value <= 65)
            {
                //RobotSpeed(9,0.7);
                speed.Value = 65;
                if (speedshow != null)
                    speedshow.Content = "Medium+";
            }
            else
            {
                //RobotSpeed(22,1.4);
                speed.Value = 95;
                if (speedshow != null)
                    speedshow.Content = "Fast";
            }
            if (JointMovePage.jointmovepage != null)
            {
                JointMovePage.jointmovepage.Joint1.IntervalTime = max - value + 1;
                JointMovePage.jointmovepage.Joint2.IntervalTime = max - value + 1;
                JointMovePage.jointmovepage.Joint3.IntervalTime = max - value + 1;
                JointMovePage.jointmovepage.Joint4.IntervalTime = max - value + 1;
                JointMovePage.jointmovepage.Joint5.IntervalTime = max - value + 1;
                JointMovePage.jointmovepage.Joint6.IntervalTime = max - value + 1;


            }
            if (CartesianMovePage.cartesianmovpage != null)
            {
                CartesianMovePage.cartesianmovpage.X.IntervalTime = max - value + 1;
                CartesianMovePage.cartesianmovpage.Y.IntervalTime = max - value + 1;
                CartesianMovePage.cartesianmovpage.Z.IntervalTime = max - value + 1;
                CartesianMovePage.cartesianmovpage.A.IntervalTime = max - value + 1;
                CartesianMovePage.cartesianmovpage.B.IntervalTime = max - value + 1;
                CartesianMovePage.cartesianmovpage.C.IntervalTime = max - value + 1;


            }
            if (FaLanMovepage.falanmovepage != null)
            {
                FaLanMovepage.falanmovepage.X.IntervalTime = max - value + 1;
                FaLanMovepage.falanmovepage.Y.IntervalTime = max - value + 1;
                FaLanMovepage.falanmovepage.Z.IntervalTime = max - value + 1;
                FaLanMovepage.falanmovepage.A.IntervalTime = max - value + 1;
                FaLanMovepage.falanmovepage.B.IntervalTime = max - value + 1;
                FaLanMovepage.falanmovepage.C.IntervalTime = max - value + 1;



            }



            if (Control2183.Control2183.control.IsConnect)
            {
                // Control2183.Control2183.control.Command_speed((int)(speed.Value / speed.Maximum * 100));
            }

        }
        private void RobotSpeed(int intertime, double intervalue)
        {
            if (JointMovePage.jointmovepage != null)
            {
                JointMovePage.jointmovepage.Joint1.IntervalTime = intertime;
                JointMovePage.jointmovepage.Joint2.IntervalTime = intertime;
                JointMovePage.jointmovepage.Joint3.IntervalTime = intertime;
                JointMovePage.jointmovepage.Joint4.IntervalTime = intertime;
                JointMovePage.jointmovepage.Joint5.IntervalTime = intertime;
                JointMovePage.jointmovepage.Joint6.IntervalTime = intertime;

                JointMovePage.jointmovepage.Joint1.IncrementValue = intervalue;
                JointMovePage.jointmovepage.Joint2.IncrementValue = intervalue;
                JointMovePage.jointmovepage.Joint3.IncrementValue = intervalue;
                JointMovePage.jointmovepage.Joint4.IncrementValue = intervalue;
                JointMovePage.jointmovepage.Joint5.IncrementValue = intervalue;
                JointMovePage.jointmovepage.Joint6.IncrementValue = intervalue;
            }
            if (CartesianMovePage.cartesianmovpage != null)
            {
                CartesianMovePage.cartesianmovpage.X.IntervalTime = intertime;
                CartesianMovePage.cartesianmovpage.Y.IntervalTime = intertime;
                CartesianMovePage.cartesianmovpage.Z.IntervalTime = intertime;
                CartesianMovePage.cartesianmovpage.A.IntervalTime = intertime;
                CartesianMovePage.cartesianmovpage.B.IntervalTime = intertime;
                CartesianMovePage.cartesianmovpage.C.IntervalTime = intertime;

                CartesianMovePage.cartesianmovpage.X.IncrementValue = intervalue;
                CartesianMovePage.cartesianmovpage.Y.IncrementValue = intervalue;
                CartesianMovePage.cartesianmovpage.Z.IncrementValue = intervalue;
                CartesianMovePage.cartesianmovpage.A.IncrementValue = intervalue;
                CartesianMovePage.cartesianmovpage.B.IncrementValue = intervalue;
                CartesianMovePage.cartesianmovpage.C.IncrementValue = intervalue;
            }
            if (FaLanMovepage.falanmovepage != null)
            {
                FaLanMovepage.falanmovepage.X.IntervalTime = intertime;
                FaLanMovepage.falanmovepage.Y.IntervalTime = intertime;
                FaLanMovepage.falanmovepage.Z.IntervalTime = intertime;
                FaLanMovepage.falanmovepage.A.IntervalTime = intertime;
                FaLanMovepage.falanmovepage.B.IntervalTime = intertime;
                FaLanMovepage.falanmovepage.C.IntervalTime = intertime;

                FaLanMovepage.falanmovepage.X.IncrementValue = intervalue;
                FaLanMovepage.falanmovepage.Y.IncrementValue = intervalue;
                FaLanMovepage.falanmovepage.Z.IncrementValue = intervalue;
                FaLanMovepage.falanmovepage.A.IncrementValue = intervalue;
                FaLanMovepage.falanmovepage.B.IncrementValue = intervalue;
                FaLanMovepage.falanmovepage.C.IncrementValue = intervalue;

            }
        }

        private void TESTe(object sender, RoutedEventArgs e)
        {

            //CartesianPosition sss = new CartesianPosition(2985.6677, 2517.8562, 588.2374, -0.7489, -0.1556, -90.3583);

            //sss = sss.AngleToRad();

            //Matrix4x4 dada = sss.ToTrans();
            ////Matrix4x4.Invert(dada, out dada);



            //Vector4 vector = new Vector4( 340, (float)(160- 0.9354), 40, 1  );
            //Vector4 vector2 = new Vector4(340, (float)(-160 - 0.9354), 40, 1  );
            //Vector4 vector3 = new Vector4(660, (float)(-160- 0.9354), 360, 1 );
            //Vector4 vector4 = new Vector4(660, (float)(160 -0.9354), 360 , 1 );
            //Vector4 vector5 = new Vector4(500, (float)(0 -0.9354), 200 , 1   );
            ////Vector4 vector = new Vector4(340, 160, 40, 1);

            //Matrix4x4 dadaT = Matrix4x4.Transpose(dada);


            ////Vector4 result = Vector4.Transform  (vector, dada);
            ////Vector4 result1 = Vector4.Transform(vector2, dada);
            ////Vector4 result2 = Vector4.Transform(vector3, dada);
            ////Vector4 result3 = Vector4.Transform(vector4, dada);
            ////Vector4 result4 = Vector4.Transform(vector5, dada);


            //Vector4 result_ = Vector4.Transform(vector, dadaT);
            //Vector4 result1_ = Vector4.Transform(vector2, dadaT);
            //Vector4 result2_ = Vector4.Transform(vector3, dadaT);
            //Vector4 result3_ = Vector4.Transform(vector4, dadaT);
            //Vector4 result4_ = Vector4.Transform(vector5, dadaT);


            //Vector4 vector6 = new  Vector4((float)2984.74579,(float)2018.18638	,(float)789.40277, 1);
            //Vector4 vector7 = new  Vector4((float)3145.08556,(float)1857.68735	,(float)947.80654, 1);
            //Vector4 vector8 = new  Vector4((float)2824.22639,(float)1859.96748	,(float)952.00419, 1);
            //Vector4 vector9 = new  Vector4((float)2822.91431,(float)2178.9784  	,(float)631.18906, 1);
            //Vector4 vector10 = new Vector4((float)3143.03799,(float)2176.706567 ,(float)627.0335111, 1);


            //Matrix4x4.Invert(dada, out dada);

            //Matrix4x4 dadaT1 = Matrix4x4.Transpose(dada);

            //Vector4 result6_ = Vector4.Transform(vector6, dadaT1);
            //Vector4 result7_ = Vector4.Transform(vector7, dadaT1);
            //Vector4 result8_ = Vector4.Transform(vector8, dadaT1);
            //Vector4 result9_ = Vector4.Transform(vector9, dadaT1);
            //Vector4 resulta_ = Vector4.Transform(vector10, dadaT1);


            //Vector4 vector2 = new Vector4(340, -160, 40 + 456, 1);
            //Vector4 vector3 = new Vector4(660, -160, 360 + 456, 1);
            //Vector4 vector4 = new Vector4(660, 160, 360 + 456, 1);
            //Vector4 vector5 = new Vector4(500, 0, 200 + 456, 1);

            /*********/


            //Matrix4x4.Invert(startRT_BaseToTool, out startRT_ToolToBase);


            //Vector4 vector = new Vector4(endRT_BaseToTool.M14, endRT_BaseToTool.M24, endRT_BaseToTool.M34, endRT_BaseToTool.M44);

            ////Matrix4x4 result = startRT_ToolToBase * endRT_BaseToTool;
            ////return new double[2] { result.M14, result.M24 };

            //Vector4 result = Vector4.Transform(vector, startRT_ToolToBase);




            //menuevents.CartesianPositions.Add(new CartesianPosition(340, 160, 40 + 456, Math.PI, 0, 0));
            //menuevents.CartesianPositions.Add(new CartesianPosition(340, -160, 40 + 456, Math.PI, 0, 0));
            //menuevents.CartesianPositions.Add(new CartesianPosition(660, -160, 360 + 456, Math.PI, 0, 0));
            //menuevents.CartesianPositions.Add(new CartesianPosition(660, 160, 360 + 456, Math.PI, 0, 0));
            //menuevents.CartesianPositions.Add(new CartesianPosition(500, 0, 200 + 456, Math.PI, 0, 0));










            //return;
            //menuevents.CpCount = 0;

            //for (int j = 0; j < 30;j++)
            //{
            //    //timer.Stop();
            //    menuevents.MpCount = 0;
            //    menuevents.MovePositions = Position.MoveL(RightNowPosition, menuevents.CartesianPositions[menuevents.CpCount].mmTom(), 0.5, MainWindow.mainwindow.GripToTool);

            //    if (menuevents.CpCount != menuevents.CartesianPositions.Count)
            //        menuevents.CpCount++;
            //    if (menuevents.CpCount == menuevents.CartesianPositions.Count) { menuevents.CpCount = 0; }
            //    if (simulation == false)
            //        Control2183.Control2183.control.Command_2183(menuevents.MovePositions, Control2183Speed);
            //    //timer.Start();



            //    List<int> deviceAddresses = new List<int> { 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6 };

            //    serialPortUtils.LoadAddress(deviceAddresses);
            //    double[] angles = new double[6];

            //    for (int i = 1; i < 7; i++)
            //    {

            //        //bool single = true;
            //        //serialPortUtils.StartPolling(single);
            //        //Thread.Sleep(200);
            //        //bool notsingle = false;
            //        //serialPortUtils.StartPolling(notsingle);
            //        //Thread.Sleep(200);

            //        bool single = true;
            //        double angle = 0;
            //        int numcircle = 0;
            //        serialPortUtils.StartPolling(single);
            //        Thread.Sleep(200);
            //        angle = serialPortUtils.Angle;
            //        serialPortUtils.Angle = 0;
            //        Console.WriteLine("angle：" + angle);
            //        Console.WriteLine("\n");
            //        single = false;
            //        serialPortUtils.StartPolling(single);
            //        Thread.Sleep(200);
            //        numcircle = serialPortUtils.NumCircle;
            //        serialPortUtils.NumCircle = 0;
            //        Console.WriteLine("numcircle：" + numcircle);
            //        Console.WriteLine("\n");
            //        angles[i - 1] = Control2183.Control2183.control.InitAngle(numcircle, angle, i);

            //    }

            //    angles[0] = angles[0] - 27.702064183091167;// 2 1 3 4 6 5
            //    angles[1] = angles[1] - 3.2096049802147761 + 0.0102 + 0.0022;
            //    angles[2] = -(angles[2] + 3.5820997933253178 + 3.8846 + 0.0446);
            //    angles[3] = -(angles[3] - 1.2262836216633357 + 0.423 - 0.4023);
            //    angles[4] = -(angles[4] + 51.505606840471515 + 4.832 + 0.157);
            //    angles[5] = -(angles[5] - 10.550751123530128);

            //    //耦合补偿
            //    angles[5] = angles[5] + 0.0125 * angles[4];

            //    MainWindow.mainwindow.OldAngles = angles;
            //    Thread.Sleep(2000);
            //    MainWindow.mainwindow.ForwardMove(angles);


            //    Thread.Sleep(5000);

            //    Thread.Sleep(5000);


            //}





            //double[,] datata = new double[50, 6]
            //    { { -0.0001695,-125.6360585,217.154152,-0.0002515,75.9607989,-0.0044495                    },
            //    { 0.0004312,-91.8415431,195.1566676,-0.0000266,76.6869311,0.0006443                        },
            //    { -9.387511,-75.8726421,189.4377216,-0.0004266,66.4371182,-9.3870035                       },
            //    { -19.6807845,-69.2123413,190.239836,-0.0012266,58.9763572,-19.6802436                     },
            //    { -17.7814674,-44.4378762,189.9376335,-0.0010414,21.1821301,-17.7829394                    },
            //    { 7.9197477,-58.0116834,189.0981092,0.0013867,48.9174205,23.659295                         },
            //    { 9.5604401,-73.4529676,211.3002415,0.00152,42.1564024,9.559394                            },
            //    { 26.1173295,-60.2002677,203.6985489,0.0035182,36.5107815,25.1837083                       },
            //    { 69.1446115,-59.5734082,197.9509144,-3.212946,29.7936734,81.3788493                       },
            //    { 10.2474561,-60.6202222,187.9090021,0.0012564,52.7178635,7.8125972                        },
            //    { -30.1520578,-47.0113493,166.4712685,-0.0029865,60.5449961,-60.3137885                    },
            //    { -37.7533611,-61.7370377,200.992273,-0.005114,40.7492952,-55.7476178                      },
            //    { -32.9159194,-79.7237035,225.9600027,-0.0049331,33.7683863,-32.9107666                    },
            //    { -66.5225334,-66.1164111,207.5724843,-0.0080622,38.5473448,-77.2471046                    },
            //    { -69.9349536,-60.7194912,182.5440125,-0.005713,58.1775425,-81.4726094                     },
            //    { -70.6319338,-59.5658935,199.4311272,-0.0083826,40.1367428,-82.2270489                    },
            //    { -81.9996274,-78.2183347,192.1971984,-0.0057931,66.0221178,-92.1998833                    },
            //    { -88.8047855,-99.9081252,228.7189509,-0.0069225,51.1911956,-101.0886638                   },
            //    { -56.687349,-92.390547,203.1806793,-0.0028832,69.2154269,-68.6627309                      },
            //    { 6.166783,-130.6717722,228.7447156,0.0038933,81.9349229,6.1690946                         },
            //    { 49.7277309,-104.4302506,213.5988219,0.0086522,70.837247,51.814556                        },
            //    { 42.9516825,-87.5441586,222.846325,0.0111294,44.7041116,52.759723                         },
            //    { 39.1551187,-72.1900363,221.2371549,0.0145401,30.9593205,48.7370227                       },
            //    { 30.4527848,-53.9426169,198.3032052,-38.5638373,35.6446942,69.7618811                     },
            //    { 28.4794685,-61.3329379,181.022178,0.0071084,60.3210303,37.5163639                        },
            //    { 34.6591317,-49.4863543,172.0274086,0.0085086,57.4688681,38.0479448                       },
            //    { 58.0915061,-85.2281824,221.598512,0.012386,43.6326098,68.3962334                         },
            //    { 65.3631405,-70.0091807,195.1549182,0.0106627,54.8581887,80.2939342                       },
            //    { 82.0685,-69.1729,210.69,-23.5174,38.4821,119.692                                         },
            //    { 74.3637605,-88.7727597,234.7564536,0.0156796,34.0157602,99.084744                        },
            //    { 42.2421182,-89.3778318,222.9682646,-15.4028749,59.4352429,60.5023363                     },
            //    { -16.5909169,-94.5181917,218.3335635,-18.364,49.2506447,-0.3230455                        },
            //    { -30.1202364,-100.7420109,218.5502631,-18.3632131,44.3920489,-32.3627523                  },
            //    { -20.7003762,-61.798567,194.8390871,-15.7838947,42.2799453,-15.9400757                    },
            //    { 6.7222769,-67.0343864,202.4833027,-9.9192306,44.7542677,26.2658358                       },
            //    { -3.410263,-79.788273,221.4636719,37.9720869,23.6319363,-23.4769192                       },
            //    { -3.956554,-75.7354476,235.9949603,67.7050722,-0.0785771,-66.9761305                      },
            //    { -2.7961964,-55.8824495,204.4320242,60.6836015,2.1729683,-60.9219022                      },
            //    { -13.5516125,-61.5978524,199.2032605,5.8738459,31.0681711,-36.9174601                     },
            //    { -45.2845116,-76.9184417,220.7343541,14.2236557,44.254509,-63.9841836                     },
            //    { -44.4912301,-75.8535095,206.0412702,-89.436985,19.7059364,37.3294731                     },
            //    { -61.6388087,-78.7796576,204.3659111,-117.9387387,-15.8485112,49.46672                    },
            //    { -85.2048526,-96.8378088,215.509718,-148.8868589,-25.0377761,60.3823445                   },
            //    { -53.3440819,-64.2051452,202.2477225,-120.5449791,-12.7453766,56.3022039                  },
            //    { -25.0589905,-61.0166656,202.2487898,-91.9685783,4.9551086,59.8936534                     },
            //    { 25.6904,-73.1048,216.267,-30.5297,36.3454,63.9292                                        },
            //    { 19.4440815,-84.7215215,222.6523817,-17.0824803,32.7998178,31.9950524                     },
            //    { 4.2986006,-89.5594833,216.6173133,-11.1089714,55.2390066,8.7984009                       },
            //    { -4.7288206,-72.0846875,221.9537121,-21.8746475,7.8094907,8.0313788                       },
            //    { 0,-90,180,0,90,0                                                                         }};


            double[,] datata = new double[6, 6]
            {
                { -44.4912301,-75.8535095,206.0412702,-89.436985,19.7059364,37.3294731             },
                { -61.6388087,-78.7796576,204.3659111,-117.9387387,-15.8485112,49.46672            },
                { -85.2048526,-96.8378088,215.509718,-148.8868589,-25.0377761,60.3823445           },
                { -53.3440819,-64.2051452,202.2477225,-120.5449791,-12.7453766,56.3022039          },
                { -25.0589905,-61.0166656,202.2487898,-91.9685783,4.9551086,59.8936534             },
                { 25.6904,-73.1048,216.267,-30.5297,36.3454,63.9292                                }
};



            double[] dddda = new double[6] { datata[test_i, 0], datata[test_i, 1] + 90, datata[test_i, 2] - 180, datata[test_i, 3], datata[test_i, 4], datata[test_i, 5] };
            test_i++;
            Console.WriteLine(test_i);
            if (test_i == 6)
                test_i = 0;

            MainWindow.mainwindow.ForwardMove(dddda);
            if (MainWindow.mainwindow.simulation == false)
                Control2183.Control2183.control.Command_2183(dddda, MainWindow.mainwindow.Control2183Speed);

        }

        private void Loadtext(object sender, RoutedEventArgs e)
        {
            OpenFileDialog saveFileDialog = new OpenFileDialog();
            // 设置文件类型过滤器
            saveFileDialog.Filter = "文本文件|*.txt|所有文件|*.*";
            saveFileDialog.Title = "读取文本文件";
            saveFileDialog.DefaultExt = "txt";    // 默认扩展名
            saveFileDialog.AddExtension = true;   // 自动添加扩展名
            List<CartesianPosition> CartesianPositions = new List<CartesianPosition>();
            if (saveFileDialog.ShowDialog() == true)
            {
                try
                {
                    string[] lines = File.ReadAllLines(saveFileDialog.FileName);
                    foreach (string line in lines)
                    {
                        if (line.Contains("X(mm)"))
                            continue;
                        // 使用String.Split方法将每一行分割成字符串数组
                        string[] parts = line.Split(' ');
                        var position = new CartesianPosition();
                        position.X = double.Parse(parts[0]);
                        position.Y = double.Parse(parts[1]);
                        position.Z = double.Parse(parts[2]);
                        position.RX = double.Parse(parts[3]);
                        position.RY = double.Parse(parts[4]);
                        position.RZ = double.Parse(parts[5]);
                        CartesianPositions.Add(position);
                    }
                    menuevents.CartesianPositions = CartesianPositions;
                    CartesianPositionList.ItemsSource = menuevents.CartesianPositions;
                    if (menuevents.CartesianPositions.Count != 0)
                        Console.WriteLine("加载成功");
                    menuevents.PathPointsProduce();
                    viewPort3d.Children.Add(menuevents.PathPoints);

                }
                catch (Exception ex)
                {
                    Console.WriteLine("文件不存在");

                }
            }

            
        }

        private void Path_Y_Mean(object sender, RoutedEventArgs e)
        {
            double sum_y = 0;
            double mean_y = 0;
            int count = 0;
            if (menuevents.Points != null)
            {
                viewPort3d.Children.Remove(menuevents.Points);
                menuevents.Points.Points.Clear();
            }
            foreach (var item in menuevents.modelVisual3Ds.ToArray()) // 使用ToArray()来避免在迭代过程中修改集合  
            {
                viewPort3d.Children.Remove(item);
            }
            for (int i = menuevents.CpTotalCount; i < menuevents.CartesianPositions.Count; i++)
            {
                sum_y += menuevents.CartesianPositions[i].Y;
                count++;
            }
            CartesianPositionList.ItemsSource = null;
            menuevents.ListBoxLidex = 0;
            mean_y = sum_y / count;
            for (int i = menuevents.CpTotalCount; i < menuevents.CartesianPositions.Count; i++)
                menuevents.CartesianPositions[i].Y = mean_y;

            foreach (CartesianPosition position in menuevents.CartesianPositions)
            {
                menuevents.Points.Points.Add(new Point3D(position.X, position.Y, position.Z));
            }
            viewPort3d.Children.Add(menuevents.Points);

            CartesianPositionList.ItemsSource = menuevents.CartesianPositions;
            menuevents.CpTotalCount = menuevents.CartesianPositions.Count;
        }

        private void Path_X_Mean(object sender, RoutedEventArgs e)
        {
            double sum_x = 0;
            double mean_x = 0;
            int count = 0;
            if (menuevents.Points != null)
            {
                viewPort3d.Children.Remove(menuevents.Points);
                menuevents.Points.Points.Clear();
            }
            foreach (var item in menuevents.modelVisual3Ds.ToArray()) // 使用ToArray()来避免在迭代过程中修改集合  
            {
                viewPort3d.Children.Remove(item);
            }
            for (int i = menuevents.CpTotalCount; i < menuevents.CartesianPositions.Count; i++)
            {
                sum_x += menuevents.CartesianPositions[i].X;
                count++;
            }
            CartesianPositionList.ItemsSource = null;
            menuevents.ListBoxLidex = 0;
            mean_x = sum_x / count;
            for (int i = menuevents.CpTotalCount; i < menuevents.CartesianPositions.Count; i++)
                menuevents.CartesianPositions[i].X = mean_x;
            if (menuevents.Points != null)
            {
                viewPort3d.Children.Remove(menuevents.Points);
                menuevents.Points.Points.Clear();
            }
            foreach (CartesianPosition position in menuevents.CartesianPositions)
            {
                menuevents.Points.Points.Add(new Point3D(position.X, position.Y, position.Z));
            }
            viewPort3d.Children.Add(menuevents.Points);

            CartesianPositionList.ItemsSource = menuevents.CartesianPositions;
            menuevents.CpTotalCount = menuevents.CartesianPositions.Count;
        }

        private void facechanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (rectangleVisual3D != null)
            {
                viewPort3d.Children.Remove(rectangleVisual3D);
                viewPort3d.Children.Remove(sphere);
                viewPort3d.Children.Remove(coordinateSystem);
                viewPort3d.Children.Remove(arr);
            }
            var planePoint = new Point3D(
        double.Parse(MainWindow.mainwindow.X_Point.Text),
        double.Parse(MainWindow.mainwindow.Y_Point.Text),
        double.Parse(MainWindow.mainwindow.Z_Point.Text)
    );

            menuevents.planeNormal = new Vector3D(
                 MainWindow.mainwindow.PlaneNormalX.Value,
               MainWindow.mainwindow.PlaneNormalY.Value,
               MainWindow.mainwindow.PlaneNormalZ.Value
              );
            menuevents.planeNormal.Normalize();
            sphere = new SphereVisual3D
            {
                Center = planePoint, // 球心设置为平面中心
                Radius = 2,      // 球的半径
                Material = MaterialHelper.CreateMaterial(Colors.Blue), // 球的材质
            };
            rectangleVisual3D = new RectangleVisual3D
            {
                Width = 1000,
                Length = 1000,
                Normal = menuevents.planeNormal,
                Origin = planePoint,
                Material = MaterialHelper.CreateMaterial(Colors.LightBlue, 0.5) // 半透明材质
            };
            var rot = MenuEvents.MenuEvents.GetRotationMatrix(menuevents.planeNormal);
            //var ro= BasicAlgorithm.RotMatrixToRxyz(rot1);
            // var rot = BasicAlgorithm.RxyzToRotMatrix(ro[0], ro[1], ro[2]);
            Matrix3D rotationMatrix = new Matrix3D(
            rot[0, 0], rot[0, 1], rot[0, 2], 0,
            rot[1, 0], rot[1, 1], rot[1, 2], 0,
            rot[2, 0], rot[2, 1], rot[2, 2], 0,
            0, 0, 0, 1
        );
            Transform3DGroup transform = new Transform3DGroup();
            transform.Children.Add(new MatrixTransform3D(rotationMatrix)); // 旋转变换
            transform.Children.Add(new TranslateTransform3D(planePoint.X, planePoint.Y, planePoint.Z)); // 平移变换


            coordinateSystem.Transform = transform;

            arr = new ArrowVisual3D
            {
                Point2 = planePoint,
                Point1 = new Point3D(
                    planePoint.X + menuevents.planeNormal.X,
                    planePoint.Y + menuevents.planeNormal.Y,
                    planePoint.Z + menuevents.planeNormal.Z
                ),
                Diameter = 5,
                Fill = new SolidColorBrush(Colors.Red)
            };

            viewPort3d.Children.Add(arr);
            viewPort3d.Children.Add(rectangleVisual3D);
            viewPort3d.Children.Add(sphere);
            viewPort3d.Children.Add(coordinateSystem);
        }

        private void OnPortSubmenuOpened(object sender, RoutedEventArgs e)
        {
            _viewModel.RefreshPorts();
        }
    }
}
