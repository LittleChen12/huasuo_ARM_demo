using Robot_window.UI_Page;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Controls;
using System.Windows;
using Robot_window.ViewModels;
using RobotLibrary;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;
using System.Windows.Media;
using PA.src;
using System.Threading;
using Microsoft.Win32;
using System.Windows.Input;
using System.Globalization;
using System.IO;
using System.Data;
using static FParsec.ErrorMessage;
using MathNet.Numerics;
using System.IO.Pipes;
using ScottPlot.Plottables;

namespace Robot_window.MenuEvents
{
    internal class MenuEvents
    {
        private HelixViewport3D viewPort3d;

        public SerialPortUtils serialPortUtils = new SerialPortUtils();

        //全部路径点
        public PointsVisual3D PathPoints = new PointsVisual3D()
        {
            Color = Colors.Red,
            Size = 3
        };
        //newik全部路径点
        public PointsVisual3D NewIKPathPoints = new PointsVisual3D()
        {
            Color = Colors.Black,
            Size = 3
        };
        //每次移动时的路径
        public PointsVisual3D PathMovePoints = new PointsVisual3D()
        {
            Color = Colors.Green,
            Size = 3
        };
        //生成轨迹的球
        public PointsVisual3D Points = new PointsVisual3D
        {
            Color = Colors.Red,
            Size = 6
        };
        public ModelVisual3D toolmodelvisual = new ModelVisual3D();//工具
        public GeometryModel3D toolmodel3d = null;
        public TranslateTransform3D tooltd;
        public MeshGeometry3D shijian;

        //生成笛卡尔轨迹列表
        internal List<CartesianPosition> CartesianPositions = new List<CartesianPosition>();
        public int CpCount = 0;
        public int CpTotalCount = 0;
        // MoveL,MoveJ生成的路径点
        internal List<Position> MovePositions = new List<Position>();
        internal int MpCount = 0;
        public int ListBoxLidex = 0;
        //坐标系切换
        internal Frame frame1 = new Frame() { Content = new JointMovePage() };
        internal Frame frame2 = new Frame() { Content = new CartesianMovePage() };
        internal Frame frame3 = new Frame() { Content = new FaLanMovepage() };
        //屏幕点击的球
        public List<ModelVisual3D> modelVisual3Ds = new List<ModelVisual3D>();

        public Vector3D planeNormal = new Vector3D();
        public int order = 3;//曲线拟合多项式阶数
        public double[] coefficients;

        public MenuEvents()
        {
            viewPort3d = MainWindow.mainwindow.viewPort3d;

            //MainWindow.mainwindow.CartesianPositionList.ItemsSource = CartesianPositions;
        }
        public void JointSpace(object sender, RoutedEventArgs e)
        {
            frame1 = new Frame() { Content = new JointMovePage() };
            MainWindow.mainwindow.contentcon.Content = frame1;
        }

        public void CartesianSpace(object sender, RoutedEventArgs e)
        {
            frame2 = new Frame() { Content = new CartesianMovePage() };
            MainWindow.mainwindow.contentcon.Content = frame2;
        }

        public void FaLanSpace(object sender, RoutedEventArgs e)
        {
            var falan = new FaLanMovepage();
            falan.Inint();
            frame3 = new Frame() { Content = falan };
            MainWindow.mainwindow.contentcon.Content = frame3;

        }

        public void PathPointsProduce()
        {
            var Points = Position.MoveL(MainWindow.mainwindow.RightNowPosition, CartesianPositions[0].mmTom(), 0.5, MainWindow.mainwindow.GripToTool);
            for (int i = 1; i < CartesianPositions.Count; i++)
            {
                var list = Position.MoveL(Points[Points.Count - 1], CartesianPositions[i].mmTom(), 0.5, MainWindow.mainwindow.GripToTool);
                foreach (var p in list)
                {
                    Points.Add(p);
                }
            }
            foreach (var p in Points)
            {
                PathPoints.Points.Add(new Point3D(p.Pose.X * 1000, p.Pose.Y * 1000, p.Pose.Z * 1000));
            }
        }

        //方形曲线
        public void GridPath_Click(object sender, RoutedEventArgs e)
        {
            List<CartesianPosition> gridpathpoints = new List<CartesianPosition>();
            GridPath gridPath = new GridPath();
            gridPath.ShowDialog();
            gridpathpoints = TrajectoryPlanning.GridPath(gridPath.StartPoint, gridPath.EndPoint, gridPath.DirVector, gridPath.width, gridPath.lenght, gridPath.re);
            if (Points != null)
            {
                viewPort3d.Children.Remove(Points);
                Points.Points.Clear();
            }
            foreach (CartesianPosition position in gridpathpoints)
            {
                Points.Points.Add(new Point3D(position.X, position.Y, position.Z));
            }
            viewPort3d.Children.Add(Points);
            CartesianPositions = gridpathpoints;
            MainWindow.mainwindow.CartesianPositionList.ItemsSource = CartesianPositions;
            PathPointsProduce();
            viewPort3d.Children.Add(PathPoints);


        }

        //圆柱体曲线
        public void RotunditPathWindow_Click(object sender, RoutedEventArgs e)
        {
            List<CartesianPosition> rotunditpathpoints = new List<CartesianPosition>();
            RotundityPathWindow rotundityPathWindow = new RotundityPathWindow();
            rotundityPathWindow.ShowDialog();

            rotunditpathpoints = TrajectoryPlanning.RotundityPath(rotundityPathWindow.COCircle, rotundityPathWindow.NormalVector, rotundityPathWindow.radius,
               rotundityPathWindow.length, rotundityPathWindow.length_num, rotundityPathWindow.round_num, rotundityPathWindow.posture, rotundityPathWindow.reverse);
            if (Points != null)
            {
                viewPort3d.Children.Remove(Points);
                Points.Points.Clear();
            }
            foreach (CartesianPosition position in rotunditpathpoints)
            {
                Points.Points.Add(new Point3D(position.X, position.Y, position.Z));
            }
            viewPort3d.Children.Add(Points);
            CartesianPositions = rotunditpathpoints;
            MainWindow.mainwindow.CartesianPositionList.ItemsSource = CartesianPositions;
            PathPointsProduce();
            viewPort3d.Children.Add(PathPoints);
        }

        //球体曲线
        public void SphericalPathWindow_Click(object sender, RoutedEventArgs e)
        {
            List<CartesianPosition> sphericapathpoints = new List<CartesianPosition>();
            SphericalPathWindow sphericalPathWindow = new SphericalPathWindow();
            sphericalPathWindow.ShowDialog();
            sphericapathpoints = TrajectoryPlanning.SphericalPath(sphericalPathWindow.COSphere, sphericalPathWindow.dirVector, sphericalPathWindow.radius,
            sphericalPathWindow.startrad, sphericalPathWindow.endrad, sphericalPathWindow.rad_num, sphericalPathWindow.round_num, 0, sphericalPathWindow.reverse);
            if (Points != null)
            {
                viewPort3d.Children.Remove(Points);
                Points.Points.Clear();
            }
            foreach (CartesianPosition position in sphericapathpoints)
            {
                Points.Points.Add(new Point3D(position.X, position.Y, position.Z));
            }
            viewPort3d.Children.Add(Points);
            CartesianPositions = sphericapathpoints;
            MainWindow.mainwindow.CartesianPositionList.ItemsSource = CartesianPositions;
            PathPointsProduce();
            viewPort3d.Children.Add(PathPoints);
        }

        //清空轨迹事件响应函数
        public void PathClick(object sender, RoutedEventArgs e)
        {
            MainWindow.mainwindow.CartesianPositionList.SelectionChanged -= MainWindow.mainwindow.CartesianPositionList_SelectionChanged;
            if (Points != null)
            {
                MainWindow.mainwindow.CartesianPositionList.ItemsSource = null;
                viewPort3d.Children.Remove(Points);
                Points.Points.Clear();
                CpCount = 0;
                MpCount = 0;
                CartesianPositions.Clear();
                viewPort3d.Children.Remove(PathMovePoints);
                MovePositions.Clear();
                viewPort3d.Children.Remove(PathPoints);
                PathPoints.Points.Clear();
                PathPoints.Children.Clear();
                viewPort3d.Children.Remove(NewIKPathPoints);
                NewIKPathPoints.Points.Clear();
                NewIKPathPoints.Children.Clear();
                MainWindow.mainwindow.CartesianPositionList.ItemsSource = CartesianPositions;
                MainWindow.mainwindow.CartesianPositionList.SelectionChanged += MainWindow.mainwindow.CartesianPositionList_SelectionChanged;
            }
        }
        //复位机械臂
        public void ResetRobot(object sender, RoutedEventArgs e)
        {
            double[] angles = { 0, 0, 0, 0, 90, 0 };
            MainWindow.mainwindow.ForwardMove(angles);
            if (MainWindow.mainwindow.simulation == false)
                Control2183.Control2183.control.Command_2183(angles, MainWindow.mainwindow.Control2183Speed);
            if (MainWindow.mainwindow.contentcon.Content == frame1)
                JointMovePage.jointmovepage.Inint();
            else
                CartesianMovePage.cartesianmovpage.Inint();
            ListBoxLidex = 0;
        }

        public void Speedset(object sender, RoutedEventArgs e)
        {
            SpeedSetWindow speedSetWindow = new SpeedSetWindow();
            speedSetWindow.ShowDialog();

        }

        public void SerialPort(object sender, RoutedEventArgs e)
        {
            List<int> deviceAddresses = new List<int> { 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6 };

            serialPortUtils.LoadAddress(deviceAddresses);
            double[] angles = new double[6];
            Console.WriteLine("------------------");
            for (int i = 1; i < 7; i++)
            {

                //bool single = true;
                //serialPortUtils.StartPolling(single);
                //Thread.Sleep(200);
                //bool notsingle = false;
                //serialPortUtils.StartPolling(notsingle);
                //Thread.Sleep(200);

                bool single = true;
                double angle = 0;
                int numcircle = 0;
                serialPortUtils.StartPolling(single);
                Thread.Sleep(300);
                angle = serialPortUtils.Angle;
                serialPortUtils.Angle = 0;
                Console.WriteLine("angle：" + angle);
                Console.WriteLine("\n");
                single = false;
                serialPortUtils.StartPolling(single);
                Thread.Sleep(300);
                numcircle = serialPortUtils.NumCircle;
                //serialPortUtils.NumCircle = 0;
                Console.WriteLine("numcircle：" + numcircle);
                Console.WriteLine("\n");
                //if (MainWindow.mainwindow.simulation == false)
                angles[i - 1] = Control2183.Control2183.control.InitAngle(numcircle, angle, i);

            }
            //double[] angles1 = new double[6];//{ angles[1] , angles[0] , angles[2] , angles[3] , angles[4] , angles[5] };

            // 0 -0.0102 -3.8846 -0.423 -4.832 0

            //            0
            //- 0.0022
            //- 0.0446
            //0.4023
            //- 0.157
            //0


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

            if (MainWindow.mainwindow.contentcon.Content == frame1)
                JointMovePage.jointmovepage.Inint();
            else
                CartesianMovePage.cartesianmovpage.Inint();

            //  bool single = false;
            //serialPortUtils.StartPolling(single);

            // Thread.Sleep(100);
            //  bool notsingle = true;
            //serialPortUtils.StartPolling(notsingle);
        }

        public void Control2183_init(object sender, RoutedEventArgs e)
        {
            Control2183.Control2183.control.Connect2183();
        }





        public void LinePathWindow_Click(object sender, RoutedEventArgs e)
        {
            LinePathWindow linePathWindow = new LinePathWindow();
            linePathWindow.ShowDialog();
            List<CartesianPosition> positions = new List<CartesianPosition>();
            CartesianPosition start = new CartesianPosition(linePathWindow.start_x, linePathWindow.start_y, linePathWindow.start_z,
                linePathWindow.A_start, linePathWindow.B_start, linePathWindow.C_start);
            CartesianPosition end = new CartesianPosition(linePathWindow.end_x, linePathWindow.end_y, linePathWindow.end_z,
                linePathWindow.A_end, linePathWindow.B_end, linePathWindow.C_end); ;
            positions = TrajectoryPlanning.LinePath(start, end, linePathWindow.num);
            if (Points != null)
            {
                viewPort3d.Children.Remove(Points);
                Points.Points.Clear();
            }
            foreach (CartesianPosition position in positions)
            {
                Points.Points.Add(new Point3D(position.X, position.Y, position.Z));
            }
            viewPort3d.Children.Add(Points);
            CartesianPositions = positions;
            MainWindow.mainwindow.CartesianPositionList.ItemsSource = CartesianPositions;
            //179.76 -3.07 90.01
            PathPointsProduce();
            viewPort3d.Children.Add(PathPoints);

        }

        public void toolset(object sender, RoutedEventArgs e)
        {
            OpenFileDialog openFileDialog = new OpenFileDialog();
            openFileDialog.InitialDirectory = "c:\\desktop";    //初始的文件夹
            openFileDialog.Filter = "OBJ Files (*.obj)|*.obj|STL Files (*.stl)|*.stl";//在对话框中显示的文件类型
            openFileDialog.FilterIndex = 2;
            openFileDialog.RestoreDirectory = true;
            openFileDialog.ShowDialog();    //显示对话框
            string filepath = openFileDialog.FileName; //获取选择的文件的全路径名


            //加载模型
            var importer = new ModelImporter();
            var scene = importer.Load(filepath);
            toolmodel3d = scene.Children[0] as GeometryModel3D;
            toolmodel3d.Material = Materials.White;
            toolmodelvisual = new ModelVisual3D { Content = toolmodel3d };

            Transform3DGroup T = new Transform3DGroup();


            double[] angles = new double[6];
            double[] JointRad = BasicAlgorithm.AngleToRad(angles);
            JointPosition jointposition = new JointPosition(JointRad);
            PathClass path = new PathClass(jointposition);
            path.FK(MainWindow.mainwindow.GripToTool);
            double[] frame = new double[3] { path.Points.X * 1000, path.Points.Y * 1000, path.Points.Z * 1000 };
            tooltd = new TranslateTransform3D(frame[0], frame[1], frame[2]);

            //T.Children.Add(R);
            //T.Children.Add(R2);
            //T.Children.Add(td);
            toolmodel3d.Transform = tooltd;
            viewPort3d.Children.Add(toolmodelvisual);
            double[] angless = MainWindow.mainwindow.RightNowPosition.Joints.Joints.ToArray();
            int j = 0;
            foreach (var i in angless)
            {
                angless[j] = i * 180 / Math.PI;
                j++;
            }
            MainWindow.mainwindow.ForwardMove(angless);

        }
        public static double[,] GetRotationMatrix(Vector3D normal)
        {
            // 规范化法向量
            normal.Normalize();

            // 选择一个辅助向量 a
            Vector3D a = Math.Abs(normal.X) < 1e-6 && Math.Abs(normal.Y) < 1e-6
                ? new Vector3D(0, 1, 0) // 如果 normal 接近 z 轴，用 y 轴作为辅助向量
                : new Vector3D(0, 0, 1); // 否则用 z 轴

            // 计算 u = a × n
            Vector3D u = Vector3D.CrossProduct(a, normal);
            u.Normalize();

            // 计算 v = n × u
            Vector3D v = Vector3D.CrossProduct(normal, u);
            v.Normalize();
            // 构造旋转矩阵

            double[,] rotationMatrix = new double[3, 3]{
                { u.X, v.X, normal.X},
                { u.Y, v.Y, normal.Y},
                { u.Z, v.Z, normal.Z},
            };

            return rotationMatrix;
        }

        public void world3dclick(object sender, MouseButtonEventArgs e)
        {
            var mousePos = e.GetPosition(viewPort3d);
            var point3D = viewPort3d.Viewport.UnProject(mousePos);
            var his = viewPort3d.Viewport.FindHits(mousePos);

            if (his.Count > 0)
            {
                MainWindow.mainwindow.CartesianPositionList.ItemsSource = null;
                var hit = his.First();
                var p = hit.Position;
                List<Point3D> points_index = new List<Point3D>();
                CartesianPosition carposition = new CartesianPosition();
                if (shijian != null)
                {
                    points_index = FaceLocation(shijian, p);
                    var t = FaVector(points_index);
                    var rot = BasicAlgorithm.RotMatrixToRxyz(t);
                    carposition = new CartesianPosition(p.X, p.Y, p.Z, rot[0], rot[1], rot[2]);
                }
                else
                {
                    var rot = GetRotationMatrix(planeNormal);
                    var ro = BasicAlgorithm.RotMatrixToRxyz(rot);
                    carposition = new CartesianPosition(p.X, p.Y, p.Z, ro[0], ro[1], ro[2]);
                }

                CartesianPositions.Add(carposition);
                var builder = new MeshBuilder(true, true);

                builder.AddSphere(p, 1);
                var g = new GeometryModel3D(builder.ToMesh(), Materials.Blue);
                ModelVisual3D v = new ModelVisual3D();
                v.Content = g;
                modelVisual3Ds.Add(v);
                viewPort3d.Children.Add(v);

                MainWindow.mainwindow.CartesianPositionList.ItemsSource = CartesianPositions;
            }
            else if (point3D.HasValue)
            {
                MainWindow.mainwindow.CartesianPositionList.ItemsSource = null;
                // 获取的三维坐标
                var position2 = point3D.Value;
                if (position2.Z < 0)
                    position2.Z = 0;

                CartesianPosition carposition = new CartesianPosition(position2.X, position2.Y, position2.Z, Math.PI, 0, 0);
                //if (MainWindow.mainwindow.X_Point.Text !="")
                //    carposition.X = x;
                //if (MainWindow.mainwindow.Y_Point.Text != "")
                //    carposition.Y = y;
                //if (MainWindow.mainwindow.Z_Point.Text != "")
                //    carposition.Z = z;

                CartesianPositions.Add(carposition);
                var builder = new MeshBuilder(true, true);
                var position1 = new Point3D(carposition.X, carposition.Y, carposition.Z);
                builder.AddSphere(position1, 10);
                var g = new GeometryModel3D(builder.ToMesh(), Materials.Blue);
                ModelVisual3D v = new ModelVisual3D();
                v.Content = g;
                modelVisual3Ds.Add(v);
                viewPort3d.Children.Add(v);
                MainWindow.mainwindow.CartesianPositionList.ItemsSource = CartesianPositions;

            }
            if (PathPoints.Points.Count != 0)
            {
                PathPoints.Points.Clear();
                PathPoints.Children.Clear();
                viewPort3d.Children.Remove(PathPoints);
            }
            PathPointsProduce();
            viewPort3d.Children.Add(PathPoints);
        }

        internal void clearworld(object sender, RoutedEventArgs e)
        {
            foreach (var item in modelVisual3Ds.ToArray()) // 使用ToArray()来避免在迭代过程中修改集合  
            {
                viewPort3d.Children.Remove(item);
            }
            ListBoxLidex = 0;


        }

        public void SpecimensProduce(object sender, RoutedEventArgs e)
        {
            RectAngleWindow rectAngleWindow = new RectAngleWindow();
            rectAngleWindow.ShowDialog();
            MeshBuilder builder = new MeshBuilder(true, true);
            builder.AddBox(rectAngleWindow.Point, rectAngleWindow.xlenght, rectAngleWindow.ylenght, rectAngleWindow.zlenght);
            GeometryModel3D model = new GeometryModel3D(builder.ToMesh(), Materials.White);

            shijian = builder.ToMesh();
            ModelVisual3D modelvisual = new ModelVisual3D();
            modelvisual.Content = model;
            viewPort3d.Children.Add(modelvisual);
        }

        private List<Point3D> FaceLocation(MeshGeometry3D c, Point3D point)
        {

            double sumX = 0, sumY = 0, sumZ = 0;
            int vertexCount = c.Positions.Count;

            foreach (Point3D vertex in c.Positions)
            {
                sumX += vertex.X;
                sumY += vertex.Y;
                sumZ += vertex.Z;
            }

            double centerX = sumX / vertexCount;
            double centerY = sumY / vertexCount;
            double centerZ = sumZ / vertexCount;

            Point3D center = new Point3D(centerX, centerY, centerZ);

            double width = c.Bounds.SizeX;
            double height = c.Bounds.SizeY;
            double depth = c.Bounds.SizeZ;
            double tolerance = 0.1;
            int[] indexs = new int[3];
            List<Point3D> result = new List<Point3D>();
            if (Math.Abs((point.X - center.X) - width / 2) < tolerance)
            {
                for (int i = 0; i < 3; i++)
                {
                    indexs[i] = c.TriangleIndices[i];
                }
            }
            else if (Math.Abs((point.X - center.X) + width / 2) < tolerance)
            {
                for (int i = 0; i < 3; i++)
                {
                    indexs[i] = c.TriangleIndices[6 + i];
                }
            }
            else if (Math.Abs((point.Y - center.Y) - height / 2) < tolerance)
            {
                for (int i = 0; i < 3; i++)
                {
                    indexs[i] = c.TriangleIndices[6 * 2 + i];
                }
            }
            else if (Math.Abs((point.Y - center.Y) + height / 2) < tolerance)
            {
                for (int i = 0; i < 3; i++)
                {
                    indexs[i] = c.TriangleIndices[6 * 3 + i];
                }
            }
            else if (Math.Abs((point.Z - center.Z) - depth / 2) < tolerance)
            {
                for (int i = 0; i < 3; i++)
                {
                    indexs[i] = c.TriangleIndices[6 * 4 + i];
                }

            }
            else if (Math.Abs((point.Z - center.Z) + depth / 2) < tolerance)
            {
                for (int i = 0; i < 3; i++)
                {
                    indexs[i] = c.TriangleIndices[6 * 5 + i];
                }
            }
            else
            {
                Console.WriteLine("该点不在任何面上");
            }
            result.Add(c.Positions[indexs[0]]);
            result.Add(c.Positions[indexs[1]]);
            result.Add(c.Positions[indexs[2]]);


            return result;

        }
        private double[,] FaVector(List<Point3D> p)
        {
            Vector3D[] result = new Vector3D[p.Count];
            Vector3D v1 = p[1] - p[0];
            Vector3D v2 = p[2] - p[0];
            Vector3D v3 = p[2] - p[1];
            Vector3D normal = Vector3D.CrossProduct(v1, v2);
            normal.Normalize(); // 可选：单位化
            v1.Normalize();
            v3.Normalize();
            result[0] = v3;
            result[1] = -v1;
            result[2] = -normal;
            double[,] R = new double[3, 3];
            if (result[2].X == 1 || result[2].X == -1)
                result[0] = -result[0];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                {

                    if (j == 0)
                        R[j, i] = result[i].X;
                    if (j == 1)
                        R[j, i] = result[i].Y;
                    if (j == 2)
                        R[j, i] = result[i].Z;

                }
            return R;
        }


        internal void DeskProduce(object sender, RoutedEventArgs e)
        {
            MeshBuilder builder = new MeshBuilder(true, true);
            Point3D center = new Point3D(600 - 280, 0, 0);
            double xlenght = 1200;
            double ylenght = 800;
            double zlenght = 50;
            builder.AddBox(center, xlenght, ylenght, zlenght);
            GeometryModel3D model = new GeometryModel3D(builder.ToMesh(), Materials.White);
            shijian = builder.ToMesh();
            ModelVisual3D modelvisual = new ModelVisual3D();
            modelvisual.Content = model;
            viewPort3d.Children.Add(modelvisual);
        }

        public void PointToPath_Click(object sender, RoutedEventArgs e)
        {
            List<CartesianPosition> gridpathpoints = new List<CartesianPosition>();
            PointToPathWindow pointToPathWindow = new PointToPathWindow();
            pointToPathWindow.ShowDialog();
            string filePath = "D:\\code\\华数机械臂\\RobotApp - 华数测试demo (3)\\RobotApp - 华数测试demo\\test.txt"; // 替换为您的文件路径
            var lines = File.ReadAllLines(filePath);

            List<CartesianPosition> positions = new List<CartesianPosition>();

            int pointnum = lines.Length / 4;

            double[,] R = new double[3, 3];

            double[] Rxyz = new double[3];

            double[] T = new double[3];

            for (int i = 0; i < lines.Length - 32; i += 32) // 每个矩阵占用 4 行
            {
                //public static double[] RotMatrixToRxyz(double[,] R)
                for (int j = 0; j < 3; j++)
                {
                    var values = lines[i + j].Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries)
                                    .Select(v => double.Parse(v, CultureInfo.InvariantCulture))
                                    .ToArray();

                    R[j, 0] = values[0];
                    R[j, 1] = values[1];
                    R[j, 2] = values[2];

                    T[j] = values[3];
                }
                Rxyz = BasicAlgorithm.RotMatrixToRxyz(R);
                T[0] = T[0] - 100;
                CartesianPosition temp = new CartesianPosition(T, Rxyz);

                positions.Add(temp);
            }

            CartesianPositions = positions;

            MainWindow.mainwindow.CartesianPositionList.ItemsSource = CartesianPositions;


            PathPointsProduce();
            viewPort3d.Children.Add(PathPoints);





        }

        public void clearrobot(object sender, RoutedEventArgs e)
        {
            MainWindow.mainwindow.viewPort3d.Children.Remove(MainWindow.mainwindow.visual);
            MainWindow.mainwindow.visual.Content = null;
            MainWindow.mainwindow.ListJoints.Clear();
        }

        public void LoadRobot(object sender, RoutedEventArgs e)
        {
            OpenFileDialog dialog = new OpenFileDialog();
            dialog.DefaultExt = ".folder"; // 设置默认扩展名
            dialog.Filter = "STL|*.stl"; // 设置文件类型过滤器
            dialog.Multiselect = false; // 只允许选择一个
            dialog.ShowDialog();
            //加载机械臂模型
            string path = "";
            if (dialog.FileName != "")
            {
                string filePath = dialog.FileName;
                path = GetDirectoryPath(filePath);
            }
            RobotParams robotParams = new RobotParams();
            RobotLoad load = new RobotLoad();
            robotParams.LinkBasePath = path;
            MainWindow.mainwindow.RobotModel = load.StlLoad(robotParams.LinkBasePath);
            MainWindow.mainwindow.visual.Content = MainWindow.mainwindow.RobotModel;
            MainWindow.mainwindow.viewPort3d.Children.Add(MainWindow.mainwindow.visual);
            //读取机器人参数
            string[] files = Directory.GetFiles(path);
            string robotJointsParams = "";
            foreach (string file in files)
            {
                if (file.Contains(".txt"))
                    robotJointsParams = file;
            }
            robotParams.RobotParamsRead(robotJointsParams);
            MainWindow.mainwindow.ListJoints = Joint.LoadParams(robotParams.robotparams);

            double[] angles = { 0, 0, 0, 0, 90, 0 };

            MainWindow.mainwindow.ForwardMove(angles);
        }
        private static string GetDirectoryPath(string filePath)
        {
            // 找到最后一个路径分隔符的位置
            int lastSeparatorIndex = filePath.LastIndexOf(Path.DirectorySeparatorChar);

            // 如果找到分隔符，截取到该位置的字符串
            if (lastSeparatorIndex > 0)
            {
                return filePath.Substring(0, lastSeparatorIndex);
            }
            else
            {
                // 如果没有找到分隔符，说明路径本身就是一个文件夹路径
                return filePath;
            }
        }

        public void SearchCom(object sender, RoutedEventArgs e)
        {
            string[] ports = System.IO.Ports.SerialPort.GetPortNames();
        }

        public void KeyDownSpace(object sender, KeyEventArgs e)
        {
            viewPort3d.Children.Remove(MainWindow.mainwindow.rectangleVisual3D);

            viewPort3d.Children.Remove(MainWindow.mainwindow.sphere);
            viewPort3d.Children.Remove(MainWindow.mainwindow.coordinateSystem);

        }

        public void ChartWindow_Click(object sender, RoutedEventArgs e)
        {
            List<double> list_t = new List<double>();
            double k = 0;
            for (int i = 0; i < MovePositions.Count; i++)
            {
                list_t.Add(k);
                k += 0.05;
            }
            double[] t = list_t.ToArray();
            double[] xFit = new double[MovePositions.Count];
            double[] xTrue = new double[MovePositions.Count];
            for (int i = 0; i < MovePositions.Count; i++)
            {
                xFit[i] = MovePositions[i].Pose.X * 1000;
                xTrue[i] = PathMovePoints.Points[i].X;
            }

            double[] yFit = new double[MovePositions.Count];
            double[] yTrue = new double[MovePositions.Count];
            for (int i = 0; i < MovePositions.Count; i++)
            {
                yFit[i] = MovePositions[i].Pose.Y * 1000;
                yTrue[i] = PathMovePoints.Points[i].Y;
            }

            double[] zFit = new double[MovePositions.Count];
            double[] zTrue = new double[MovePositions.Count];
            for (int i = 0; i < MovePositions.Count; i++)
            {
                //dsadsad
                zFit[i] = MovePositions[i].Pose.Z * 1000;
                zTrue[i] = PathMovePoints.Points[i].Z;
                //dsadasdsadsds
            }

            Error_Mse(xTrue, xFit, "X");
            Error_Mse(yTrue, yFit, "Y");
            Error_Mse(zTrue, zFit, "Z");

            Chart chart = new Chart();
            chart.Show();
            chart.pltshow(t, xFit);
            chart.SetlegendText("XFit");
            chart.pltshow(t, xTrue);
            chart.SetlegendText("XTrue");

            chart.pltshow(t, yFit);
            chart.SetlegendText("YFit");
            chart.pltshow(t, yTrue);
            chart.SetlegendText("YTrue");

            chart.pltshow(t, zFit);
            chart.SetlegendText("ZFit");
            chart.pltshow(t, zTrue);
            chart.SetlegendText("ZTrue");

        }
        private void Error_Mse(double[] xreal, double[] xfit, string name)
        {
            double mse = 0;
            double maxerror = 0;
            for (int i = 0; i < xreal.Length; i++)
            {
                maxerror = Math.Max(maxerror, Math.Abs(xreal[i] - xfit[i]));
                mse += (xreal[i] - xfit[i]) * (xreal[i] - xfit[i]);
            }
            mse = mse / xreal.Length;
            Console.WriteLine("MSE" + name + ":" + mse.ToString());
            Console.WriteLine("MaxError" + name + ":" + maxerror.ToString() + "mm");

        }
        /// <summary>
        /// 三次样条插值，先给每个空间的点赋予时间，然后进行三次样条插值
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        public void ThreePath_Click(object sender, RoutedEventArgs e)
        {
            //每个点赋予时间
            for (int i = 0; i < CartesianPositions.Count; i++)
            {
                CartesianPositions[i].t = i;
            }
            //三次样条插值
            coefficients = TrajectoryPlanning.CurveFitting(CartesianPositions, order: 3);
            List<CartesianPosition> positions = new List<CartesianPosition>();
            for (double i = 0; i <= CartesianPositions.Count - 1;)
            {
                double x = coefficients[0] + coefficients[1] * i + coefficients[2] * i * i + coefficients[3] * i * i * i;
                double y = coefficients[4] + coefficients[5] * i + coefficients[6] * i * i + coefficients[7] * i * i * i;
                double z = coefficients[8] + coefficients[9] * i + coefficients[10] * i * i + coefficients[11] * i * i * i;
                double A = Math.PI;
                double B = 0;
                double C = 0;
                CartesianPosition temp = new CartesianPosition(x, y, z, A, B, C);
                temp.t = i;
                positions.Add(temp);
                i += 0.05;
            }
            if (MainWindow.mainwindow.NewIKFlag)
            {
                CartesianPositions = positions;
                MainWindow.mainwindow.CartesianPositionList.ItemsSource = CartesianPositions;
            }
            //显示路径点
            if (NewIKPathPoints != null)
            {
                viewPort3d.Children.Remove(NewIKPathPoints);
                NewIKPathPoints.Points.Clear();
            }
            foreach (CartesianPosition position in CartesianPositions)
            {
                NewIKPathPoints.Points.Add(new Point3D(position.X, position.Y, position.Z));
            }
            viewPort3d.Children.Add(NewIKPathPoints);

            //求出x,y,z的表达式
            NewIK.FunctionExpressionx = SymbolDerivation.FuncStringProduce(coefficients, 0, order);
            NewIK.FunctionExpressiony = SymbolDerivation.FuncStringProduce(coefficients, 4, order);
            NewIK.FunctionExpressionz = SymbolDerivation.FuncStringProduce(coefficients, 8, order);
            Console.WriteLine("x表达式：" + NewIK.FunctionExpressionx);
            Console.WriteLine("y表达式：" + NewIK.FunctionExpressiony);
            Console.WriteLine("z表达式：" + NewIK.FunctionExpressionz);

            double[,] Q3 = NewIK.Q3Product(CartesianPositions);
            double[,] Q = NewIK.QProduct(CartesianPositions);
        }

        public void Q6Draw_Click(object sender, RoutedEventArgs e)
        {
            double[,] Q3 = NewIK.Q3Product(CartesianPositions);
            Chart.Q3Draw(Q3, CartesianPositions);
            double[,] Q = NewIK.QProduct(CartesianPositions);
            List<double> t = new List<double>();
            t.Add(0.15);
            t.Add(0.25);
            t.Add(0.35);
            t.Add(0.45);
            int[] indexs = NewIK.TChanglistToIndex(t, CartesianPositions);
            Chart.QDraw(Q, indexs, CartesianPositions);
        }

        public void A6B6Draw_Click(object sender, RoutedEventArgs e)
        {
            double[,] A6B6 = NewIK.A6B6Product(CartesianPositions);
            Chart.A6B6Draw(A6B6, CartesianPositions);
        }

        internal void Grep(object sender, RoutedEventArgs e)
        {
            Thread thread = new Thread(WorkMethod);

            // 启动线程
            thread.Start();

            // 主线程继续执行
            Console.WriteLine("主线程继续执行...");

        }

        private void WorkMethod(object? obj)
        {
            while (true)
            {
                using (var pipeClient = new NamedPipeClientStream(".", "MyPipe", PipeDirection.In))
                {
                    pipeClient.Connect();

                    int[] array = null;

                    using (var reader = new BinaryReader(pipeClient))
                    {
                        try
                        {
                            // 读取数组大小
                            int arraySize = reader.ReadInt32();

                            // 读取数组数据
                            array = new int[arraySize];

                            for (int i = 0; i < arraySize; i++)
                            {
                                array[i] = reader.ReadInt32();
                            }
                        }
                        catch (EndOfStreamException)
                        {
                            // 管道已关闭，退出循环
                            //break;
                        }
                        
                        if (array != null)
                        {
                            MainWindow.mainwindow.RightNowPosition.ChaoSheng = array;
                            // 输出接收到的数组
                            Console.WriteLine("xyz:" + MainWindow.mainwindow.RightNowPosition.Pose.X.ToString() + " " + MainWindow.mainwindow.RightNowPosition.Pose.Y.ToString() + " " + MainWindow.mainwindow.RightNowPosition.Pose.Z.ToString());
                            Console.WriteLine("Received array:");
                            foreach (var item in MainWindow.mainwindow.RightNowPosition.ChaoSheng)
                            {
                                Console.Write(item + " ");
                            }
                            Console.Write("\n");
                        }

                    }
                }
            }
        }

        internal void Grep2(object sender, RoutedEventArgs e)
        {
            // 输出接收到的数组
            Console.WriteLine("xyz:" + MainWindow.mainwindow.RightNowPosition.Pose.X.ToString() + " " + MainWindow.mainwindow.RightNowPosition.Pose.Y.ToString() + " " + MainWindow.mainwindow.RightNowPosition.Pose.Z.ToString());
            Console.WriteLine("Received array:");
            foreach (var item in MainWindow.mainwindow.RightNowPosition.ChaoSheng)
            {
                Console.Write(item + " ");
            }
            Console.Write("\n");
        }
    }
}





