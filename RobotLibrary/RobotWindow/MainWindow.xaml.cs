using Device;
using HelixToolkit.Wpf;
using Model;
using RobotLibrary;
using RobotLibraryAlgorithm;
using Robots;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace RobotWindow
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public static MainWindow mainwindow;
        public RobotManager manager;
        public MainWindow()
        {
            InitializeComponent();
            manager = new RobotManager();
            //HuaShu AnChuan Roke
            RobotInit("HuaShu");
            InitView();

            //box
            //BoxModel box = new BoxModel();
            //box.BoxModelInit(new Point3D(400,0,500),new double[] { 100, 100, 100 });
            //viewPort3d.Children.Add(box.Boxmodelvisual);

            //frame
            

            mainwindow = this;
        }
        public void RobotInit(string robottype)
        {
            manager.RobotInit(robottype);
            viewPort3d.Children.Add(manager.robot.Joints.RobotModelVisual);
            //frame
            //var p = manager.algorithmManager.kinematicsAlgorithm.FkAngle(new double[] { 0, 0, 0, 0, 0, 0 });
            //double[] position = new double[3];
            //position[0] = p.Point.X * 1000;
            //position[1] = p.Point.Y * 1000;
            //position[2] = p.Point.Z * 1000;
            //FrameModel frame = new FrameModel();
            //frame.FrameModelInit(position);
            //viewPort3d.Children.Add(frame.FrameModelVisual);
        }
       public void InitView()
        {
            AnChuan.Click += qiehuanrotbot;
            HuaShu.Click += qiehuanrotbot;
            Roke.Click += qiehuanrotbot;
        }

        private void qiehuanrotbot(object sender, RoutedEventArgs e)
        {
            if (manager.robot!=null)
            {
                viewPort3d.Children.Remove(manager.robot.Joints.RobotModelVisual);
                manager.RobotClear();
            }
            MenuItem clickedItem = sender as MenuItem;
            if (clickedItem != null)
            {
                // 获取 Header 值
                string headerValue = clickedItem.Header.ToString();
                RobotInit(headerValue);
            }
        }
        private void sphere()
        {
            var builder = new MeshBuilder(true, true);
            builder.AddSphere(new Point3D(), 20, 15, 15);
            var g = new GeometryModel3D(builder.ToMesh(), Materials.Red);
            ModelVisual3D v = new ModelVisual3D();
            v.Content = g;
            viewPort3d.Children.Add(v);

            var sphere = new SphereVisual3D
            {
                Center = new Point3D(0, 0, 120),
                Radius = 10,
                Fill = Brushes.SkyBlue
            };
            viewPort3d.Children.Add(sphere);
        }

        private void textChanged(object sender, TextChangedEventArgs e)
        {
            if (manager != null)
            {
                var start = new Vector3(450, -200, 300);
                var end = new Vector3(450, 200, 300);
                var vec = new Vector3(0, 0, 1);
                float a = 50;
                float b = 300;
                bool c = true;
                manager.cartesianpositionlist.CartesianPositions = manager.algorithmManager.Trajectory.GridPath(start, end, vec, a, b, c);
                PointsModel p = new PointsModel();
                p = manager.PathPointsShow(manager.RightNowPosition, manager.cartesianpositionlist);
                p.PointsModelInit();
                viewPort3d.Children.Add(p.PointsVisual);
            }

        }
        private void LiFaceChange(object sender, TextChangedEventArgs e)
        {
            double[] angles= new double[6];
            if (J6 != null&& manager!=null)
            {
                angles[0] = double.Parse(LeftUp.Text);
                angles[1] = double.Parse(RightUp.Text);
                angles[2] = double.Parse(LeftBotm.Text);
                angles[3] = double.Parse(Rightbotm.Text);
                angles[4] = double.Parse(J5.Text);
                angles[5] = double.Parse(J6.Text);
                manager.robot.ForwardMove(angles);
                manager.FKShow(angles);
            }
            


        }
    }
}
