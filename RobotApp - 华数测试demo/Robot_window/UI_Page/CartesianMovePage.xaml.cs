using RobotLibrary;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
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

namespace Robot_window.UI_Page
{
    /// <summary>
    /// CartesianMovePage.xaml 的交互逻辑
    /// </summary>
    public partial class CartesianMovePage : Page
    {
        public static CartesianMovePage cartesianmovpage;
        public double oldrx, oldry, oldrz;
        public CartesianMovePage()
        {
            InitializeComponent();
            Inint();
            cartesianmovpage = this;

        }

        public void Inint()
        {
            RobotParams robotParams = new RobotParams();
            
            X.Mininum = robotParams.robotlimparams[2,0];
            Y.Mininum = robotParams.robotlimparams[2, 1];
            Z.Mininum = robotParams.robotlimparams[2, 2];
            A.Mininum = robotParams.robotlimparams[2, 3]-0.5;
            B.Mininum = robotParams.robotlimparams[2, 4]-0.5;
            C.Mininum = robotParams.robotlimparams[2, 5] - 0.5;
            X.Maxium = robotParams.robotlimparams[3,0];
            Y.Maxium = robotParams.robotlimparams[3,1];
            Z.Maxium = robotParams.robotlimparams[3,2];
            A.Maxium = robotParams.robotlimparams[3, 3]+0.5;
            B.Maxium = robotParams.robotlimparams[3, 4] + 0.5;
            C.Maxium = robotParams.robotlimparams[3, 5] + 0.5;

            X.JointValue = MainWindow.mainwindow.RightNowPosition.Pose.X * 1000;
            Y.JointValue = MainWindow.mainwindow.RightNowPosition.Pose.Y * 1000;
            Z.JointValue = MainWindow.mainwindow.RightNowPosition.Pose.Z * 1000;
            //A.JointValue = MainWindow.mainwindow.RightNowPosition.Pose.RX * 180 / Math.PI;
            //B.JointValue = MainWindow.mainwindow.RightNowPosition.Pose.RY * 180 / Math.PI;
            //C.JointValue = MainWindow.mainwindow.RightNowPosition.Pose.RZ * 180 / Math.PI;
            A.JointValue = 0;
            B.JointValue = 0; C.JointValue = 0;
            oldrx = A.JointValue; oldry=B.JointValue;oldrz = C.JointValue;
            int max = (int)MainWindow.mainwindow.speed.Maximum;
            int value = (int)MainWindow.mainwindow.speed.Value;
            X.IntervalTime=max-value+1;
            Y.IntervalTime=max-value+1;
            Z.IntervalTime=max-value+1;
            A.IntervalTime = max - value + 1;
            B.IntervalTime=max - value + 1;
            C.IntervalTime=max - value + 1;
            //string show = ((int)(   MainWindow.mainwindow.speed.Value / MainWindow.mainwindow.  speed.Maximum * 100)).ToString() + "%";
            if (MainWindow.mainwindow.speedshow != null)
                MainWindow.mainwindow.speedshow.Content ="Slow";
        }

        private void JointValueChanged(object sender, RoutedEventArgs e)
        {
            if (A.JointValue == -180.5)
                A.JointValue = 180;
            if (A.JointValue == 180.5)
                A.JointValue = -180;
            if (B.JointValue == -180.5)
                B.JointValue = 180;
            if (B.JointValue == 180.5)
                B.JointValue = -180;
            if (C.JointValue == -180.5)
                C.JointValue = 180;
            if (C.JointValue == 180.5)
                C.JointValue = -180;


            double[] eerot = new double[] { MainWindow.mainwindow.RightNowPosition.Pose.RX,MainWindow.mainwindow.RightNowPosition.Pose.RY,MainWindow.mainwindow.RightNowPosition.Pose.RZ };
            double[] deerot=new double[] {A.JointValue-oldrx,B.JointValue-oldry,C.JointValue-oldrz};
            double[] eetrans=new double[] { X.JointValue/1000,Y.JointValue/1000, Z.JointValue/1000};


           
            deerot =BasicAlgorithm.AngleToRad(deerot);
            double[,] R = BasicAlgorithm.RxyzToRotMatrix(eerot[0], eerot[1], eerot[2]);
            double[,] Rt = BasicAlgorithm.RxyzToRotMatrix(deerot[0], deerot[1], deerot[2]);
            var R2 = BasicAlgorithm.MatrixMultiply(Rt, R);
            var ro=BasicAlgorithm.RotMatrixToRxyz(R2);
           
           
            CartesianPosition cartesianPosition = new CartesianPosition(eetrans,ro);
            PathClass path = new PathClass(cartesianPosition);
           
            path.IK(MainWindow.mainwindow.RightNowPosition.Joints, MainWindow.mainwindow.GripToTool);
            if (path.Num == 0)
                MessageBox.Show("奇异点");
            else
            {
                double[] angles = BasicAlgorithm.RadToAngle(path.Joints);
                MainWindow.mainwindow.ForwardMove(angles);
                if (MainWindow.mainwindow.simulation == false)
                    Control2183.Control2183.control.Command_2183(angles, MainWindow.mainwindow.Control2183Speed);
            }
            oldrx = A.JointValue;
            oldry = B.JointValue;
            oldrz = C.JointValue;

        }

       
    }
}
