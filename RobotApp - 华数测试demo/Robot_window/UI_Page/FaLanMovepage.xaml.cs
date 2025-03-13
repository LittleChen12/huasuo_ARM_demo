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
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace Robot_window.UI_Page
{
    /// <summary>
    /// FaLanMovepage.xaml 的交互逻辑
    /// </summary>
    public partial class FaLanMovepage : Page
    {
        public static FaLanMovepage falanmovepage;
        private double oldx =0;
        private double oldy = 0;
        private double oldz = 0;
        private double oldrx = 0;
        private double oldry = 0;
        private double oldrz = 0;
        public FaLanMovepage()
        {
            InitializeComponent();
            falanmovepage = this;
        }

        public void Inint()
        {
            RobotParams robotParams = new RobotParams();

            X.Mininum = -1000;
            Y.Mininum = -1000;
            Z.Mininum = -1000;
            A.Mininum = robotParams.robotlimparams[2, 3] - 0.5;
            B.Mininum = robotParams.robotlimparams[2, 4] - 0.5;
            C.Mininum = robotParams.robotlimparams[2, 5] - 0.5;
            X.Maxium = 1000;
            Y.Maxium = 1000;
            Z.Maxium = 1000;
            A.Maxium = robotParams.robotlimparams[3, 3] + 0.5;
            B.Maxium = robotParams.robotlimparams[3, 4] + 0.5;
            C.Maxium = robotParams.robotlimparams[3, 5] + 0.5;

            X.JointValue = 0;
            Y.JointValue = 0;
            Z.JointValue = 0;
            A.JointValue = 0;
            B.JointValue = 0;
            C.JointValue = 0;
           

            int max = (int)MainWindow.mainwindow.speed.Maximum;
            int value = (int)MainWindow.mainwindow.speed.Value;
            X.IntervalTime = max - value + 1;
            Y.IntervalTime = max - value + 1;
            Z.IntervalTime = max - value + 1;
            A.IntervalTime = max - value + 1;
            B.IntervalTime = max - value + 1;
            C.IntervalTime = max - value + 1;
            //string show = ((int)(MainWindow.mainwindow.speed.Value / MainWindow.mainwindow.speed.Maximum * 100)).ToString() + "%";
            if (MainWindow.mainwindow.speedshow != null)
                MainWindow.mainwindow.speedshow.Content = "slow";
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
            var Tbasetogrip = MainWindow.mainwindow.RightNowPosition.Pose.ToTrans();
            Console.WriteLine("start:rx"+MainWindow.mainwindow.RightNowPosition.Pose.RX*180/Math.PI);
            CartesianPosition p = new CartesianPosition((X.JointValue - oldx) / 1000, (Y.JointValue - oldy) / 1000, (Z.JointValue - oldz) / 1000, (A.JointValue-oldrx ) / 180 * Math.PI, (B.JointValue - oldry) / 180 * Math.PI,(C.JointValue-oldrz)/180*Math.PI);
            var Tbc=p.ToTrans();
            Matrix4x4 Tac = Tbasetogrip * Tbc;
            CartesianPosition t  =   CartesianPosition.TransToPose(Tac);
            oldx=X.JointValue; oldy=Y.JointValue; oldz=Z.JointValue;
            oldrx = A.JointValue; oldry = B.JointValue;oldrz = C.JointValue;
            //Console.WriteLine("x:"+t.X*1000);
            //Console.WriteLine("y:"+t.Y*1000);
            //Console.WriteLine("z:"+t.Z*1000);
            //Console.WriteLine("rx:"+t.RX/Math.PI*180);
            //Console.WriteLine("ry:"+t.RY/Math.PI*180);
            //Console.WriteLine("rz:"+t.RZ / Math.PI * 180);
            PathClass path = new PathClass(t);
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

        }
    }
}
