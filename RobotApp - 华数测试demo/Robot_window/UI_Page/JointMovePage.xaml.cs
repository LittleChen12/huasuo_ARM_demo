using Robot_window.MenuEvents;
using RobotLibrary;
using System;
using System.Collections.Generic;
using System.Linq;
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
    /// JointMovePage.xaml 的交互逻辑
    /// </summary>
    public partial class JointMovePage : Page
    {
        public static JointMovePage jointmovepage;
        
        public JointMovePage()
        {
            InitializeComponent();
            jointmovepage = this;
            Inint();
           
        }

        public void Inint()
        {
            RobotParams robotParams = new RobotParams();

            Joint1.Mininum = robotParams.robotlimparams[0, 0];
            Joint2.Mininum = robotParams.robotlimparams[0, 1];
            Joint3.Mininum = robotParams.robotlimparams[0, 2];
            Joint4.Mininum = robotParams.robotlimparams[0, 3];
            Joint5.Mininum = robotParams.robotlimparams[0, 4];
            Joint6.Mininum = robotParams.robotlimparams[0, 5];
            Joint1.Maxium = robotParams.robotlimparams[1, 0];
            Joint2.Maxium = robotParams.robotlimparams[1, 1];
            Joint3.Maxium = robotParams.robotlimparams[1, 2];
            Joint4.Maxium = robotParams.robotlimparams[1, 3];
            Joint5.Maxium = robotParams.robotlimparams[1, 4];
            Joint6.Maxium = robotParams.robotlimparams[1, 5];

            double[] angles = new double[6];
            for(int i=0;i<6;i++)
            {
                angles[i] = MainWindow.mainwindow.RightNowPosition.Joints.Joints[i];
            }
            angles=BasicAlgorithm.RadToAngle(angles);
            Joint1.JointValue = angles[0];
            Joint2.JointValue = angles[1];
            Joint3.JointValue = angles[2];
            Joint4.JointValue = angles[3];
            Joint5.JointValue = angles[4];
            Joint6.JointValue = angles[5];
            
            int max = (int)MainWindow.mainwindow.speed.Maximum;
            int value = (int )MainWindow.mainwindow.speed.Value; 
            Joint1.IntervalTime = max - value + 1;
            Joint2.IntervalTime = max - value + 1;
            Joint3.IntervalTime = max - value + 1;
            Joint4.IntervalTime = max - value + 1;
            Joint5.IntervalTime = max - value + 1;
            Joint6.IntervalTime = max - value + 1;
            //string show = ((int)(MainWindow.mainwindow.speed.Value /    MainWindow.mainwindow.speed.Maximum * 100)).ToString() + "%";
            if (MainWindow.mainwindow. speedshow != null)
                MainWindow.mainwindow.speedshow.Content = "Slow";
        }

        private void JointValueChanged(object sender, RoutedEventArgs e)
        {
            double[] angles = { Joint1.JointValue,Joint2.JointValue,Joint3.JointValue,
            Joint4.JointValue,Joint5.JointValue,Joint6.JointValue};
            MainWindow.mainwindow.ForwardMove(angles);
            if (MainWindow.mainwindow. simulation == false)
                Control2183.Control2183.control.Command_2183(angles, MainWindow.mainwindow.Control2183Speed);
        }
       
    }
}
