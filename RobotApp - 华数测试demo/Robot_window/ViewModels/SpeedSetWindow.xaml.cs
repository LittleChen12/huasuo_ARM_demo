using Robot_window.UI_Page;
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
using System.Windows.Shapes;

namespace Robot_window.ViewModels
{
    /// <summary>
    /// SpeedSetWindow.xaml 的交互逻辑
    /// </summary>
    public partial class SpeedSetWindow : Window
    {
        public int JointSpeed;
        public int XYZSpeed;
        public int ABCSpeed;
        public int C2183Speed;
        public float JointInterlValue;
        public float XYZInterlValue;
        public float ABCInterlValue;
        

        public SpeedSetWindow()
        {
            InitializeComponent();

        }

        private void SpeedSetClick(object sender, RoutedEventArgs e)
        {
            JointSpeed=int.Parse(jointspeed.Text);
            XYZSpeed = int.Parse(XYZspeed.Text);
            ABCSpeed = int.Parse(ABCspeed.Text);
            C2183Speed = int.Parse(c2183speed.Text);
            JointInterlValue = float.Parse(jointintelvalue.Text);
            XYZInterlValue = float.Parse(XYZintelvalue.Text);
            ABCInterlValue = float.Parse(ABCintelvalue.Text);
            MainWindow.mainwindow.timer.Interval = TimeSpan.FromMilliseconds(double.Parse(Tick.Text));


          JointMovePage.jointmovepage.Joint1.IntervalTime = JointSpeed;
            JointMovePage.jointmovepage.Joint2.IntervalTime = JointSpeed;
            JointMovePage.jointmovepage.Joint3.IntervalTime = JointSpeed;
            JointMovePage.jointmovepage.Joint4.IntervalTime = JointSpeed;
            JointMovePage.jointmovepage.Joint5.IntervalTime = JointSpeed;
           JointMovePage.jointmovepage.Joint6.IntervalTime = JointSpeed;
            CartesianMovePage.cartesianmovpage.X.IntervalTime = XYZSpeed;
            CartesianMovePage.cartesianmovpage.Y.IntervalTime = XYZSpeed;
            CartesianMovePage.cartesianmovpage.Z.IntervalTime = XYZSpeed;
            CartesianMovePage.cartesianmovpage.A.IntervalTime = ABCSpeed;
            CartesianMovePage.cartesianmovpage.B.IntervalTime = ABCSpeed;
            CartesianMovePage.cartesianmovpage.C.IntervalTime = ABCSpeed;
            MainWindow.mainwindow.Control2183Speed = C2183Speed;


            // 直接设置速度
            Control2183.Control2183.control.Command_speed(C2183Speed);

            JointMovePage.jointmovepage.Joint1.IncrementValue = JointInterlValue;
            JointMovePage.jointmovepage.Joint2.IncrementValue= JointInterlValue;
            JointMovePage.jointmovepage.Joint3.IncrementValue = JointInterlValue;
            JointMovePage.jointmovepage.Joint4.IncrementValue = JointInterlValue;
            JointMovePage.jointmovepage.Joint5.IncrementValue = JointInterlValue;
            JointMovePage.jointmovepage.Joint6.IncrementValue = JointInterlValue;
            CartesianMovePage.cartesianmovpage.X.IncrementValue = XYZInterlValue;
            CartesianMovePage.cartesianmovpage.Y.IncrementValue = XYZInterlValue;
            CartesianMovePage.cartesianmovpage.Z.IncrementValue = XYZInterlValue;
            CartesianMovePage.cartesianmovpage.A.IncrementValue = ABCInterlValue;
            CartesianMovePage.cartesianmovpage.B.IncrementValue = ABCInterlValue;
            CartesianMovePage.cartesianmovpage.C.IncrementValue = ABCInterlValue;

            this.Close();
        }
    }
}
