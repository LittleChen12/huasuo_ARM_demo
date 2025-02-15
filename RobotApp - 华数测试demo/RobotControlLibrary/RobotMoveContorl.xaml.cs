using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Reflection.Metadata;
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

namespace RobotControlLibrary
{
    /// <summary>
    /// RobotMoveContorl.xaml 的交互逻辑
    /// </summary>
    /// 
    
    public partial class RobotMoveContorl : UserControl,INotifyPropertyChanged
    {
        //当前关节角度
        public event PropertyChangedEventHandler? PropertyChanged;
        private double jointvalue;
        public double JointValue
        {
            get { return jointvalue; }
            set
            {
                jointvalue = value;
                if (PropertyChanged != null)
                {
                    PropertyChanged.Invoke(this, new PropertyChangedEventArgs("JointValue"));   
                }
            }
        }
       

        //机械臂每个关节运动限定范围的两个依赖属性
        public double Mininum
        {
            get { return (double)GetValue(MininumProperty); }
            set { SetValue(MininumProperty, value); }
        }
        public static readonly DependencyProperty MininumProperty =
            DependencyProperty.Register("Mininum", typeof(double), typeof(RobotMoveContorl), new PropertyMetadata(-180.0));
      
        public double Maxium
        {
            get { return (double)GetValue(MaxiumProperty); }
            set { SetValue(MaxiumProperty, value); }
        }
        public static readonly DependencyProperty MaxiumProperty =
            DependencyProperty.Register("Maxium", typeof(double), typeof(RobotMoveContorl), new PropertyMetadata(180.0));

        //每次按钮触发，jointvalue的增量
        public double IncrementValue
        {
            get { return (double)GetValue(IncrementValueProperty); }
            set { SetValue(IncrementValueProperty, value); }
        }
        public static readonly DependencyProperty IncrementValueProperty =
            DependencyProperty.Register("IncrementValue", typeof(double), typeof(RobotMoveContorl), new PropertyMetadata(0.1));


        //按钮点击间隔依赖属性 单位：ms
        public int DelayTime
        {
            get { return (int)GetValue(DelayTimeProperty); }
            set { SetValue(DelayTimeProperty, value); }
        }
        public static readonly DependencyProperty DelayTimeProperty =
            DependencyProperty.Register("DelayTime", typeof(int), typeof(RobotMoveContorl), new PropertyMetadata(500));

        //触发事件间隔属性 单位:ms
        public int IntervalTime
        {
            get { return (int)GetValue(IntervalTimeProperty); }
            set { SetValue(IntervalTimeProperty, value); }
        }

        public static readonly DependencyProperty IntervalTimeProperty =
            DependencyProperty.Register("IntervalTime", typeof(int), typeof(RobotMoveContorl), new PropertyMetadata(30));



        public string Name
        {
            get { return (string)GetValue(NameProperty); }
            set { SetValue(NameProperty, value); }
        }
        public static readonly DependencyProperty NameProperty =
            DependencyProperty.Register("Name", typeof(string), typeof(RobotMoveContorl), new PropertyMetadata("Joint"));


        public event RoutedEventHandler Clickd;
       


        public RobotMoveContorl()
        {
            InitializeComponent();
            this.DataContext = this;
            jointvalue = 0;
        }

        private void IncreaseJointValue(object sender, RoutedEventArgs e)
        {
            
            JointValue += IncrementValue;
            if (jointvalue > Maxium) { JointValue = (double)Maxium; }
            Clickd?.Invoke(this, e);
        }

        private void DecreaseJointValue(object sender, RoutedEventArgs e)
        {
            JointValue -= IncrementValue;

            if (jointvalue < Mininum) { JointValue = (double)Mininum; }
            Clickd?.Invoke(this, e);

        }
    }
}
