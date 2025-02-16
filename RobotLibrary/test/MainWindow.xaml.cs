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
using RobotLibrary;
using RobotLibraryAlgorithm;
using Robots;

namespace test
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        /// <summary>
        /// manager字段：机器人结构、一些参数、2183、串口、工具坐标
        /// positionplan字段：movej,movel,虚拟轴、轨迹生成功能
        /// </summary>
        RobotManager manager;
        AlgorithmManager algorithmManager;
        public MainWindow()
        {
            InitializeComponent();
            manager = new RobotManager();
            
        }

        private void Button_Click(object sender, RoutedEventArgs e)
        {
            //manager.Init();
            //manager.ConvertToRobotMatrix();

            //moveinter.MoveL();
            //moveinter.MoveJ;
            //trajectoryplanning.SphericalPath();
            //algorithm.RadToAngle();

        }
    }
}
