using Device;
using HelixToolkit.Wpf;
using Microsoft.Win32;
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
        public RobotManager robotmanger = new RobotManager();
        private List<string> _frames;

        public List<string> Frames
        {
            get { return _frames; }
            set { _frames = value; }
        }
        public MainWindow()
        {
            InitializeComponent();
            Frames = new List<string>() { "关节坐标", "笛卡尔坐标", "法兰坐标" };
        }
        
    }
}
