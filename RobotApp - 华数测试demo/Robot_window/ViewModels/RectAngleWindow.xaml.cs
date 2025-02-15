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
using System.Windows.Media.Media3D;
using System.Windows.Shapes;

namespace Robot_window.ViewModels
{
    /// <summary>
    /// RectAngleWindow.xaml 的交互逻辑
    /// </summary>
    public partial class RectAngleWindow : Window
    {
        public Point3D Point=new Point3D(0,0,0);
        public double xlenght;
        public double ylenght;
        public double zlenght;
        public RectAngleWindow()
        {
            InitializeComponent();
        }

        private void en_Click(object sender, RoutedEventArgs e)
        {
            Point.X = double.Parse(centerx.Text);
            Point.Y = double.Parse(centery.Text);
            Point.Z = double.Parse(centerz.Text);
            xlenght = double.Parse(LengthX.Text);
            ylenght=double.Parse(LengthY.Text);
            zlenght=double.Parse(LengthZ.Text);
            this.Close();
        }

        private void not_Click(object sender, RoutedEventArgs e)
        {

        }
    }
}
