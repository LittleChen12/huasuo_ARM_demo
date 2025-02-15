using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
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
    /// LinePathWindow.xaml 的交互逻辑
    /// </summary>
    public partial class LinePathWindow : Window
    {
        public double start_x, start_y, start_z,end_x, end_y, end_z;
        public double A_start, A_end;
        public double B_start, B_end;
        public double C_start, C_end;
        public int num;
        public LinePathWindow()
        {
            InitializeComponent();
        }

        private void en_Click(object sender, RoutedEventArgs e)
        {
            start_x = double.Parse(startx.Text);
            start_y= double.Parse(starty.Text);
            start_z = double.Parse(startz.Text);
            end_x = double.Parse(EndX.Text);
            end_y = double.Parse(EndY.Text);
            end_z = double.Parse(EndZ.Text);

            A_start = double.Parse(Astart.Text)*Math.PI/180;
            B_start = double.Parse(Bstart.Text)*Math.PI/180;
            C_start = double.Parse(Cstart.Text) * Math.PI / 180;
            A_end = double.Parse(Aend.Text) * Math.PI / 180;
            B_end = double.Parse(Bend.Text) * Math.PI / 180;
            C_end = double.Parse(Cend.Text) * Math.PI / 180;
            num = int.Parse(_num.Text);
            this.Close();
        }

        private void not_Click(object sender, RoutedEventArgs e)
        {

        }
    }
}
