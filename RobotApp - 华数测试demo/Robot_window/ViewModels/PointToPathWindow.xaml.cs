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
using System.Windows.Shapes;

namespace Robot_window.ViewModels
{
    /// <summary>
    /// PointToPathWindow.xaml 的交互逻辑
    /// </summary>
    public partial class PointToPathWindow : Window
    {
        public Vector3 StartPoint = new Vector3();
        public Vector3 EndPoint = new Vector3();
        public Vector3 DirVector = new Vector3();
        public float width;
        public float lenght;
        public bool re;
        public PointToPathWindow()
        {
            InitializeComponent();
        }

        private void en_Click(object sender, RoutedEventArgs e)
        {
            StartPoint.X = float.Parse(startx.Text);
            StartPoint.Y = float.Parse(starty.Text);
            StartPoint.Z = float.Parse(startz.Text);

            EndPoint.X = float.Parse(EndX.Text);
            EndPoint.Y = float.Parse(EndY.Text);
            EndPoint.Z = float.Parse(EndZ.Text);

            DirVector.X = float.Parse(DirX.Text);
            DirVector.Y = float.Parse(DirY.Text);
            DirVector.Z = float.Parse(DirZ.Text);

            width = float.Parse(Width.Text);
            lenght = float.Parse(Length.Text);
            re = true;
            //this.Close();
        }

        private void not_Click(object sender, RoutedEventArgs e)
        {

        }

        private void not_Click2(object sender, RoutedEventArgs e)
        {

        }
    }
}
