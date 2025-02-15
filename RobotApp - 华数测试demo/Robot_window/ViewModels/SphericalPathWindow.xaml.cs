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
    /// SphericalPathWindow.xaml 的交互逻辑
    /// </summary>
    public partial class SphericalPathWindow : Window
    {
        public Vector3 COSphere;
        public Vector3 dirVector;
        public float radius;
        public float startrad;
        public float endrad;
        public int rad_num;
        public int round_num;
        public float posture = 0;
        public bool reverse = false;

       


        public SphericalPathWindow()
        {
            InitializeComponent();
        }

        private void en_Click(object sender, RoutedEventArgs e)
        {
            COSphere.X = float.Parse(startx.Text);
            COSphere.Y = float.Parse(starty.Text);
            COSphere.Z = float.Parse(startz.Text);


            dirVector.X = float.Parse(DirX.Text);
            dirVector.Y = float.Parse(DirY.Text);
            dirVector.Z = float.Parse(DirZ.Text);
            radius = float.Parse(EndX.Text);
            startrad = float.Parse(EndY.Text);
            endrad = float.Parse(EndZ.Text);
            rad_num = int.Parse(Width.Text);
            round_num = int.Parse(Length.Text);
            if (Reverse.Text == "1") reverse = true;
            if (Reverse.Text == "0") reverse = false;
            this.Close();
        }

        private void not_Click(object sender, RoutedEventArgs e)
        {

        }


    }
}
