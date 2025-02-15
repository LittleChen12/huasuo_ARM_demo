using System;
using System.Collections.Generic;
using System.IO;
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
    /// RotundityPathWindow.xaml 的交互逻辑
    /// </summary>
    public partial class RotundityPathWindow : Window
    {
        public Vector3 COCircle;
        public Vector3 NormalVector;
        public float radius;
        public float length;
        public int length_num;
        public int round_num;
        public float posture=0;
        public bool reverse = false;
      
        public RotundityPathWindow()
        {
            InitializeComponent();
            EndZ.Text = "0";
        }

        private void en_Click(object sender, RoutedEventArgs e)
        {
            COCircle.X = float.Parse(startx.Text);
            COCircle.Y = float.Parse(starty.Text);
            COCircle.Z = float.Parse(startz.Text);


            NormalVector.X = float.Parse(DirX.Text);
            NormalVector.Y = float.Parse(DirY.Text);
            NormalVector.Z = float.Parse(DirZ.Text);
            round_num = int.Parse(EndX.Text);
            length_num = int.Parse(EndY.Text);
            posture = float.Parse(EndZ.Text);
            radius = float.Parse(Width.Text);
            length = float.Parse(Length.Text);
            reverse = true;
            if (Reverse.Text == "1") reverse = true;
            if (Reverse.Text == "0") reverse = false;
            this.Close();

        }

        private void not_Click(object sender, RoutedEventArgs e)
        {

        }
    }
}
