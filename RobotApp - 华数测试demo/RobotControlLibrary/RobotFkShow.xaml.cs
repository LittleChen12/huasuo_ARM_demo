using RobotLibrary;
using System;
using System.Collections.Generic;
using System.ComponentModel;
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

namespace RobotControlLibrary
{
    /// <summary>
    /// RobotFkShow.xaml 的交互逻辑
    /// </summary>
    public partial class RobotFkShow : UserControl, INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler? PropertyChanged;
        //private CartesianPosition  position;



        public CartesianPosition position
        {
            get { return (CartesianPosition)GetValue(positionProperty); }
            set { SetValue(positionProperty, value);
                if (PropertyChanged != null)
                    PropertyChanged.Invoke(this, new PropertyChangedEventArgs("position"));
            }
        }

        public static readonly DependencyProperty positionProperty =
            DependencyProperty.Register("position", typeof(CartesianPosition), typeof(RobotFkShow), new PropertyMetadata(new CartesianPosition()));



        public RobotFkShow()
        {
            InitializeComponent();
        }

      
    }
}
