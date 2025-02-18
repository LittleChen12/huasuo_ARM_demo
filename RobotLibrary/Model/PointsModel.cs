using HelixToolkit.Wpf;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media;

namespace Model
{
    public class PointsModel
    {
        public PointsVisual3D PointsVisual;
        public PointsModel()
        {
            PointsVisual = new PointsVisual3D();
          
        }
        /// <summary>
        /// 初始化路径点模型
        /// mm为毫米单位
        /// </summary>
        public void PointsModelInit()
        {
            PointsVisual.Color = Colors.Red;
            PointsVisual.Size = 4;
        }
        public void PintsModelClear()
        {
            PointsVisual.Points.Clear();
            PointsVisual.Children.Clear();
        }
    }
}
