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
        /// <summary>
        /// 轨迹清除
        /// </summary>
        public void PointsModelClear()
        {
            PointsVisual.Points.Clear();
            PointsVisual.Children.Clear();
        }
        /// <summary>
        /// 设置路径点模型属性
        /// </summary>
        /// <param name="color"></param>
        /// <param name="size"></param>
        public void PoinModelSet(Color color, double size=4)
        {
            PointsVisual.Color = color;
            PointsVisual.Size = size;
        }
    }
}
