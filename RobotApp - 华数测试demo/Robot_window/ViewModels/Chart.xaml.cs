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
using System.Windows.Shapes;
using RobotLibrary;
using ScottPlot;
using ScottPlot.Plottables;

namespace Robot_window.ViewModels
{
    /// <summary>
    /// Chart.xaml 的交互逻辑
    /// </summary>
    public partial class Chart : Window
    {
        public Scatter line;
        public Chart()
        {
            InitializeComponent();
        }

        public void pltshow(List<Position> positions, int i)
        {
            List<double> list_t = new List<double>();
            foreach (var position in positions)
            {
                list_t.Add(position.Pose.t);
            }
            List<double> list_theta = new List<double>();
            foreach (var position in positions)
            {
                list_theta.Add(position.Joints.Joints[i]);
            }
            var ts = list_t.ToArray();
            var thetas = list_theta.ToArray();
            line = WpfPlot1.Plot.Add.Scatter(ts, thetas);
            line.LegendText = string.Format("theta.{i}");
            // 设置图表标题和轴标签
            WpfPlot1.Plot.Title("jiaodu");
            WpfPlot1.Plot.XLabel("t /s");
            WpfPlot1.Plot.YLabel("theta /rad");
            WpfPlot1.Plot.ShowLegend();
            // 刷新图表
            WpfPlot1.Refresh();
        }
        //画角度图
        public void pltshow(List<Position> positions)
        {
            List<double> list_t = new List<double>();
            foreach (var position in positions)
            {
                list_t.Add(position.Pose.t);
            }
            var ts = list_t.ToArray();

            for (int i = 0; i < 6; i++)
            {
                List<double> list_theta = new List<double>();
                foreach (var position in positions)
                {
                    list_theta.Add(position.Joints.Joints[i]);
                }
                var thetas = list_theta.ToArray();
                line = WpfPlot1.Plot.Add.Scatter(ts, thetas);

                switch (i)
                {
                    case 0:
                        line.LinePattern = LinePattern.Dotted;
                        break;
                    case 1:
                        line.LinePattern = LinePattern.Solid;
                        break;
                    case 2:
                        line.LinePattern = LinePattern.Dashed;
                        break;
                    case 3:
                        line.LinePattern = LinePattern.DenselyDashed;
                        break;
                    case 4:
                        line.LinePattern = LinePattern.Solid;
                        break;
                    case 5:
                        line.LinePattern = LinePattern.Solid;
                        break;
                }

                line.LegendText = string.Format("theta{0}", i + 1);
                int middleIndex = ts.Length / 2;
                double middleT = ts[middleIndex];
                double middleTheta = thetas[middleIndex];
                WpfPlot1.Plot.Add.Text($"{line.LegendText}", middleT, middleTheta);
            }
            // 设置图表标题和轴标签
            WpfPlot1.Plot.Title("jiaodu");
            WpfPlot1.Plot.XLabel("t /s");
            WpfPlot1.Plot.YLabel("theta /rad");
            WpfPlot1.Plot.ShowLegend();
            // 刷新图表
            WpfPlot1.Refresh();
        }
        public void pltshow(double[] ts, double[] ys)
        {
            line = WpfPlot1.Plot.Add.Scatter(ts, ys);

            // 刷新图表
            WpfPlot1.Refresh();
        }
        public void Set(string title = "title", string xLabel = "x", string yLabel = "y")
        {
            WpfPlot1.Plot.Title(title);
            WpfPlot1.Plot.XLabel(xLabel);
            WpfPlot1.Plot.YLabel(yLabel);
            WpfPlot1.Refresh();
        }
        public void SetlegendText(string legendText = "legend")
        {
            line.LegendText = legendText;
            WpfPlot1.Plot.ShowLegend();
            WpfPlot1.Refresh();
        }
        public void SetLineStyle(LinePattern LineStyle)
        {
            line.LinePattern = LineStyle;
            WpfPlot1.Refresh();
        }
        private void WpfPlot1_MouseMove(object sender, MouseEventArgs e)
        {
            // 获取鼠标位置
            Point p = e.GetPosition(WpfPlot1);
            Pixel mousePixel = new Pixel(p.X * WpfPlot1.DisplayScale, p.Y * WpfPlot1.DisplayScale);
            Coordinates coordinates = WpfPlot1.Plot.GetCoordinates(mousePixel);

            // 查找最近的数据点
            var plottables = WpfPlot1.Plot.GetPlottables();
            foreach (var plottable in plottables)
            {
                if (plottable is ScottPlot.Plottables.Scatter scatter)
                {
                    var renderDetails = WpfPlot1.Plot.LastRender;
                    var nearestPoint = scatter.GetNearest(coordinates, renderDetails);
                    if ((double.IsFinite(nearestPoint.X) && double.IsFinite(nearestPoint.Y)))
                    {
                        // 显示数据点的值
                        PointInfo.Text = $"X: {nearestPoint.X:0.####}, Y: {nearestPoint.Y:0.####}";
                        return;
                    }
                }
            }


        }

    

}
}
