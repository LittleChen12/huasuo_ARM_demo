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
using MathNet.Numerics;
using RobotLibrary;
using ScottPlot;
using ScottPlot.Plottables;
using static FParsec.ErrorMessage;

namespace Robot_window.ViewModels
{
    /// <summary>
    /// Chart.xaml 的交互逻辑
    /// </summary>
    public partial class Chart : System.Windows.Window
    {
        public Scatter line;
        public static int order = 3;
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
        public void pltshowChange(double[] ts, double[] ys, int[] keyPointIndices)
        {
            line = WpfPlot1.Plot.Add.Scatter(ts, ys);
            int keyPointIndex = 2;
            foreach (var index in keyPointIndices)
            {
                line = WpfPlot1.Plot.Add.Scatter(new[] { ts[index] }, new[] { ys[index] });
                line.MarkerSize = 20;
            }
            // 刷新图表
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
        public static void Q3Draw(double[,] Q3, List<CartesianPosition> CartesianPositions)
        {
            Chart chart3 = new Chart();
            chart3.Show();
            List<double> list_t = new List<double>();
            foreach (var position in CartesianPositions)
            {
                list_t.Add(position.t);
            }
            var ts = list_t.ToArray();
            var Q31 = new double[ts.Length];
            // 使用 LINQ 获取指定行的数据

            int columnsCount = Q3.GetLength(1); // 获取列数
            Q31 = Enumerable.Range(0, columnsCount)
                                       .Select(col => Q3[0, col])
                                       .ToArray();
            double[] Q3Value = new double[ts.Length];
            int j = 0;
            foreach (var item in ts)
            {
                Q3Value[j] = SymbolDerivation.StringExpretion(NewIK.Q3String, item);
                j++;
            }
            chart3.pltshow(ts, Q31);
            chart3.SetlegendText("Q3true");
            chart3.pltshow(ts, Q3Value);
            chart3.SetlegendText("Q3nihe");
            chart3.SetLineStyle(LinePattern.Dotted);
            chart3.Set("Q3:" + NewIK.Q3String, "t", "Q3");
        }
        public static void QDraw(double[,] Q, int[] orders, List<CartesianPosition> CartesianPositions)
        {
            Chart chart3 = new Chart();
            chart3.Show();
            List<double> list_t = new List<double>();
            foreach (var position in CartesianPositions)
            {
                list_t.Add(position.t);
            }
            var ts = list_t.ToArray();
            var Q1 = new double[ts.Length];
            // 使用 LINQ 获取指定行的数据

            int columnsCount = Q.GetLength(1); // 获取列数
            Q1 = Enumerable.Range(0, columnsCount)
                                       .Select(col => Q[0, col])
                                       .ToArray();
            var Q2 = new double[ts.Length];
            // 使用 LINQ 获取指定行的数据
            Q2 = Enumerable.Range(0, columnsCount)
                                       .Select(col => Q[1, col])
                                       .ToArray();
            var Q3 = new double[ts.Length];
            // 使用 LINQ 获取指定行的数据
            Q3 = Enumerable.Range(0, columnsCount)
                                       .Select(col => Q[2, col])
                                       .ToArray();
            var Q4 = new double[ts.Length];
            // 使用 LINQ 获取指定行的数据
            Q4 = Enumerable.Range(0, columnsCount)
                                       .Select(col => Q[3, col])
                                       .ToArray();
            var Q5 = new double[ts.Length];
            // 使用 LINQ 获取指定行的数据
            Q5 = Enumerable.Range(0, columnsCount)
                                       .Select(col => Q[4, col])
                                       .ToArray();
            var Q6 = new double[ts.Length];
            // 使用 LINQ 获取指定行的数据
            Q6 = Enumerable.Range(0, columnsCount)
                                       .Select(col => Q[5, col])
                                       .ToArray();

            double[] Q1Coefficients = Fit.Polynomial(ts, Q1, order);
            NewIK.Q1Value = new double[ts.Length];
            NewIK.Q1String = SymbolDerivation.FuncStringProduce(Q1Coefficients, 0, order);

            double[] Q2Coefficients = Fit.Polynomial(ts, Q2, order);
            NewIK.Q2Value = new double[ts.Length];
            NewIK.Q2String = SymbolDerivation.FuncStringProduce(Q2Coefficients, 0, order);

            double[] Q3Coefficients = Fit.Polynomial(ts, Q3, order);
            NewIK.Q3Value = new double[ts.Length];
            NewIK.Q3String = SymbolDerivation.FuncStringProduce(Q3Coefficients, 0, order);

            double[] Q4Coefficients = Fit.Polynomial(ts, Q4, order);
            NewIK.Q4Value = new double[ts.Length];
            NewIK.Q4String = SymbolDerivation.FuncStringProduce(Q4Coefficients, 0, order);

            double[] Q5Coefficients = Fit.Polynomial(ts, Q5, order);
            NewIK.Q5Value = new double[ts.Length];
            NewIK.Q5String = SymbolDerivation.FuncStringProduce(Q5Coefficients, 0, order);

            double[] Q6Coefficients = Fit.Polynomial(ts, Q6, order);
            NewIK.Q6Value = new double[ts.Length];
            NewIK.Q6String = SymbolDerivation.FuncStringProduce(Q6Coefficients, 0, order);



            //Console.WriteLine("Q1表达式：" + PathClass.Q1String);

            int j = 0;
            foreach (var item in ts)
            {
                NewIK.Q1Value[j] = SymbolDerivation.StringExpretion(NewIK.Q1String, item);
                NewIK.Q2Value[j] = SymbolDerivation.StringExpretion(NewIK.Q2String, item);
                NewIK.Q3Value[j] = SymbolDerivation.StringExpretion(NewIK.Q3String, item);
                NewIK.Q4Value[j] = SymbolDerivation.StringExpretion(NewIK.Q4String, item);
                NewIK.Q5Value[j] = SymbolDerivation.StringExpretion(NewIK.Q5String, item);
                NewIK.Q6Value[j] = SymbolDerivation.StringExpretion(NewIK.Q6String, item);
                j++;
            }

            chart3.pltshowChange(ts, Q1, orders);
            chart3.SetlegendText("Q1true");
            chart3.pltshow(ts, NewIK.Q1Value);
            chart3.SetlegendText("Q1nihe");

            chart3.pltshow(ts, Q2);
            chart3.SetlegendText("Q2true");
            chart3.pltshow(ts, NewIK.Q2Value);
            chart3.SetlegendText("Q2nihe");

            chart3.pltshow(ts, Q3);
            chart3.SetlegendText("Q3true");
            chart3.pltshow(ts, NewIK.Q3Value);
            chart3.SetlegendText("Q3nihe");

            chart3.pltshow(ts, Q4);
            chart3.SetlegendText("Q4true");
            chart3.pltshow(ts, NewIK.Q4Value);
            chart3.SetlegendText("Q4nihe");

            chart3.pltshow(ts, Q5);
            chart3.SetlegendText("Q5true");
            chart3.pltshow(ts, NewIK.Q5Value);
            chart3.SetlegendText("Q5nihe");

            chart3.pltshow(ts, Q6);
            chart3.SetlegendText("Q6true");
            chart3.pltshow(ts, NewIK.Q6Value);
            chart3.SetlegendText("Q6nihe");


            //chart3.SetLineStyle(LinePattern.Dotted);
            chart3.Set("Q3:" + NewIK.Q3String, "t", "Q");
        }
        public static void A6B6Draw(double[,] A6B6, List<CartesianPosition> CartesianPositions)
        {
            Chart chart2 = new Chart();
            chart2.Show();
            List<double> list_t = new List<double>();
            foreach (var position in CartesianPositions)
            {
                list_t.Add(position.t);
            }
            var ts = list_t.ToArray();
            var A6 = new double[ts.Length];
            // 使用 LINQ 获取指定行的数据

            int columnsCount = A6B6.GetLength(1); // 获取列数
            A6 = Enumerable.Range(0, columnsCount)
                                       .Select(col => A6B6[0, col])
                                       .ToArray();
            var B6 = new double[ts.Length];
            // 使用 LINQ 获取指定行的数据
            B6 = Enumerable.Range(0, columnsCount)
                                      .Select(col => A6B6[1, col])
                                      .ToArray();

            double[] A6Coefficients = Fit.Polynomial(ts, A6, 3);
            double[] A6Value = new double[ts.Length];
            NewIK.A6String = SymbolDerivation.FuncStringProduce(A6Coefficients, 0, order);
            double[] B6Coefficients = Fit.Polynomial(ts, B6, 3);
            double[] B6Value = new double[ts.Length];
            NewIK.B6String = SymbolDerivation.FuncStringProduce(B6Coefficients, 0, order);
            Console.WriteLine("A6表达式：" + NewIK.A6String);
            Console.WriteLine("B6表达式：" + NewIK.B6String);

            int j = 0;
            foreach (var item in ts)
            {
                A6Value[j] = SymbolDerivation.StringExpretion(NewIK.A6String, item);
                B6Value[j] = SymbolDerivation.StringExpretion(NewIK.B6String, item);
                j++;
            }
            chart2.pltshow(ts, A6);
            chart2.SetlegendText("A6true");
            chart2.pltshow(ts, A6Value);
            chart2.SetlegendText("A6nihe");
            chart2.SetLineStyle(LinePattern.Dotted);
            chart2.Set("A6:" + NewIK.A6String + " B6:" + NewIK.B6String, "t", "A6");

            chart2.pltshow(ts, B6);
            chart2.SetlegendText("B6true");
            chart2.pltshow(ts, B6Value);
            chart2.SetlegendText("B6nihe");
            chart2.SetLineStyle(LinePattern.Dashed);

        }



    }
}
