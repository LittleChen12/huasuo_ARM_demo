using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Symbolics;
using Expr = MathNet.Symbolics.SymbolicExpression;

namespace RobotLibrary
{
    public class SymbolDerivation
    {
        //求导示例：
        //Func<double, double> f = x => x * x + 3 * x + 2;
        //string str = "x^2+3*x+2";
        //string x = "x";
        //double t = PathClass.symbolDerivation(str, x, 2);
        //double xx = PathClass.NumericalDerivation(f, 2);
        public static double NumericalDerivation(Func<double, double> f, double x, double h = 1e-6)
        {
            double derivative = (f(x + h) - f(x - h)) / (2 * h);
            return derivative;
        }
        public static double symbolDerivation(string expression, string variableName, double value, out string dy)
        { // 1. 定义符号变量
            var variable = Expr.Variable(variableName);

            // 2. 解析输入表达式（例如 "x^2 + sin(x)"）
            var expr = (Expr)Infix.ParseOrThrow(expression);

            //3.符号求导（调用静态方法 Expr.Differentiate）
            var derivative = expr.Differentiate(variable);
            var formattedDerivative = Infix.Format(derivative.Expression);
            dy = formattedDerivative;
            // 4. 代入数值计算
            var substitutions = new Dictionary<string, FloatingPoint>
            {
                { variableName, value }
            };
            var result = derivative.Evaluate(symbols: substitutions);
            return result.RealValue;

            //var x = Expr.Variable("x");

            //// 定义表达式 f(x) = x^2 + 3x + 2
            //var f = x.Pow(2) + 3 * x + 2;

            //// 对表达式求导
            //var derivative = f.Differentiate(x);
            //Console.WriteLine($"The derivative of f(x) is {derivative}");

            //var xValue = 2.0;
            //var substituted = derivative.Substitute(x, Expr.Real(xValue));

            //// 计算数值结果
            //var symbols = new Dictionary<string, FloatingPoint> { { "x", FloatingPoint.NewReal(xValue) } };
            //var result = substituted.Evaluate(symbols).RealValue;    
        }
        /// <summary>
        /// 字符串表达式求值
        /// </summary>
        /// <param name="expression"></param>
        /// <param name="x"></param>
        /// <returns></returns>
        public static double StringExpretion(string expression, double x)
        {
            var expr = (Expr)Infix.ParseOrThrow(expression);
            var symbols = new Dictionary<string, FloatingPoint> { { "t", FloatingPoint.NewReal(x) } };
            var result = expr.Evaluate(symbols).RealValue;
            return result;
        }
        /// <summary>
        /// 系数数组转函数字符串
        /// </summary>
        /// <param name="coefficients"></param>
        /// <param name="start"></param>
        /// <param name="order"></param>
        /// <returns></returns>
        public static string FuncStringProduce(double[] coefficients, int start, int order)
        {
            string funstring = "";
            for (int i = start; i <= start + order; i++)
            {
                if ((i + 1) <= start + order)
                {
                    if (coefficients[i + 1] > 0)
                    {
                        string str = string.Format("{0}*t^{1}+", coefficients[i], i - start);
                        funstring += str;
                    }
                    else
                    {
                        string str = string.Format("{0}*t^{1}", coefficients[i], i - start);
                        funstring += str;
                    }
                }
                else
                {
                    string str = string.Format("{0}*t^{1}", coefficients[i], i - start);
                    funstring += str;
                }
            }
            return funstring;
        }

    }
}
