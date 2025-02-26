using MathNet.Numerics;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace RobotLibrary
{
    public class NewIK
    {
        private static Matrix4x4 fame6ToGrip = new Matrix4x4(
            0, 0, -1, 0f,
            0, 1, 0, 0f,
            1, 0, 0, 0f,
            0, 0, 0, 1.0f
         );

        private static Matrix4x4 huashu_moduan = new Matrix4x4(
            1, 0, 0, 0f,
            0, 1, 0, 0f,
            0, 0, 1, 95f,
            0, 0, 0, 1.0f
         );

        public static bool SolveQFlag = true;
        public static bool SolveQFlag6 = true;
        public static bool SolveQFlag3 = true;

        //flag==1时，进入表达式1.flag设置为全局变量
        //x,y,z的函数表达式
        public static string? FunctionExpressionx;
        public static string? FunctionExpressiony;
        public static string? FunctionExpressionz;

        public static string A6String = "";
        public static string B6String = "";
        public static string Q1String = "";
        public static string Q2String = "";
        public static string Q3String = "";
        public static string Q4String = "";
        public static string Q5String = "";
        public static string Q6String = "";

        public static double[]? Q1Value;
        public static double[]? Q2Value;
        public static double[]? Q3Value;
        public static double[]? Q4Value;
        public static double[]? Q5Value;
        public static double[]? Q6Value;
        public static double tik = 0;

        public static List<double>? TChange;

        public static double[] SolveQ(double px, double py, double pz, double t)
        {
            //q1表达式在不同时段不一样，例如从0到0.5时刻，是第二个表达式，0.5是奇异位置，
            //则0.5到最后改用第一个表达式
            int k1 = 1;
            double q1, q2, q3, q4, q5, q6;
            double[] q = new double[6];
            if (SolveQFlag && (px == 0 && py == 0) || ((-720 * t == 0) && (24 * t + 72 * t * (6 * Math.Pow(t, 2) - 20)) == 0))
            {
                double pxdd = -720;
                double pydd = 1296 * Math.Pow(t, 2) - 1416;
                q1 = Math.Atan2(-pxdd, pydd) + Math.Atan2(-k1, 0);
                //SolveQFlag = false;//对全局变量进行设置
            } //表达式1
            else
                q1 = Math.Atan2(-px, py) + Math.Atan2(k1, 0);        //表达式2
            q[0] = q1;
            //q[1] = q2; q[2] = q3; q[3] = q4; q[4] = q5; q[5] = q6;
            return q;
        }

        public static CartesianPosition FKNewRad(double[] joints, Matrix4x4? Grip2Tool = null)
        {
            Matrix4x4 Grip2toolMatrix = Grip2Tool ?? Matrix4x4.Identity;

            var result = new CartesianPosition();
            double[,] T1 = new double[4, 4];
            double[,] T2 = new double[4, 4];
            double[,] T3 = new double[4, 4];
            double[,] T4 = new double[4, 4];
            double[,] T5 = new double[4, 4];
            double[,] T6 = new double[4, 4];
            double[,] T_last = new double[4, 4];

            double a1 = 59.69338; double d1 = 456;
            double a2 = 440.64619; double d2 = 0;
            double a3 = 34.6563; double d3 = 0;
            double a4 = 0; double d4 = 410.69093;
            double a5 = 0; double d5 = 0;
            double a6 = 0; double d6 = 0;

            double alfa = Math.PI / 2;

            //Console.WriteLine("第一个角度: " + joints[0]);
            double u1 = Math.Sin(joints[0]);
            double u2 = Math.Cos(joints[0]);
            double v1 = Math.Sin(-alfa);
            double v2 = Math.Cos(-alfa);

            T1[0, 0] = u2;
            T1[0, 1] = -u1 * v2;
            T1[0, 2] = u1 * v1;
            T1[0, 3] = a1 * u2;
            T1[1, 0] = u1;
            T1[1, 1] = u2 * v2;
            T1[1, 2] = -u2 * v1;
            T1[1, 3] = a1 * u1;
            T1[2, 0] = 0;
            T1[2, 1] = v1;
            T1[2, 2] = v2;
            T1[2, 3] = d1;
            T1[3, 0] = 0;
            T1[3, 1] = 0;
            T1[3, 2] = 0;
            T1[3, 3] = 1;

            // Console.WriteLine("第二个角度: " + joints[1]);
            double u_2jiaodu1 = Math.Sin(joints[1] - Math.PI / 2);
            double u_2jiaodu2 = Math.Cos(joints[1] - Math.PI / 2);
            double v11 = Math.Sin(0);
            double v21 = Math.Cos(0);

            T2[0, 0] = u_2jiaodu2;
            T2[0, 1] = -u_2jiaodu1 * v21;
            T2[0, 2] = u_2jiaodu1 * v11;
            T2[0, 3] = a2 * u_2jiaodu2;
            T2[1, 0] = u_2jiaodu1;
            T2[1, 1] = u_2jiaodu2 * v21;
            T2[1, 2] = -u_2jiaodu2 * v11;
            T2[1, 3] = a2 * u_2jiaodu1;
            T2[2, 0] = 0;
            T2[2, 1] = v11;
            T2[2, 2] = v21;
            T2[2, 3] = d2;
            T2[3, 0] = 0;
            T2[3, 1] = 0;
            T2[3, 2] = 0;
            T2[3, 3] = 1;

            //Console.WriteLine("第三个角度: " + joints[2]);
            double u12 = Math.Sin(joints[2]);
            double u22 = Math.Cos(joints[2]);
            double v12 = Math.Sin(-alfa);
            double v22 = Math.Cos(-alfa);

            T3[0, 0] = u22;
            T3[0, 1] = -u12 * v22;
            T3[0, 2] = u12 * v12;
            T3[0, 3] = a3 * u22;
            T3[1, 0] = u12;
            T3[1, 1] = u22 * v22;
            T3[1, 2] = -u22 * v12;
            T3[1, 3] = a3 * u12;
            T3[2, 0] = 0;
            T3[2, 1] = v12;
            T3[2, 2] = v22;
            T3[2, 3] = d3;
            T3[3, 0] = 0;
            T3[3, 1] = 0;
            T3[3, 2] = 0;
            T3[3, 3] = 1;

            //Console.WriteLine("第四个角度: " + joints[3]);
            double u13 = Math.Sin(joints[3]);
            double u23 = Math.Cos(joints[3]);
            double v13 = Math.Sin(alfa);
            double v23 = Math.Cos(alfa);

            T4[0, 0] = u23;
            T4[0, 1] = -u13 * v23;
            T4[0, 2] = u13 * v13;
            T4[0, 3] = a4 * u23;
            T4[1, 0] = u13;
            T4[1, 1] = u23 * v23;
            T4[1, 2] = -u23 * v13;
            T4[1, 3] = a4 * u13;
            T4[2, 0] = 0;
            T4[2, 1] = v13;
            T4[2, 2] = v23;
            T4[2, 3] = d4;
            T4[3, 0] = 0;
            T4[3, 1] = 0;
            T4[3, 2] = 0;
            T4[3, 3] = 1;

            //Console.WriteLine("第五个角度: " + joints[4]);
            double u14 = Math.Sin(joints[4] + alfa);
            double u24 = Math.Cos(joints[4] + alfa);
            double v14 = Math.Sin(-alfa);
            double v24 = Math.Cos(-alfa);

            T5[0, 0] = u24;
            T5[0, 1] = -u14 * v24;
            T5[0, 2] = u14 * v14;
            T5[0, 3] = a5 * u24;
            T5[1, 0] = u14;
            T5[1, 1] = u24 * v24;
            T5[1, 2] = -u24 * v14;
            T5[1, 3] = a5 * u14;
            T5[2, 0] = 0;
            T5[2, 1] = v14;
            T5[2, 2] = v24;
            T5[2, 3] = d5;
            T5[3, 0] = 0;
            T5[3, 1] = 0;
            T5[3, 2] = 0;
            T5[3, 3] = 1;

            // Console.WriteLine("第六个角度: " + joints[5]);
            double u15 = Math.Sin(joints[5] + 2 * alfa);
            double u25 = Math.Cos(joints[5] + 2 * alfa);
            double v15 = Math.Sin(0);
            double v25 = Math.Cos(0);

            T6[0, 0] = u25;
            T6[0, 1] = -u15 * v25;
            T6[0, 2] = u15 * v15;
            T6[0, 3] = a6 * u25;
            T6[1, 0] = u15;
            T6[1, 1] = u25 * v25;
            T6[1, 2] = -u25 * v15;
            T6[1, 3] = a6 * u15;
            T6[2, 0] = 0;
            T6[2, 1] = v15;
            T6[2, 2] = v25;
            T6[2, 3] = d6;
            T6[3, 0] = 0;
            T6[3, 1] = 0;
            T6[3, 2] = 0;
            T6[3, 3] = 1;

            ////T_last = T1 * T2 * T3 * T4 * T5 * T6;
            var temp = BasicAlgorithm.MatrixMultiply(T1, T2);
            temp = BasicAlgorithm.MatrixMultiply(temp, T3);
            temp = BasicAlgorithm.MatrixMultiply(temp, T4);
            temp = BasicAlgorithm.MatrixMultiply(temp, T5);
            temp = BasicAlgorithm.MatrixMultiply(temp, T6);
            T_last = temp;
            //Console.WriteLine("T_last: " + T_last);

            double[,] R_last = new double[3, 3];
            for (int i = 0; i < 4; i++)
            {
                if (i == 3)
                    continue;

                for (int j = 0; j < 4; j++)
                {
                    if (j == 3)
                        continue;
                    R_last[i, j] = T_last[i, j];

                }
            }
            Matrix4x4 T = new Matrix4x4(
                           (float)T_last[0, 0], (float)T_last[0, 1], (float)T_last[0, 2], (float)T_last[0, 3],
                           (float)T_last[1, 0], (float)T_last[1, 1], (float)T_last[1, 2], (float)T_last[1, 3],
                           (float)T_last[2, 0], (float)T_last[2, 1], (float)T_last[2, 2], (float)T_last[2, 3],
                           (float)T_last[3, 0], (float)T_last[3, 1], (float)T_last[3, 2], (float)T_last[3, 3]
                           );  //T_last是6个角度相乘后的末端4*4矩阵，

            //Matrix4x4 RT = T * fame6ToGrip * Grip2toolMatrix * huashu_moduan;   // 正解  得到工具坐标系  第7个矩阵相乘结果
            Matrix4x4 RT = T * fame6ToGrip * huashu_moduan;
            //Matrix4x4 RT = T * huashu_moduan;

            //Console.WriteLine("R_last: " + R_last);
            var rresult = BasicAlgorithm.RotMatrixToRxyz(RT);
            result.X = RT.M14;
            result.Y = RT.M24;
            result.Z = RT.M34;
            result.RX = rresult[0];
            result.RY = rresult[1];
            result.RZ = rresult[2];

            return result;
        }
        public static Position IK_New(CartesianPosition endpose, Matrix4x4? Grip2Tool = null)
        {
            Stopwatch stopwatch = new Stopwatch();

            // 开始计时
            stopwatch.Start();

            Matrix4x4 Grip2ToolMatrix = Grip2Tool ?? Matrix4x4.Identity; //

            Matrix4x4 tool2GripMatrix = new Matrix4x4();
            Matrix4x4 huashu_moduan_ni = new Matrix4x4();
            Matrix4x4.Invert(Grip2ToolMatrix, out tool2GripMatrix);

            var T1 = BasicAlgorithm.CatsionPositionToTMatrix(endpose); //输入末端位姿   得到7个矩阵相乘的4*4矩阵
            //T1 mm
            Matrix4x4 T2 = new Matrix4x4(
                          (float)T1[0, 0], (float)T1[0, 1], (float)T1[0, 2], (float)T1[0, 3],
                           (float)T1[1, 0], (float)T1[1, 1], (float)T1[1, 2], (float)T1[1, 3],
                           (float)T1[2, 0], (float)T1[2, 1], (float)T1[2, 2], (float)T1[2, 3],
                           0, 0, 0, 1
                           );
            var Griptoframe6 = new Matrix4x4();

            Matrix4x4.Invert(fame6ToGrip, out Griptoframe6);//逆矩阵
            Matrix4x4.Invert(huashu_moduan, out huashu_moduan_ni);//逆矩阵

            //Matrix4x4 RT = T2 * huashu_moduan_ni* tool2GripMatrix * Griptoframe6;//得到6个矩阵相乘的最终位姿   4*4矩阵
            Matrix4x4 RT = T2 * huashu_moduan_ni * Griptoframe6;
            //Matrix4x4 RT = T2 * huashu_moduan_ni ;
            double[,] T = new double[4, 4];
            T[0, 0] = RT.M11;
            T[0, 1] = RT.M12;
            T[0, 2] = RT.M13;
            T[0, 3] = RT.M14;
            T[1, 0] = RT.M21;
            T[1, 1] = RT.M22;
            T[1, 2] = RT.M23;
            T[1, 3] = RT.M24;
            T[2, 0] = RT.M31;
            T[2, 1] = RT.M32;
            T[2, 2] = RT.M33;
            T[2, 3] = RT.M34;
            T[3, 0] = RT.M41;
            T[3, 1] = RT.M42;
            T[3, 2] = RT.M43;
            T[3, 3] = RT.M44;


            //单位mm
            double d1 = 456;
            double d4 = 410.69093;
            double d2 = 0;
            double d3 = 0;
            double d5 = 0;
            double d6 = 0;

            double a1 = 59.69338;
            double a2 = 440.64619;
            double a3 = 34.6563;
            double a4 = 0;
            double a5 = 0;
            double a6 = 0;

            double beta = Math.PI / 2;
            int k1 = 1;
            int k2 = 1;
            int k3 = 1;

            var px = T[0, 3];
            var py = T[1, 3];
            var pz = T[2, 3];   //% 位置信息
                                // % % % ax = T1(1, 3); ay = T1(2, 3); az = T1(3, 3);
                                //% % % ox = T1(1, 2); oy = T1(2, 2); oz = T1(3, 2);
                                //% % % nx = T1(1, 1); ny = T1(2, 1); nz = T1(3, 1);
                                //% px  py反正切 夹角
            var q1 = Math.Atan2(-px, py) + Math.Atan2(k1, 0); //36页  2-37

            // double px = 360 - 360 * Math.Pow(t, 2);
            //double py = 3 * Math.Pow(6 * Math.Pow(t, 2) - 20, 2) + 2 * (6 * Math.Pow(t, 2) - 20) - 560;
            //以下为角度1通用表达式
            //string px_str = "360 - 360 * t^2";
            //string py_str = "3 *(6*t^2-20)^2+2*(6*t^2-20)-560";
            string t_str = "t";
            string pxd_str;
            string pyd_str;
            double pxd = SymbolDerivation.symbolDerivation(FunctionExpressionx, t_str, endpose.t, out pxd_str); //一阶倒数
            double pyd = SymbolDerivation.symbolDerivation(FunctionExpressiony, t_str, endpose.t, out pyd_str); //一阶倒数
            string pxdd_str;
            string pydd_str;
            double pxdd = SymbolDerivation.symbolDerivation(pxd_str, t_str, endpose.t, out pxdd_str); //二阶倒数
            double pydd = SymbolDerivation.symbolDerivation(pyd_str, t_str, endpose.t, out pydd_str); //二阶倒数


            if (Math.Sqrt(Math.Pow(px, 2.0) + Math.Pow(py, 2.0)) < 0.000006)//每次出现，k值均需变号
            {
                SolveQFlag = !SolveQFlag;
                TChange.Add(endpose.t);

            }
            if (SolveQFlag == false)
            {
                q1 = Math.Atan2(-pxd, pyd) + Math.Atan2(-k1, 0); //调用一阶表达式
                if (Math.Sqrt(Math.Pow(pxd, 2.0) + Math.Pow(pyd, 2.0)) < 0.000006)
                    q1 = Math.Atan2(-pxdd, pydd) + Math.Atan2(-k1, 0);//调用二阶表达式
            }

            //以下为实验曲线角度1的表达式
            //double pxdd = -720;
            //double pydd = 1296 * Math.Pow(endpose.t, 2) - 1416;//1296*t^2-1416
            //if (Math.Sqrt(Math.Pow(px, 2.0) + Math.Pow(py, 2.0)) < 0.000006)//每次出现，k值均需变号
            //    SolveQFlag = !SolveQFlag;
            //if (SolveQFlag == false)
            //    q1 = Math.Atan2(-pxdd, pydd) + Math.Atan2(-k1, 0); //新解析式


            //% 对应P26页限制条件
            if (q1 < -Math.PI)
                q1 = q1 + 2 * Math.PI;
            else if (q1 > Math.PI)
                q1 = q1 - 2 * Math.PI;
            //Console.WriteLine("q1if: " + q1);
            //  % % % % % % a2 ^ 2 + 2 * cos(q3) * a2 * a3 + 2 * sin(q3) * a2 * d4 + a3 ^ 2 + d4 ^ 2 = (pn1 - a1) ^ 2 + (C1) ^ 2
            var pn1 = px * Math.Cos(q1) + py * Math.Sin(q1) - a1;
            // Console.WriteLine("pn1: " + pn1);
            var C1 = d1 - pz;
            // Console.WriteLine("C1: " + C1);
            var C3 = (Math.Pow(pn1, 2.0) + Math.Pow(C1, 2.0) - Math.Pow(a2, 2.0) - Math.Pow(a3, 2.0) - Math.Pow(d4, 2.0)) / (2 * a2);
            //Console.WriteLine("C3: " + C3);
            var D3 = Math.Pow(a3, 2.0) + Math.Pow(d4, 2.0) - Math.Pow(C3, 2.0);
            // Console.WriteLine("D3: " + D3);

            if (D3 < 0)
                D3 = 0;
            //int DD = 111;//% 标志位
            var q3 = Math.Atan2(k2 * Math.Sqrt(D3), C3) - Math.Atan2(d4, a3);//36页  2-39

            string Q31_str = "";
            double Q3d = 0;
            Q3d = SymbolDerivation.symbolDerivation(Q3String, "t", endpose.t, out Q31_str); //一阶倒数
            if (Q3d > 1000) //速度极大 则需要编号
            {
                SolveQFlag3 = !SolveQFlag3;
                TChange.Add(endpose.t);
            }
            if (SolveQFlag == false)
            {
                q3 = Math.Atan2(-k2 * Math.Sqrt(D3), C3) - Math.Atan2(d4, a3);//36页  2-39
            }


            //Console.WriteLine("q3: " + q3);

            //% % (d4 * cos(q3) + a3 * sin(q3)) * cos(q2) + (a3 * cos(q3) - d4 * sin(q3) + a2) * sin(q2) = pn1
            //% % (d4 * sin(q3) - a3 * cos(q3) - a2) * cos(q2) + (d4 * cos(q3) + a3 * sin(q3)) * sin(q2) = C1

            var a = (d4 * Math.Cos(q3) + a3 * Math.Sin(q3));   //  % hm1
            var b = (a3 * Math.Cos(q3) - d4 * Math.Sin(q3) + a2); // % hm2
            var c = -b;  //% -hm2
            var d = a;   //% hm1
            var e = pn1; //% 差值
            var f = C1;  //%
            var s2 = (a * f - c * e) / (a * d - b * c); // % m
            var c2 = (e * d - b * f) / (a * d - b * c);
            var q2 = Math.Atan2(s2, c2);//37页  2-40
            //Console.WriteLine("q2: " + q2);
            //% % (-cos(q2 + q3) * (ox * cos(q1) * sin(beta) + oy * sin(beta) * sin(q1)) - oz * sin(q2 + q3) * sin(beta)) * c6 + (-cos(q2 + q3) * (nx * cos(q1) * sin(beta) + ny * sin(beta) * sin(q1)) - nz * sin(q2 + q3) * sin(beta)) * s6 = cos(beta) - (+cos(q2 + q3) * (ax * cos(beta) * cos(q1) + ay * cos(beta) * sin(q1)) + az * sin(q2 + q3) * cos(beta));
            //double[] n = T(1:3, 1);// % 第1列   n向量
            var nx = T[0, 0];
            var ny = T[1, 0];
            var nz = T[2, 0];

            var ox = T[0, 1];
            var oy = T[1, 1];
            var oz = T[2, 1];

            var ax = T[0, 2];
            var ay = T[1, 2];
            var az = T[2, 2];
            //-oz    -30*sin(q2+q3)) t1,A6(1),t2,A6(2).......拟合 t^2+3t+2 
            var A6 = -oz * Math.Sin(q2 + q3) + Math.Cos(q2 + q3) * Math.Cos(q1) * ox + Math.Cos(q2 + q3) * Math.Sin(q1) * oy;
            var B6 = -nz * Math.Sin(q2 + q3) + Math.Cos(q2 + q3) * Math.Cos(q1) * nx + Math.Cos(q2 + q3) * Math.Sin(q1) * ny;
            double q6 = 0;
            double A61 = 0;
            double B61 = 0;
            double A62 = 0;
            double B62 = 0;
            q6 = Math.Atan2(k3, 0) + Math.Atan2(B6, A6);//37页  2-41
            string A61_str = "";
            string B61_str = "";
            string A62_str;
            string B62_str;

            if (Math.Sqrt(Math.Pow(A6, 2.0) + Math.Pow(B6, 2.0)) < 0.000006)//每次出现，k值均需变号
            {
                SolveQFlag6 = !SolveQFlag6;
                TChange.Add(endpose.t);
            }
            if (SolveQFlag6 == false)
            {
                A61 = SymbolDerivation.symbolDerivation(A6String, "t", endpose.t, out A61_str); //一阶倒数
                B61 = SymbolDerivation.symbolDerivation(B6String, "t", endpose.t, out B61_str); //一阶倒数
                q6 = Math.Atan2(-k3, 0) + Math.Atan2(B61, A61); //调用一阶表达式 表示q6

                if (Math.Sqrt(Math.Pow(B61, 2.0) + Math.Pow(A61, 2.0)) < 0.000006) //如果一阶平方和约为0
                {
                    A62 = SymbolDerivation.symbolDerivation(A61_str, "t", endpose.t, out A62_str); //二阶倒数
                    B62 = SymbolDerivation.symbolDerivation(B61_str, "t", endpose.t, out B62_str); //二阶倒数
                    q6 = Math.Atan2(-k3, 0) + Math.Atan2(B62, A62);//调用二阶表达式
                }
            }


            //if (Math.Sqrt(Math.Pow(A6, 2.0) + Math.Pow(B6, 2.0)) < 0.000006)//每次出现，k值均需变号
            //{
            //    SolveQFlag6 = !SolveQFlag6;

            //    if (A6String!="")
            //    {
            //        A61 = PathClass.symbolDerivation(A6String,"t", endpose.t, out A61_str); //一阶倒数
            //        B61 = PathClass.symbolDerivation(B6String, "t",endpose.t, out B61_str); //一阶倒数
            //    }

            //        q6 = Math.Atan2(-k3, 0) + Math.Atan2(B61, A61);//37页  2-41
            //}
            //if (SolveQFlag6 == false)
            //{
            //    q6 = Math.Atan2(k3, 0) + Math.Atan2(B6, A6);//37页  2-41
            //    if (Math.Sqrt(Math.Pow(A61, 2.0) + Math.Pow(B61, 2.0)) < 0.000006)
            //    {
            //        if (A6String != "")
            //        {
            //            A62 = PathClass.symbolDerivation(A61_str, "t",endpose.t,out A62_str); //二阶倒数
            //            B62 = PathClass.symbolDerivation(B61_str, "t",endpose.t, out B62_str); //二阶倒数
            //        }
            //            q6 = Math.Atan2(-k3, 0) + Math.Atan2(B62, A62);//调用二阶表达式
            //    }
            //}

            //Console.WriteLine("q6: " + q6);
            //% P26约束条件
            if (q6 < -Math.PI)
                q6 = q6 + 2 * Math.PI;
            else if (q6 > Math.PI)
                q6 = q6 - 2 * Math.PI;
            //Console.WriteLine("q6if: " + q6);

            var A4 = (ox * Math.Cos(q6) + nx * Math.Sin(q6)) * Math.Sin(q1) - (oy * Math.Cos(q6) + ny * Math.Sin(q6)) * Math.Cos(q1);
            var B4 = (ox * Math.Cos(q6) + nx * Math.Sin(q6)) * (-Math.Cos(q1) * Math.Sin(q2 + q3)) - (oy * Math.Cos(q6) + ny * Math.Sin(q6)) * (Math.Sin(q1) * Math.Sin(q2 + q3)) - (oz * Math.Cos(q6) + nz * Math.Sin(q6)) * (Math.Cos(q2 + q3));
            var C4 = -Math.Sin(beta);
            var q4 = Math.Atan2(0, C4) + Math.Atan2(B4, A4);//37页  2-43
            //Console.WriteLine("q4: " + q4);
            if (q4 > Math.PI)
                q4 = q4 - 2 * Math.PI;
            //Console.WriteLine("q4if: " + q4);

            var A5 = Math.Cos(q2 + q3) * Math.Cos(q1) * (ox * Math.Sin(q6) - nx * Math.Cos(q6)) + Math.Cos(q2 + q3) * Math.Sin(q1) * (oy * Math.Sin(q6) - ny * Math.Cos(q6)) - Math.Sin(q2 + q3) * (oz * Math.Sin(q6) - nz * Math.Cos(q6));
            var B5 = Math.Sin(q2 + q3) * az - Math.Cos(q2 + q3) * Math.Cos(q1) * ax - Math.Cos(q2 + q3) * Math.Sin(q1) * ay;
            var C5 = Math.Sin(beta);
            var q5 = Math.Atan2(0, C5) + Math.Atan2(B5, A5);//38页  2-45
            //Console.WriteLine("q5: " + q5);
            double[] Q = { q1, q2, q3, q4, q5, q6 };
            var position = new Position(Q);  //Position 包含转角   和  位姿
            var pnew = FKNewRad(Q);           // 返回X  Y  Z  Rx   Ry   Rz     欧拉角
            position.Pose = pnew;
            position.Pose.t = endpose.t;
            stopwatch.Stop();

            // 获取并输出经过的时间
            TimeSpan elapsed = stopwatch.Elapsed;
            tik += elapsed.TotalMilliseconds;
            return position;
            //Q = [q1 q2 q3 q4 q5 q6];
        }

        public static double[] Q3Solve(CartesianPosition endpose, Matrix4x4? Grip2Tool = null)
        {
            Matrix4x4 Grip2ToolMatrix = Grip2Tool ?? Matrix4x4.Identity; //

            Matrix4x4 tool2GripMatrix = new Matrix4x4();
            Matrix4x4 huashu_moduan_ni = new Matrix4x4();
            Matrix4x4.Invert(Grip2ToolMatrix, out tool2GripMatrix);

            var T1 = BasicAlgorithm.CatsionPositionToTMatrix(endpose); //输入末端位姿   得到7个矩阵相乘的4*4矩阵
            //T1 mm
            Matrix4x4 T2 = new Matrix4x4(
                           (float)T1[0, 0], (float)T1[0, 1], (float)T1[0, 2], (float)T1[0, 3],
                           (float)T1[1, 0], (float)T1[1, 1], (float)T1[1, 2], (float)T1[1, 3],
                           (float)T1[2, 0], (float)T1[2, 1], (float)T1[2, 2], (float)T1[2, 3],
                           0, 0, 0, 1
                           );
            var Griptoframe6 = new Matrix4x4();

            Matrix4x4.Invert(fame6ToGrip, out Griptoframe6);//逆矩阵
            Matrix4x4.Invert(huashu_moduan, out huashu_moduan_ni);//逆矩阵

            //Matrix4x4 RT = T2 * huashu_moduan_ni* tool2GripMatrix * Griptoframe6;//得到6个矩阵相乘的最终位姿   4*4矩阵
            Matrix4x4 RT = T2 * huashu_moduan_ni * Griptoframe6;
            //Matrix4x4 RT = T2 * huashu_moduan_ni ;
            double[,] T = new double[4, 4];
            T[0, 0] = RT.M11;
            T[0, 1] = RT.M12;
            T[0, 2] = RT.M13;
            T[0, 3] = RT.M14;
            T[1, 0] = RT.M21;
            T[1, 1] = RT.M22;
            T[1, 2] = RT.M23;
            T[1, 3] = RT.M24;
            T[2, 0] = RT.M31;
            T[2, 1] = RT.M32;
            T[2, 2] = RT.M33;
            T[2, 3] = RT.M34;
            T[3, 0] = RT.M41;
            T[3, 1] = RT.M42;
            T[3, 2] = RT.M43;
            T[3, 3] = RT.M44;


            //单位mm
            double d1 = 456;
            double d4 = 410.69093;
            double d2 = 0;
            double d3 = 0;
            double d5 = 0;
            double d6 = 0;

            double a1 = 59.69338;
            double a2 = 440.64619;
            double a3 = 34.6563;
            double a4 = 0;
            double a5 = 0;
            double a6 = 0;

            double beta = Math.PI / 2;
            int k1 = 1;
            int k2 = 1;
            int k3 = 1;

            var px = T[0, 3];
            var py = T[1, 3];
            var pz = T[2, 3];   //% 位置信息
                                // % % % ax = T1(1, 3); ay = T1(2, 3); az = T1(3, 3);
                                //% % % ox = T1(1, 2); oy = T1(2, 2); oz = T1(3, 2);
                                //% % % nx = T1(1, 1); ny = T1(2, 1); nz = T1(3, 1);
                                //% px  py反正切 夹角
            var q1 = Math.Atan2(-px, py) + Math.Atan2(k1, 0); //36页  2-37

            // double px = 360 - 360 * Math.Pow(t, 2);
            //double py = 3 * Math.Pow(6 * Math.Pow(t, 2) - 20, 2) + 2 * (6 * Math.Pow(t, 2) - 20) - 560;
            //以下为角度1通用表达式
            //string px_str = "360 - 360 * t^2";
            //string py_str = "3 *(6*t^2-20)^2+2*(6*t^2-20)-560";
            string t_str = "t";
            string pxd_str;
            string pyd_str;
            double pxd = SymbolDerivation.symbolDerivation(FunctionExpressionx, t_str, endpose.t, out pxd_str); //一阶倒数
            double pyd = SymbolDerivation.symbolDerivation(FunctionExpressiony, t_str, endpose.t, out pyd_str); //一阶倒数
            string pxdd_str;
            string pydd_str;
            double pxdd = SymbolDerivation.symbolDerivation(pxd_str, t_str, endpose.t, out pxdd_str); //二阶倒数
            double pydd = SymbolDerivation.symbolDerivation(pyd_str, t_str, endpose.t, out pydd_str); //二阶倒数

            if (Math.Sqrt(Math.Pow(px, 2.0) + Math.Pow(py, 2.0)) < 0.000006)//每次出现，k值均需变号
                SolveQFlag = !SolveQFlag;
            if (SolveQFlag == false)
            {
                q1 = Math.Atan2(-pxd, pyd) + Math.Atan2(-k1, 0); //调用一阶表达式
                if (Math.Sqrt(Math.Pow(pxd, 2.0) + Math.Pow(pyd, 2.0)) < 0.000006)
                    q1 = Math.Atan2(-pxdd, pydd) + Math.Atan2(-k1, 0);//调用二阶表达式
            }

            //以下为实验曲线角度1的表达式
            //double pxdd = -720;
            //double pydd = 1296 * Math.Pow(endpose.t, 2) - 1416;//1296*t^2-1416
            //if (Math.Sqrt(Math.Pow(px, 2.0) + Math.Pow(py, 2.0)) < 0.000006)//每次出现，k值均需变号
            //    SolveQFlag = !SolveQFlag;
            //if (SolveQFlag == false)
            //    q1 = Math.Atan2(-pxdd, pydd) + Math.Atan2(-k1, 0); //新解析式


            //% 对应P26页限制条件
            if (q1 < -Math.PI)
                q1 = q1 + 2 * Math.PI;
            else if (q1 > Math.PI)
                q1 = q1 - 2 * Math.PI;
            //Console.WriteLine("q1if: " + q1);
            //  % % % % % % a2 ^ 2 + 2 * cos(q3) * a2 * a3 + 2 * sin(q3) * a2 * d4 + a3 ^ 2 + d4 ^ 2 = (pn1 - a1) ^ 2 + (C1) ^ 2
            var pn1 = px * Math.Cos(q1) + py * Math.Sin(q1) - a1;
            // Console.WriteLine("pn1: " + pn1);
            var C1 = d1 - pz;
            // Console.WriteLine("C1: " + C1);
            var C3 = (Math.Pow(pn1, 2.0) + Math.Pow(C1, 2.0) - Math.Pow(a2, 2.0) - Math.Pow(a3, 2.0) - Math.Pow(d4, 2.0)) / (2 * a2);
            //Console.WriteLine("C3: " + C3);
            var D3 = Math.Pow(a3, 2.0) + Math.Pow(d4, 2.0) - Math.Pow(C3, 2.0);
            // Console.WriteLine("D3: " + D3);

            if (D3 < 0)
                D3 = 0;
            //int DD = 111;//% 标志位

            var q3 = Math.Atan2(k2 * Math.Sqrt(D3), C3) - Math.Atan2(d4, a3);//
            //string Q31_str = "";
            //double Q3d = 0;
            //Q3d = PathClass.symbolDerivation(Q3String, "t", endpose.t, out Q31_str); //一阶倒数
            //if (Q3d > 1000) //速度极大 则需要编号
            //    SolveQFlag3 = !SolveQFlag3;
            //if (SolveQFlag == false)
            //{
            //    q3 = Math.Atan2(-k2 * Math.Sqrt(D3), C3) - Math.Atan2(d4, a3);//36页  2-39
            //}
            double[] result1 = new double[1];
            result1[0] = q3;


            return result1;
        }

        public static double[] A6B6Solve(CartesianPosition endpose, Matrix4x4? Grip2Tool = null)
        {
            Matrix4x4 Grip2ToolMatrix = Grip2Tool ?? Matrix4x4.Identity; //

            Matrix4x4 tool2GripMatrix = new Matrix4x4();
            Matrix4x4 huashu_moduan_ni = new Matrix4x4();
            Matrix4x4.Invert(Grip2ToolMatrix, out tool2GripMatrix);

            var T1 = BasicAlgorithm.CatsionPositionToTMatrix(endpose); //输入末端位姿   得到7个矩阵相乘的4*4矩阵
            //T1 mm
            Matrix4x4 T2 = new Matrix4x4(
                           (float)T1[0, 0], (float)T1[0, 1], (float)T1[0, 2], (float)T1[0, 3],
                           (float)T1[1, 0], (float)T1[1, 1], (float)T1[1, 2], (float)T1[1, 3],
                           (float)T1[2, 0], (float)T1[2, 1], (float)T1[2, 2], (float)T1[2, 3],
                           0, 0, 0, 1
                           );
            var Griptoframe6 = new Matrix4x4();

            Matrix4x4.Invert(fame6ToGrip, out Griptoframe6);//逆矩阵
            Matrix4x4.Invert(huashu_moduan, out huashu_moduan_ni);//逆矩阵

            //Matrix4x4 RT = T2 * huashu_moduan_ni* tool2GripMatrix * Griptoframe6;//得到6个矩阵相乘的最终位姿   4*4矩阵
            Matrix4x4 RT = T2 * huashu_moduan_ni * Griptoframe6;
            //Matrix4x4 RT = T2 * huashu_moduan_ni ;
            double[,] T = new double[4, 4];
            T[0, 0] = RT.M11;
            T[0, 1] = RT.M12;
            T[0, 2] = RT.M13;
            T[0, 3] = RT.M14;
            T[1, 0] = RT.M21;
            T[1, 1] = RT.M22;
            T[1, 2] = RT.M23;
            T[1, 3] = RT.M24;
            T[2, 0] = RT.M31;
            T[2, 1] = RT.M32;
            T[2, 2] = RT.M33;
            T[2, 3] = RT.M34;
            T[3, 0] = RT.M41;
            T[3, 1] = RT.M42;
            T[3, 2] = RT.M43;
            T[3, 3] = RT.M44;


            //单位mm
            double d1 = 456;
            double d4 = 410.69093;
            double d2 = 0;
            double d3 = 0;
            double d5 = 0;
            double d6 = 0;

            double a1 = 59.69338;
            double a2 = 440.64619;
            double a3 = 34.6563;
            double a4 = 0;
            double a5 = 0;
            double a6 = 0;

            double beta = Math.PI / 2;
            int k1 = 1;
            int k2 = 1;
            int k3 = 1;

            var px = T[0, 3];
            var py = T[1, 3];
            var pz = T[2, 3];   //% 位置信息
                                // % % % ax = T1(1, 3); ay = T1(2, 3); az = T1(3, 3);
                                //% % % ox = T1(1, 2); oy = T1(2, 2); oz = T1(3, 2);
                                //% % % nx = T1(1, 1); ny = T1(2, 1); nz = T1(3, 1);
                                //% px  py反正切 夹角
            var q1 = Math.Atan2(-px, py) + Math.Atan2(k1, 0); //36页  2-37

            // double px = 360 - 360 * Math.Pow(t, 2);
            //double py = 3 * Math.Pow(6 * Math.Pow(t, 2) - 20, 2) + 2 * (6 * Math.Pow(t, 2) - 20) - 560;
            //以下为角度1通用表达式
            //string px_str = "360 - 360 * t^2";
            //string py_str = "3 *(6*t^2-20)^2+2*(6*t^2-20)-560";
            string t_str = "t";
            string pxd_str;
            string pyd_str;
            double pxd = SymbolDerivation.symbolDerivation(FunctionExpressionx, t_str, endpose.t, out pxd_str); //一阶倒数
            double pyd = SymbolDerivation.symbolDerivation(FunctionExpressiony, t_str, endpose.t, out pyd_str); //一阶倒数
            string pxdd_str;
            string pydd_str;
            double pxdd = SymbolDerivation.symbolDerivation(pxd_str, t_str, endpose.t, out pxdd_str); //二阶倒数
            double pydd = SymbolDerivation.symbolDerivation(pyd_str, t_str, endpose.t, out pydd_str); //二阶倒数

            if (Math.Sqrt(Math.Pow(px, 2.0) + Math.Pow(py, 2.0)) < 0.000006)//每次出现，k值均需变号
                SolveQFlag = !SolveQFlag;
            if (SolveQFlag == false)
            {
                q1 = Math.Atan2(-pxd, pyd) + Math.Atan2(-k1, 0); //调用一阶表达式
                if (Math.Sqrt(Math.Pow(pxd, 2.0) + Math.Pow(pyd, 2.0)) < 0.000006)
                    q1 = Math.Atan2(-pxdd, pydd) + Math.Atan2(-k1, 0);//调用二阶表达式
            }

            //以下为实验曲线角度1的表达式
            //double pxdd = -720;
            //double pydd = 1296 * Math.Pow(endpose.t, 2) - 1416;//1296*t^2-1416
            //if (Math.Sqrt(Math.Pow(px, 2.0) + Math.Pow(py, 2.0)) < 0.000006)//每次出现，k值均需变号
            //    SolveQFlag = !SolveQFlag;
            //if (SolveQFlag == false)
            //    q1 = Math.Atan2(-pxdd, pydd) + Math.Atan2(-k1, 0); //新解析式


            //% 对应P26页限制条件
            if (q1 < -Math.PI)
                q1 = q1 + 2 * Math.PI;
            else if (q1 > Math.PI)
                q1 = q1 - 2 * Math.PI;
            //Console.WriteLine("q1if: " + q1);
            //  % % % % % % a2 ^ 2 + 2 * cos(q3) * a2 * a3 + 2 * sin(q3) * a2 * d4 + a3 ^ 2 + d4 ^ 2 = (pn1 - a1) ^ 2 + (C1) ^ 2
            var pn1 = px * Math.Cos(q1) + py * Math.Sin(q1) - a1;
            // Console.WriteLine("pn1: " + pn1);
            var C1 = d1 - pz;
            // Console.WriteLine("C1: " + C1);
            var C3 = (Math.Pow(pn1, 2.0) + Math.Pow(C1, 2.0) - Math.Pow(a2, 2.0) - Math.Pow(a3, 2.0) - Math.Pow(d4, 2.0)) / (2 * a2);
            //Console.WriteLine("C3: " + C3);
            var D3 = Math.Pow(a3, 2.0) + Math.Pow(d4, 2.0) - Math.Pow(C3, 2.0);
            // Console.WriteLine("D3: " + D3);

            if (D3 < 0)
                D3 = 0;
            //int DD = 111;//% 标志位

            var q3 = Math.Atan2(k2 * Math.Sqrt(D3), C3) - Math.Atan2(d4, a3);//
            //Console.WriteLine("q3: " + q3);
            string Q31_str = "";
            double Q3d = 0;
            Q3d = SymbolDerivation.symbolDerivation(Q3String, "t", endpose.t, out Q31_str); //一阶倒数
            if (Q3d > 1000) //速度极大 则需要编号
                SolveQFlag3 = !SolveQFlag3;
            if (SolveQFlag == false)
            {
                q3 = Math.Atan2(-k2 * Math.Sqrt(D3), C3) - Math.Atan2(d4, a3);//36页  2-39
            }

            //% % (d4 * cos(q3) + a3 * sin(q3)) * cos(q2) + (a3 * cos(q3) - d4 * sin(q3) + a2) * sin(q2) = pn1
            //% % (d4 * sin(q3) - a3 * cos(q3) - a2) * cos(q2) + (d4 * cos(q3) + a3 * sin(q3)) * sin(q2) = C1

            var a = (d4 * Math.Cos(q3) + a3 * Math.Sin(q3));   //  % hm1
            var b = (a3 * Math.Cos(q3) - d4 * Math.Sin(q3) + a2); // % hm2
            var c = -b;  //% -hm2
            var d = a;   //% hm1
            var e = pn1; //% 差值
            var f = C1;  //%
            var s2 = (a * f - c * e) / (a * d - b * c); // % m
            var c2 = (e * d - b * f) / (a * d - b * c);
            var q2 = Math.Atan2(s2, c2);//37页  2-40
            //Console.WriteLine("q2: " + q2);
            //% % (-cos(q2 + q3) * (ox * cos(q1) * sin(beta) + oy * sin(beta) * sin(q1)) - oz * sin(q2 + q3) * sin(beta)) * c6 + (-cos(q2 + q3) * (nx * cos(q1) * sin(beta) + ny * sin(beta) * sin(q1)) - nz * sin(q2 + q3) * sin(beta)) * s6 = cos(beta) - (+cos(q2 + q3) * (ax * cos(beta) * cos(q1) + ay * cos(beta) * sin(q1)) + az * sin(q2 + q3) * cos(beta));
            //double[] n = T(1:3, 1);// % 第1列   n向量
            var nx = T[0, 0];
            var ny = T[1, 0];
            var nz = T[2, 0];

            var ox = T[0, 1];
            var oy = T[1, 1];
            var oz = T[2, 1];

            var ax = T[0, 2];
            var ay = T[1, 2];
            var az = T[2, 2];
            //-oz    -30*sin(q2+q3)) t1,A6(1),t2,A6(2).......拟合 t^2+3t+2 
            var A6 = -oz * Math.Sin(q2 + q3) + Math.Cos(q2 + q3) * Math.Cos(q1) * ox + Math.Cos(q2 + q3) * Math.Sin(q1) * oy;
            var B6 = -nz * Math.Sin(q2 + q3) + Math.Cos(q2 + q3) * Math.Cos(q1) * nx + Math.Cos(q2 + q3) * Math.Sin(q1) * ny;
            double[] result = new double[2];
            result[0] = A6;
            result[1] = B6;
            return result;
        }
        public static double[,] Q3Product(List<CartesianPosition> cartesianposition)
        {
            double[,] result = new double[1, cartesianposition.Count];
            for (int i = 0; i < cartesianposition.Count; i++)
            {
                var temp = Q3Solve(cartesianposition[i].mmTom());
                result[0, i] = temp[0];
            }
            List<double> list_t = new List<double>();
            foreach (var position in cartesianposition)
            {
                list_t.Add(position.t);
            }
            var ts = list_t.ToArray();
            var Q31 = new double[ts.Length];
            // 使用 LINQ 获取指定行的数据

            int columnsCount = result.GetLength(1); // 获取列数
            Q31 = Enumerable.Range(0, columnsCount)
                                       .Select(col => result[0, col])
                                       .ToArray();

            double[] Q3Coefficients = Fit.Polynomial(ts, Q31, 3);
            Q3Value = new double[ts.Length];
            Q3String = SymbolDerivation.FuncStringProduce(Q3Coefficients, 0, 3);

            int j = 0;
            foreach (var item in ts)
            {
                Q3Value[j] = SymbolDerivation.StringExpretion(Q3String, item);
               
                j++;
            }
            return result;
        }
        public static double[,] QProduct(List<CartesianPosition> cartesianposition)
        {
            double[,] result = new double[6, cartesianposition.Count];
            for (int i = 0; i < cartesianposition.Count; i++)
            {
                var temp = IK_New(cartesianposition[i].mmTom());
                result[0, i] = temp.Joints.Joints.ToArray()[0];
                result[1, i] = temp.Joints.Joints.ToArray()[1];
                result[2, i] = temp.Joints.Joints.ToArray()[2];
                result[3, i] = temp.Joints.Joints.ToArray()[3];
                result[4, i] = temp.Joints.Joints.ToArray()[4];
                result[5, i] = temp.Joints.Joints.ToArray()[5];
            }
            List<double> list_t = new List<double>();
            foreach (var position in cartesianposition)
            {
                list_t.Add(position.t);
            }
            var ts = list_t.ToArray();
            var Q1 = new double[ts.Length];
            // 使用 LINQ 获取指定行的数据

            int columnsCount = result.GetLength(1); // 获取列数
            Q1 = Enumerable.Range(0, columnsCount)
                                       .Select(col => result[0, col])
                                       .ToArray();
            var Q2 = new double[ts.Length];
            // 使用 LINQ 获取指定行的数据
            Q2 = Enumerable.Range(0, columnsCount)
                                       .Select(col => result[1, col])
                                       .ToArray();
            var Q3 = new double[ts.Length];
            // 使用 LINQ 获取指定行的数据
            Q3 = Enumerable.Range(0, columnsCount)
                                       .Select(col => result[2, col])
                                       .ToArray();
            var Q4 = new double[ts.Length];
            // 使用 LINQ 获取指定行的数据
            Q4 = Enumerable.Range(0, columnsCount)
                                       .Select(col => result[3, col])
                                       .ToArray();
            var Q5 = new double[ts.Length];
            // 使用 LINQ 获取指定行的数据
            Q5 = Enumerable.Range(0, columnsCount)
                                       .Select(col => result[4, col])
                                       .ToArray();
            var Q6 = new double[ts.Length];
            // 使用 LINQ 获取指定行的数据
            Q6 = Enumerable.Range(0, columnsCount)
                                       .Select(col => result[5, col])
                                       .ToArray();

            double[] Q1Coefficients = Fit.Polynomial(ts, Q1, 3);
            NewIK.Q1Value = new double[ts.Length];
            NewIK.Q1String = SymbolDerivation.FuncStringProduce(Q1Coefficients, 0, 3);

            double[] Q2Coefficients = Fit.Polynomial(ts, Q2, 3);
            NewIK.Q2Value = new double[ts.Length];
            NewIK.Q2String = SymbolDerivation.FuncStringProduce(Q2Coefficients, 0, 3);

            double[] Q3Coefficients = Fit.Polynomial(ts, Q3, 3);
            NewIK.Q3Value = new double[ts.Length];
            NewIK.Q3String = SymbolDerivation.FuncStringProduce(Q3Coefficients, 0, 3);

            double[] Q4Coefficients = Fit.Polynomial(ts, Q4, 3);
            NewIK.Q4Value = new double[ts.Length];
            NewIK.Q4String = SymbolDerivation.FuncStringProduce(Q4Coefficients, 0, 3);

            double[] Q5Coefficients = Fit.Polynomial(ts, Q5, 3);
            NewIK.Q5Value = new double[ts.Length];
            NewIK.Q5String = SymbolDerivation.FuncStringProduce(Q5Coefficients, 0, 3);

            double[] Q6Coefficients = Fit.Polynomial(ts, Q6, 3);
            NewIK.Q6Value = new double[ts.Length];
            NewIK.Q6String = SymbolDerivation.FuncStringProduce(Q6Coefficients, 0, 3);

            int j = 0;
            foreach (var item in ts)
            {
               Q1Value[j] = SymbolDerivation.StringExpretion(Q1String, item);
               Q2Value[j] = SymbolDerivation.StringExpretion(Q2String, item);
               Q3Value[j] = SymbolDerivation.StringExpretion(Q3String, item);
               Q4Value[j] = SymbolDerivation.StringExpretion(Q4String, item);
               Q5Value[j] = SymbolDerivation.StringExpretion(Q5String, item);
               Q6Value[j] = SymbolDerivation.StringExpretion(Q6String, item);
                j++;
            }

            return result;
        }


        public static double[,] A6B6Product(List<CartesianPosition> cartesianposition)
        {
            double[,] result = new double[2, cartesianposition.Count];
            for (int i = 0; i < cartesianposition.Count; i++)
            {
                var temp = A6B6Solve(cartesianposition[i].mmTom());
                result[0, i] = temp[0];
                result[1, i] = temp[1];
            }
            return result;
        }
        public static int[] TChanglistToIndex(List<double> t, List<CartesianPosition> CartesianPositions)
        {
            int[] index = new int[t.Count];
            for (int i = 0; i < t.Count; i++)
            {
                index[i] = CartesianPositions.FindIndex(p => Math.Abs(p.t - t[i]) <= 1e-6);
            }
            return index;
        }

    }
}
