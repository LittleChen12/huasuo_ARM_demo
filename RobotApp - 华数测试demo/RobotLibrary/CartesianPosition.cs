using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace RobotLibrary
{

    // 笛卡尔空间位姿类
    public class CartesianPosition
    {
        private double x { get; set; }
        private double y { get; set; }
        private double z { get; set; }
        private double Rx { get; set; }
        private double Ry { get; set; }
        private double Rz { get; set; }


        public double X{ get { return x; } set { x = value; }}
        public double Y { get { return y; } set { y = value; } }
        public double Z { get { return z; } set { z = value; } }
        public double RX { get { return Rx; } set { Rx = value; } }
        public double RY { get { return Ry; } set { Ry = value; } }
        public double RZ { get { return Rz; } set { Rz = value; } }
        public double t;


        // 构造函数：分别赋值
        public CartesianPosition(double x_, double y_, double z_, double rx_, double ry_, double rz_)
        {
            x = x_;
            y = y_;
            z = z_;
            Rx = rx_;
            Ry = ry_;
            Rz = rz_;
            
        }
        //默认构造函数
        public CartesianPosition()
        {
            x = 0;
            y = 0;
            z = 0;
            Rx = 0;
            Ry = 0;
            Rz = 0;
        }
        // 构造函数：数组赋值
        public CartesianPosition(double[] pose)
        {
            x = pose[0];
            y = pose[1];
            z = pose[2];
            Rx = pose[3];
            Ry = pose[4];
            Rz = pose[5];
        }

        public CartesianPosition(double[] xyz, double[] Rxyz)
        {
            x = xyz[0];
            y = xyz[1];
            z = xyz[2];
            Rx = Rxyz[0];
            Ry = Rxyz[1];
            Rz = Rxyz[2];
        }


        //拷贝构造函数
        public CartesianPosition(CartesianPosition _pose)
        {
            x = _pose.X;
            y = _pose.Y;
            z = _pose.Z;
            Rx = _pose.RX;
            Ry = _pose.RY;
            Rz = _pose.RZ;
        }
        public CartesianPosition mmTom()
        {
            return new CartesianPosition(
            x/1000,
            y/1000,
            z/1000,
            Rx,
            Ry,
            Rz)
          ;    

        }

        public CartesianPosition mTomm()
        {
            return new CartesianPosition(
            x * 1000,
            y * 1000,
            z * 1000,
            Rx,
            Ry,
            Rz)
          ;

        }


        public Matrix4x4 ToTrans()
        {
            Matrix4x4 _RT = new Matrix4x4();
            double[,] Rrot = new double[3, 3];
            Rrot = BasicAlgorithm.RxyzToRotMatrix(RX,RY,RZ);


            _RT.M11 = (float)Rrot[0,0];
            _RT.M12 = (float)Rrot[0,1];
            _RT.M13 = (float)Rrot[0,2];
            _RT.M21 = (float)Rrot[1,0];
            _RT.M22 = (float)Rrot[1,1];
            _RT.M23 = (float)Rrot[1,2];
            _RT.M31 = (float)Rrot[2,0];
            _RT.M32 = (float)Rrot[2,1];
            _RT.M33 = (float)Rrot[2,2];

            _RT.M14 = (float)X;
            _RT.M24 = (float)Y;
            _RT.M34 = (float)Z;

            _RT.M41 = 0;
            _RT.M42 = 0;
            _RT.M43 = 0;
            _RT.M44 = 1;

            return _RT;
        }

        public CartesianPosition AngleToRad()
        {
            return new CartesianPosition(
            x,
            y,
            z,
            Rx / 180 * Math.PI,
            Ry / 180 * Math.PI,
            Rz / 180 * Math.PI);
        }





        /*
         * 功能描述：[0,2pi]的连续 线性插值
        * 输入：
        * start：起始位姿（单位：rad）
        * end ：终止位姿（单位：rad）
        * t ：[0,1] 确定插值的位置
        * 
        * 输出：CartesianPosition  插值后的位姿数据
        */
        public static CartesianPosition Lerp(CartesianPosition start, CartesianPosition end, double t)
        {//20240516  以线性插值    注释和逻辑性  可读性差
            return new CartesianPosition(
                LerpValue(start.X, end.X, t),
                LerpValue(start.Y, end.Y, t),
                LerpValue(start.Z, end.Z, t),
                LerpAngle(start.RX, end.RX, t),
                LerpAngle(start.RY, end.RY, t),
                LerpAngle(start.RZ, end.RZ, t));
        }

        


        // 从位姿矩阵转换为位姿
        public static CartesianPosition TransToPose(Matrix4x4 _RT)
        {
            CartesianPosition pose = new CartesianPosition();
            
         var R=   BasicAlgorithm.RotMatrixToRxyz(_RT);

            pose.X = _RT.M14;
            pose.Y = _RT.M24;
            pose.Z = _RT.M34;
            pose.RX = R[0];
            pose.RY = R[1];
            pose.RZ = R[2];
            return pose; 

        }



        //线性插值
        private static double LerpValue(double startVal, double endVal, double t)
        {
            return startVal * (1 - t) + endVal * t;
        }
        //角度连续的线性插值
        private static double LerpAngle(double startAngle, double endAngle, double t)
        {
            double delta = WrapAngle(endAngle - startAngle);
            return WrapAngle(startAngle + delta * t);
        }
        // 确保angle在[0,2pi]
        private static double WrapAngle(double angle)
        {
            while (angle > Math.PI) angle -= 2 * Math.PI;
            while (angle < -Math.PI) angle += 2 * Math.PI;
            return angle;
        }

        /*
         * 功能描述：三次样条插值，其中姿态插补采用四元数进行
        * 输入：
        * poses：输入点序列
        * num ：插值点数（每两点之间）
        * 
        * 输出：插值后的CartesianPosition序列
        */
        public static List<CartesianPosition> Cubic_spline_interpolation(List<CartesianPosition> poses, int num)
        {
            List<CartesianPosition> interpolatedPoses = new List<CartesianPosition>();
            double[,] Ri1 = new double[3, 3];

            BasicAlgorithm.OrientInpParam Param = new BasicAlgorithm.OrientInpParam();


            interpolatedPoses.Add(poses[0]);

            //循环访问list 里面的每一个pose
            for (int i = 1; i < poses.Count; i++)
            {
                //计算两个姿态之间的插值
                BasicAlgorithm.InitialOrientInpParam(BasicAlgorithm.RxyzToRotMatrix(poses[i - 1].RX, poses[i - 1].RY, poses[i - 1].RZ), BasicAlgorithm.RxyzToRotMatrix(poses[i].RX, poses[i].RY, poses[i].RZ), Param);

                double totalTheta = Param.theta; // Total rotation angle
                double dtheta = totalTheta / (num + 1); // Angle change per step

                for (int j = 0; j < num; j++)
                {
                    BasicAlgorithm.QuaternionOrientInp(Param, dtheta, Ri1);
                    double[] ddd = BasicAlgorithm.RotMatrixToRxyz(Param.Ri);
                    interpolatedPoses.Add(new CartesianPosition(0, 0, 0, ddd[0], ddd[1], ddd[2]));
                }
                interpolatedPoses.Add(poses[i]);
            }

            double[] dis = new double[poses.Count];

            double[] point_x = new double[poses.Count];
            double[] point_y = new double[poses.Count];
            double[] point_z = new double[poses.Count];

            double[] dislist = new double[(poses.Count - 1) * (num + 1) + 1];
            dis[0] = 0;
            dislist[0] = dis[0];

            for (int i = 1; i < poses.Count; i++)
            {
                double distance = Math.Sqrt(Math.Pow((poses[i].X - poses[i - 1].X) * 1, 2) + Math.Pow((poses[i].Y - poses[i - 1].Y) * 1, 2) + Math.Pow((poses[i].Z - poses[i - 1].Z) * 1, 2));
                dis[i] = distance + dis[i - 1];

                double dthe = distance / (num + 1);

                for (int j = 0; j < num; j++)
                {

                    dislist[(i - 1) * (num + 1) + j + 1] = dislist[(i - 1) * (num + 1) + j] + dthe;
                }
                dislist[(i - 1) * (num + 1) + num + 1] = dis[i];


            }
            for (int i = 0; i < poses.Count; i++)
            {
                point_x[i] = poses[i].X;
                point_y[i] = poses[i].Y;
                point_z[i] = poses[i].Z;
            }

            double[] interpolation_x = SplineInsertPoint(dis, point_x, dislist);
            double[] interpolation_Y = SplineInsertPoint(dis, point_y, dislist);
            double[] interpolation_Z = SplineInsertPoint(dis, point_z, dislist);

            for (int i = 0; i < dislist.Length; i++)
            {
                interpolatedPoses[i].X = interpolation_x[i];
                interpolatedPoses[i].Y = interpolation_Y[i];
                interpolatedPoses[i].Z = interpolation_Z[i];
            }

            return interpolatedPoses;

        }

        //参数说明：
        //dis: x轴 单调递增的数组
        //points: y轴数组
        //xs: 需要插值的点
        private static double[] SplineInsertPoint(double[] dis, double[] points, double[] xs)
        {
            int plength = points.Length;
            double[] h = new double[plength];
            double[] f = new double[plength];
            double[] l = new double[plength];
            double[] v = new double[plength];
            double[] g = new double[plength];

            for (int i = 0; i < plength - 1; i++)
            {
                h[i] = dis[i + 1] - dis[i];
                f[i] = (points[i + 1] - points[i]) / h[i];
            }

            for (int i = 1; i < plength - 1; i++)
            {
                l[i] = h[i] / (h[i - 1] + h[i]);
                v[i] = h[i - 1] / (h[i - 1] + h[i]);
                g[i] = 3 * (l[i] * f[i - 1] + v[i] * f[i]);
            }

            double[] b = new double[plength];
            double[] tem = new double[plength];
            double[] m = new double[plength];
            double f0 = (points[0] - points[1]) / (dis[0] - dis[1]);
            double fn = (points[plength - 1] - points[plength - 2]) / (dis[plength - 1] - dis[plength - 2]);

            b[1] = v[1] / 2;
            for (int i = 2; i < plength - 2; i++)
            {
                // Console.Write(" " + i);
                b[i] = v[i] / (2 - b[i - 1] * l[i]);
            }
            tem[1] = g[1] / 2;
            for (int i = 2; i < plength - 1; i++)
            {
                //Console.Write(" " + i);
                tem[i] = (g[i] - l[i] * tem[i - 1]) / (2 - l[i] * b[i - 1]);
            }
            m[plength - 2] = tem[plength - 2];
            for (int i = plength - 3; i > 0; i--)
            {
                //Console.Write(" " + i);
                m[i] = tem[i] - b[i] * m[i + 1];
            }
            m[0] = 3 * f[0] / 2.0;
            m[plength - 1] = fn;
            int xlength = xs.Length;
            double[] insertRes = new double[xlength];
            for (int i = 0; i < xlength; i++)
            {
                int j = 0;
                for (j = 0; j < plength; j++)
                {
                    if (xs[i] < dis[j])
                        break;
                }
                j = j - 1;
                Console.WriteLine(j);
                if (j == -1 || j == points.Length - 1)
                {
                    if (j == -1)
                        throw new Exception("插值下边界超出");
                    if (j == points.Length - 1 && xs[i] == dis[j])
                        insertRes[i] = points[j];
                    else
                        throw new Exception("插值下边界超出");
                }
                else
                {
                    double p1;
                    p1 = (xs[i] - dis[j + 1]) / (dis[j] - dis[j + 1]);
                    p1 = p1 * p1;
                    double p2; p2 = (xs[i] - dis[j]) / (dis[j + 1] - dis[j]);
                    p2 = p2 * p2;
                    double p3; p3 = p1 * (1 + 2 * (xs[i] - dis[j]) / (dis[j + 1] - dis[j])) * points[j] + p2 * (1 + 2 * (xs[i] - dis[j + 1]) / (dis[j] - dis[j + 1])) * points[j + 1];

                    double p4; p4 = p1 * (xs[i] - dis[j]) * m[j] + p2 * (xs[i] - dis[j + 1]) * m[j + 1];
                    //         Console.WriteLine(m[j] + " " + m[j + 1] + " " + j);
                    p4 = p4 + p3;
                    insertRes[i] = p4;
                    //Console.WriteLine("f(" + xs[i] + ")= " + p4);
                }

            }
            //Console.ReadLine();
            return insertRes;
        }






    }
}
