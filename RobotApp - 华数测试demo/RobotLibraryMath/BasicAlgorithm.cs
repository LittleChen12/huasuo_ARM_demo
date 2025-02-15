using System;
using System.Collections.Generic;
using System.Numerics;
using System.Windows.Controls;

namespace RobotLibraryAlgorithm
{
    public class BasicAlgorithm
    {//做继承20240516


        /****************************************************************************
        *@name	  : ZXToRotMatrix
        *@brief	  : B坐标系 基于A（基）坐标系描述的Z轴和X轴（某坐标系的 Z轴与X轴） 转换为旋转矩阵
        *@param	  Vector3 z  : z轴向量
        *@param	  Vector3 x  : x轴向量
        *@return  double[3,3] : 旋转矩阵
        ****************************************************************************/
        public virtual double[,] ZXToRotMatrix(Vector3 z, Vector3 x)
        {
            // 确保 z 是单位向量
            z = Vector3.Normalize(z);
            x = Vector3.Normalize(x);

            Vector3 y = Vector3.Cross(z, x);
            x = Vector3.Cross(y, z);

            //// 构建旋转矩阵 (x y z)
            double[,] R = new double[3, 3];
            R[0, 0] = x.X;
            R[1, 0] = x.Y;
            R[2, 0] = x.Z;
            R[0, 1] = y.X;
            R[1, 1] = y.Y;
            R[2, 1] = y.Z;
            R[0, 2] = z.X;
            R[1, 2] = z.Y;
            R[2, 2] = z.Z;

            return R;
        }


        /****************************************************************************
        *@name	  : ZThetaToRotMatrix
        *@brief	  : B坐标系 基于A（基）坐标系描述下的Z轴和theta参数 转换为旋转矩阵
        *@param	  Vector3 z  : z轴向量
        *@param	  float theta  : 确定了Z轴后，x轴与y轴的方向由theta参数控制（单位：rad）
        *@return  double[3,3] : 旋转矩阵
        ****************************************************************************/
        public virtual double[,] ZThetaToRotMatrix(Vector3 z, float theta)
        {
            // 确保 z 是单位向量
            z = Vector3.Normalize(z);

            Vector3 baseX;
            if (Vector3.Dot(z, new Vector3(1, 0, 0)) > 0.999f || Vector3.Dot(z, new Vector3(-1, 0, 0)) > 0.999f)
            {
                // 如果 z 接近 x 轴，改用 y 轴
                baseX = new Vector3(0, 1, 0);
            }
            else
            {
                // 通常情况下，使用 x 轴
                baseX = new Vector3(1, 0, 0);
            }

            // 计算旋转后的 x 向量
            float cosTheta = (float)Math.Cos(theta);
            float sinTheta = (float)Math.Sin(theta);
            Vector3 x = cosTheta * baseX + sinTheta * Vector3.Cross(z, baseX);
            x = Vector3.Normalize(x); // 单位化 x

            // 计算 y 向量
            Vector3 y = Vector3.Cross(z, x);
            y = Vector3.Normalize(y); // 单位化 y

            // 构建旋转矩阵 (x y z)
            double[,] R = new double[3, 3];
            R[0, 0] = x.X;
            R[1, 0] = x.Y;
            R[2, 0] = x.Z;
            R[0, 1] = y.X;
            R[1, 1] = y.Y;
            R[2, 1] = y.Z;
            R[0, 2] = z.X;
            R[1, 2] = z.Y;
            R[2, 2] = z.Z;

            return R;
        }



        /****************************************************************************
        *@name	  : ZThetaToX
        *@brief	  : B坐标系 基于A（基）坐标系描述下的Z轴和theta参数 计算出与Z向量垂直的X向量
        *@param	  Vector3 z  : z轴向量
        *@param	  float theta  : 确定了Z轴后，x轴的方向由theta参数控制（单位：rad）
        *@return  Vector3 : 与Z向量垂直的 X向量
        ****************************************************************************/

        public virtual Vector3 ZThetaToX(Vector3 z, float theta)
        {
            // 确保 z 是单位向量
            z = Vector3.Normalize(z);

            Vector3 baseX;
            if (Vector3.Dot(z, new Vector3(1, 0, 0)) > 0.999f)
            {
                // 如果 z 接近 x 轴，改用 y 轴
                baseX = new Vector3(0, 1, 0);
            }
            else
            {
                // 通常情况下，使用 x 轴
                baseX = new Vector3(1, 0, 0);
            }

            // 计算旋转后的 x 向量
            float cosTheta = (float)Math.Cos(theta);
            float sinTheta = (float)Math.Sin(theta);
            Vector3 x = cosTheta * baseX + sinTheta * Vector3.Cross(z, baseX);
            x = Vector3.Normalize(x); // 单位化 x

            return x;
        }
        /****************************************************************************
        *@name	  : RxyzToRotMatrix
        *@brief	  : 欧拉角转旋转矩阵
        *@param	  double RX, double RY, double RZ  : 3D欧拉角
        *@return  double[3,3] : 旋转矩阵
        ****************************************************************************/
        public static double[,] RxyzToRotMatrix(double RX, double RY, double RZ)
        {           
            double[,] R = new double[3, 3];


            double sa = Math.Sin(RZ);
            double ca = Math.Cos(RZ);
            double sb = Math.Sin(RY);
            double cb = Math.Cos(RY);
            double sg = Math.Sin(RX);
            double cg = Math.Cos(RX);


            R[0, 0] = cb * ca;
            R[0, 1] = ca * sb * sg - sa * cg;
            R[0, 2] = ca * sb * cg + sg * sa;
            R[1, 0] = sa * cb;
            R[1, 1] = sg * sb * sa + cg * ca;
            R[1, 2] = sa * sb * cg - ca * sg;
            R[2, 0] = -sb;
            R[2, 1] = cb * sg;
            R[2, 2] = cb * cg;

            return R;

        }

        /****************************************************************************
        *@name	  : RotMatrixToRxyz
        *@brief	  : 旋转矩阵转欧拉角
        *@param	  double[3,3] : 旋转矩阵
        *@return   double[3] : 3D欧拉角 Rx, Ry, Rz;
        ****************************************************************************/
        public static double[] RotMatrixToRxyz(double[,] R)
        {
            double Rx, Ry, Rz;
            if (R[2, 0] != 1 && R[2, 0] != -1)
            {
                Ry = -Math.Asin(R[2, 0]);
                Rx = Math.Atan2(R[2, 1] / Math.Cos(Ry), R[2, 2] / Math.Cos(Ry));
                Rz = Math.Atan2(R[1, 0] / Math.Cos(Ry), R[0, 0] / Math.Cos(Ry));
            }
            else
            {
                Rz = 0;
                if (R[2, 0] == -1)
                {
                    Ry = Math.PI / 2;
                    Rx = Rz + Math.Atan2(R[0, 1], R[0, 2]);
                }
                else
                {
                    Ry = -Math.PI / 2;
                    Rx = -Rz + Math.Atan2(-R[0, 1], -R[0, 2]);
                }
            }

            return new double[] { Rx, Ry, Rz };

        }

        /****************************************************************************
        *@name	  : RadToAngle
        *@brief	  : 弧度转角度
        *@param	  double[] joints : 弧度值 0~ 2pi
        *@return   double[] : 3D欧拉角 Rx, Ry, Rz; 0~360
        ****************************************************************************/
       public virtual double[] RadToAngle(double[] joints)
        {
            double[] angle = new double[joints.GetLength(0)];
            for (int i = 0; i < joints.GetLength(0); i++)
            {
                angle[i] = joints[i] * 180 / Math.PI;
            }
            return angle;
        }

        /****************************************************************************
        *@name	  : AngleToRad
        *@brief	  : 角度转弧度
        *@param	  double[] joints : 3D欧拉角 Rx, Ry, Rz; 0~360
        *@return   double[] : 弧度值 0~ 2pi
        ****************************************************************************/
        public virtual double[] AngleToRad(double[] joints)
        {
            double[] rad = new double[joints.GetLength(0)];
            for (int i = 0; i < joints.GetLength(0); i++)
            {
                rad[i] = joints[i] * Math.PI /180;
            }
            return rad;
        }



        /****************************************************************************
        *@name	  : MatrixMultiply
        *@brief	  : A B 矩阵相乘
        *@param	  double[,] A B: 输入矩阵
        *@return   double[,] :  A B 矩阵相乘后的结果
        ****************************************************************************/
        public virtual double[,] MatrixMultiply(double[,] A, double[,] B)
        {
            int rowA = A.GetLength(0);
            int colA = A.GetLength(1);
            int rowB = B.GetLength(0);
            int colB = B.GetLength(1);
            double[,] C = new double[rowA, colB];
            if (colA != rowB)
            {
                Console.WriteLine("矩阵A的列数不等于矩阵B的行数，不能相乘");
                return null;
            }
            else
            {
                for (int i = 0; i < rowA; i++)
                {
                    for (int j = 0; j < colB; j++)
                    {
                        for (int k = 0; k < colA; k++)
                        {
                            C[i, j] += A[i, k] * B[k, j];
                        }
                    }
                }
                return C;
            }
        }


        /****************************************************************************
         * 四元数插值参数类
         ****************************************************************************/
        public class OrientInpParam
        {
            public double[,] Rs = new double[3, 3];
            public double[,] Re = new double[3, 3];
            public double[,] R = new double[3, 3];
            public double[] omg = new double[3];
            public double theta;
            public double[,] Ri = new double[3, 3];
            public double thetai;
            public int InpFlag;
        }

        public virtual void InitialOrientInpParam(double[,] Rs, double[,] Re, OrientInpParam Param)
        {
            double[,] InvR = new double[3, 3];
            Array.Copy(Rs, Param.Rs, Rs.Length);
            Array.Copy(Re, Param.Re, Re.Length);
            RotInv(Rs, InvR);
            MatrixMult(InvR, Re, Param.R);
            RotToAxisAng(Param.R, Param.omg, out Param.theta);
            Array.Copy(Param.R, Param.Ri, Param.R.Length);
            Param.thetai = 0.0;
            Param.InpFlag = 1;
        }

        public static void QuaternionOrientInp(OrientInpParam Param, double dtheta, double[,] Ri1)
        {
            double[] q = new double[4];
            double[,] R = new double[3, 3];
            Param.InpFlag = 2;
            Param.thetai += dtheta;
            if (Param.thetai >= Param.theta)
            {
                Param.thetai = Param.theta;
                Param.InpFlag = 3;
            }
            AxisAngToQuaternion(Param.omg, Param.thetai, q);
            QuaternionToRot(q, R);
            MatrixMult(Param.Rs, R, Ri1);
            Array.Copy(Ri1, Param.Ri, Ri1.Length);
        }

        // Function implementations
        public static void RotInv(double[,] R, double[,] InvR)
        {
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    InvR[i, j] = R[j, i];
                }
            }
        }

        public static void MatrixMult(double[,] A, double[,] B, double[,] result)
        {
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    result[i, j] = 0;
                    for (int k = 0; k < 3; k++)
                    {
                        result[i, j] += A[i, k] * B[k, j];
                    }
                }
            }
        }


        public static void RotToAxisAng(double[,] R, double[] omg, out double theta)
        {
            double cosTheta = (R[0, 0] + R[1, 1] + R[2, 2] - 1) / 2;
            double sinTheta = Math.Sqrt((R[2, 1] - R[1, 2]) * (R[2, 1] - R[1, 2]) +
                                        (R[0, 2] - R[2, 0]) * (R[0, 2] - R[2, 0]) +
                                        (R[1, 0] - R[0, 1]) * (R[1, 0] - R[0, 1])) / 2;

            theta = Math.Atan2(sinTheta, cosTheta);

            if (sinTheta == 0)
            {
                omg[0] = 1.0;
                omg[1] = 0.0;
                omg[2] = 0.0;
            }
            else
            {
                omg[0] = (R[2, 1] - R[1, 2]) / (2 * sinTheta);
                omg[1] = (R[0, 2] - R[2, 0]) / (2 * sinTheta);
                omg[2] = (R[1, 0] - R[0, 1]) / (2 * sinTheta);
            }
        }

        public static void AxisAngToQuaternion(double[] omg, double theta, double[] q)
        {
            double halfTheta = theta / 2;
            double sinHalfTheta = Math.Sin(halfTheta);

            q[0] = Math.Cos(halfTheta);
            q[1] = omg[0] * sinHalfTheta;
            q[2] = omg[1] * sinHalfTheta;
            q[3] = omg[2] * sinHalfTheta;
        }

        public static void QuaternionToRot(double[] q, double[,] R)
        {
            double q0 = q[0];
            double q1 = q[1];
            double q2 = q[2];
            double q3 = q[3];

            R[0, 0] = 1 - 2 * (q2 * q2 + q3 * q3);
            R[0, 1] = 2 * (q1 * q2 - q0 * q3);
            R[0, 2] = 2 * (q0 * q2 + q1 * q3);
            R[1, 0] = 2 * (q1 * q2 + q0 * q3);
            R[1, 1] = 1 - 2 * (q1 * q1 + q3 * q3);
            R[1, 2] = 2 * (q2 * q3 - q0 * q1);
            R[2, 0] = 2 * (q1 * q3 - q0 * q2);
            R[2, 1] = 2 * (q0 * q1 + q2 * q3);
            R[2, 2] = 1 - 2 * (q1 * q1 + q2 * q2);
        }



    }




}
