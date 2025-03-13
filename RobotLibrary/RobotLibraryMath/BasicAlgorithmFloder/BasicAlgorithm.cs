using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Windows.Controls;
using System.Windows.Media.Media3D;

namespace RobotLibraryAlgorithm
{
    public abstract class BasicAlgorithm
    {
        /****************************************************************************
        *@name	  : ZXToRotMatrix
        *@brief	  : B����ϵ ����A����������ϵ������Z���X�ᣨĳ����ϵ�� Z����X�ᣩ ת��Ϊ��ת����
        *@param	  Vector3 z  : z������
        *@param	  Vector3 x  : x������
        *@return  double[3,3] : ��ת����
        ****************************************************************************/
        public virtual double[,] ZXToRotMatrix(Vector3 z, Vector3 x)
        {
            // ȷ�� z �ǵ�λ����
            z = Vector3.Normalize(z);
            x = Vector3.Normalize(x);

            Vector3 y = Vector3.Cross(z, x);
            x = Vector3.Cross(y, z);

            //// ������ת���� (x y z)
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
        *@brief	  : B����ϵ ����A����������ϵ�����µ�Z���theta���� ת��Ϊ��ת����
        *@param	  Vector3 z  : z������
        *@param	  float theta  : ȷ����Z���x����y��ķ�����theta�������ƣ���λ��rad��
        *@return  double[3,3] : ��ת����
        ****************************************************************************/
        public virtual double[,] ZThetaToRotMatrix(Vector3 z, float theta)
        {
            // ȷ�� z �ǵ�λ����
            z = Vector3.Normalize(z);

            Vector3 baseX;
            if (Vector3.Dot(z, new Vector3(1, 0, 0)) > 0.999f || Vector3.Dot(z, new Vector3(-1, 0, 0)) > 0.999f)
            {
                // ��� z �ӽ� x �ᣬ���� y ��
                baseX = new Vector3(0, 1, 0);
            }
            else
            {
                // ͨ������£�ʹ�� x ��
                baseX = new Vector3(1, 0, 0);
            }

            // ������ת��� x ����
            float cosTheta = (float)Math.Cos(theta);
            float sinTheta = (float)Math.Sin(theta);
            Vector3 x = cosTheta * baseX + sinTheta * Vector3.Cross(z, baseX);
            x = Vector3.Normalize(x); // ��λ�� x

            // ���� y ����
            Vector3 y = Vector3.Cross(z, x);
            y = Vector3.Normalize(y); // ��λ�� y

            // ������ת���� (x y z)
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
        *@brief	  : B����ϵ ����A����������ϵ�����µ�Z���theta���� �������Z������ֱ��X����
        *@param	  Vector3 z  : z������
        *@param	  float theta  : ȷ����Z���x��ķ�����theta�������ƣ���λ��rad��
        *@return  Vector3 : ��Z������ֱ�� X����
        ****************************************************************************/
        public virtual Vector3 ZThetaToX(Vector3 z, float theta)
        {
            // ȷ�� z �ǵ�λ����
            z = Vector3.Normalize(z);

            Vector3 baseX;
            if (Vector3.Dot(z, new Vector3(1, 0, 0)) > 0.999f)
            {
                // ��� z �ӽ� x �ᣬ���� y ��
                baseX = new Vector3(0, 1, 0);
            }
            else
            {
                // ͨ������£�ʹ�� x ��
                baseX = new Vector3(1, 0, 0);
            }

            // ������ת��� x ����
            float cosTheta = (float)Math.Cos(theta);
            float sinTheta = (float)Math.Sin(theta);
            Vector3 x = cosTheta * baseX + sinTheta * Vector3.Cross(z, baseX);
            x = Vector3.Normalize(x); // ��λ�� x

            return x;
        }
        /****************************************************************************
        *@name	  : RxyzToRotMatrix
        *@brief	  : ŷ����ת��ת����
        *@param	  double RX, double RY, double RZ  : 3Dŷ����
        *@return  double[3,3] : ��ת����
        ****************************************************************************/
        public virtual double[,] RxyzToRotMatrix(double RX, double RY, double RZ)
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
        *@brief	  : ��ת����תŷ����
        *@param	  double[3,3] : ��ת����
        *@return   double[3] : 3Dŷ���� Rx, Ry, Rz;
        ****************************************************************************/
        public virtual double[] RotMatrixToRxyz(double[,] R)
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
        *@name	  : RotMatrixToRxyz
        *@brief	  : ��ת����תŷ����
        *@param	 Matrix4x4  : ��ת����
        *@return   double[3] : 3Dŷ���� Rx, Ry, Rz;
        ****************************************************************************/
        public virtual double[] RotMatrixToRxyz(Matrix4x4 R)
        {
            double Rx, Ry, Rz;

            if (R.M31 != 1 && R.M31 != -1)
            {
                Ry = -Math.Asin(R.M31);
                Rx = Math.Atan2(R.M32 / Math.Cos(Ry), R.M33 / Math.Cos(Ry));
                Rz = Math.Atan2(R.M21 / Math.Cos(Ry), R.M11 / Math.Cos(Ry));
            }
            else
            {
                Rz = 0;
                if (R.M31 == -1)
                {
                    Ry = Math.PI / 2;
                    Rx = Rz + Math.Atan2(R.M12, R.M13);
                }
                else
                {
                    Ry = -Math.PI / 2;
                    Rx = -Rz + Math.Atan2(-R.M12, -R.M13);
                }
            }

            return new double[] { Rx, Ry, Rz };

        }

        /****************************************************************************
        *@name	  : RadToAngle
        *@brief	  : ����ת�Ƕ�
        *@param	  double[] joints : ����ֵ 0~ 2pi
        *@return   double[] : 3Dŷ���� Rx, Ry, Rz; 0~360
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
        *@brief	  : �Ƕ�ת����
        *@param	  double[] joints : 3Dŷ���� Rx, Ry, Rz; 0~360
        *@return   double[] : ����ֵ 0~ 2pi
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
        *@brief	  : A B �������
        *@param	  double[,] A B: �������
        *@return   double[,] :  A B ������˺�Ľ��
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
                Console.WriteLine("����A�����������ھ���B���������������");
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

       
        //���Բ�ֵ
        public virtual double LerpValue(double startVal, double endVal, double t)
        {
            return startVal * (1 - t) + endVal * t;
        }
        //�Ƕ����������Բ�ֵ
        public virtual double LerpAngle(double startAngle, double endAngle, double t)
        {
            double delta = WrapAngle(endAngle - startAngle);
            return WrapAngle(startAngle + delta * t);
        }

        // ȷ��angle��[0,2pi]
        private double WrapAngle(double angle)
        {
            while (angle > Math.PI) angle -= 2 * Math.PI;
            while (angle < -Math.PI) angle += 2 * Math.PI;
            return angle;
        }
        /*
         * ����������[0,2pi]������ ���Բ�ֵ
         * ���룺
         * start����ʼλ�ˣ���λ��rad��
         * end ����ֹλ�ˣ���λ��rad��
         * t ��[0,1] ȷ����ֵ��λ��
         * 
         * �����CartesianPosition  ��ֵ���λ������
         */
        public virtual double[] Lerp(JointPosition start, JointPosition end, double t)
        {//
            double[] ret = new double[start.Joints.Count()];
            for (int i = 0; i < start.Joints.Count(); i++)
            {
                ret[i] = LerpAngle(start.Joints[i], end.Joints[i], t);
            }
            return ret;
        }
        /*
         * ����������[0,2pi]������ ���Բ�ֵ
         * ���룺
         * start����ʼ�ѿ���λ��
         * end ����ֹ�ѿ���λ��
         * t ��[0,1] ȷ����ֵ��λ��
         * 
         * �����CartesianPosition  ��ֵ���λ������
         */
        public virtual CartesianPosition Lerp(CartesianPosition start, CartesianPosition end, double t)
        {//20240516  �����Բ�ֵ    ע�ͺ��߼���  �ɶ��Բ�
            return new CartesianPosition(
                LerpValue(start.Point.X, end.Point.X, t),
                LerpValue(start.Point.Y, end.Point.Y, t),
                LerpValue(start.Point.Z, end.Point.Z, t),
                LerpAngle(start.Rx, end.Rx, t),
                LerpAngle(start.Ry, end.Ry, t),
                LerpAngle(start.Rz, end.Rz, t));
        }


        public CartesianPosition TransToPose(Matrix4x4 _RT)
        {
            CartesianPosition pose = new CartesianPosition();
            var R = RotMatrixToRxyz(_RT);
            pose.Point = new Point3D(_RT.M14, _RT.M24, _RT.M34);
            pose.Rx = R[0];
            pose.Ry = R[1];
            pose.Rz = R[2];
            return pose;
        }
        public Matrix4x4 ToTrans(CartesianPosition position)
        {
            Matrix4x4 _RT = new Matrix4x4();
            double[,] Rrot = new double[3, 3];
            Rrot = RxyzToRotMatrix(position.Rx, position.Ry, position.Rz);

            _RT.M11 = (float)Rrot[0, 0];
            _RT.M12 = (float)Rrot[0, 1];
            _RT.M13 = (float)Rrot[0, 2];
            _RT.M21 = (float)Rrot[1, 0];
            _RT.M22 = (float)Rrot[1, 1];
            _RT.M23 = (float)Rrot[1, 2];
            _RT.M31 = (float)Rrot[2, 0];
            _RT.M32 = (float)Rrot[2, 1];
            _RT.M33 = (float)Rrot[2, 2];

            _RT.M14 = (float)position.Point.X;
            _RT.M24 = (float)position.Point.Y;
            _RT.M34 = (float)position.Point.Z;

            _RT.M41 = 0;
            _RT.M42 = 0;
            _RT.M43 = 0;
            _RT.M44 = 1;

            return _RT;
        }



        /****************************************************************************
         * ��Ԫ����ֵ������
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

        public virtual void QuaternionOrientInp(OrientInpParam Param, double dtheta, double[,] Ri1)
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
        public virtual void RotInv(double[,] R, double[,] InvR)
        {
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    InvR[i, j] = R[j, i];
                }
            }
        }

        public virtual void MatrixMult(double[,] A, double[,] B, double[,] result)
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


        public virtual void RotToAxisAng(double[,] R, double[] omg, out double theta)
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

        public virtual void AxisAngToQuaternion(double[] omg, double theta, double[] q)
        {
            double halfTheta = theta / 2;
            double sinHalfTheta = Math.Sin(halfTheta);

            q[0] = Math.Cos(halfTheta);
            q[1] = omg[0] * sinHalfTheta;
            q[2] = omg[1] * sinHalfTheta;
            q[3] = omg[2] * sinHalfTheta;
        }
        public virtual void QuaternionToRot(double[] q, double[,] R)
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
