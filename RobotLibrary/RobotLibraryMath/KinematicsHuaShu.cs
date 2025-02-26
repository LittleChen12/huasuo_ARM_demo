using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

namespace RobotLibraryAlgorithm.KinematicsAlgorithm
{
    public class KinematicsHuaShu:Kinematics
    {
       
        //最优解，path.IK找出最优解，返回最优解的位姿
        private double[] joints = new double[6];
        //所有解 findnearest 找出最优解，
        private double[] jointss = new double[48];
        private int num;
        private CartesianPosition points;
        private double[] eerot = new double[9];
        private double[] eetrans = new double[3];
        private double[,] Rrot = new double[3, 3];
        //定义4*4 位姿矩阵
        //public double[,] T = new double[4, 4];

        private Matrix4x4 Rt = new Matrix4x4(
             1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1
        );

        private static Matrix4x4 fame6ToGrip = new Matrix4x4(
            1, 0, 0, 0f,
            0, -1, 0, 0f,
            0, 0, -1, -0.095f,
            0, 0, 0, 1.0f
            );
      
        [STAThread]
        [DllImport("ikfast_607.dll")]
        extern static void SolveFK(double[] joints, double[] eerot, double[] eetrans);
        [DllImport("ikfast_607.dll")]
        extern static bool SolveIK(double[] eerot, double[] eetrans, double[] joints, ref int num);

        public KinematicsHuaShu(Position pose)
        {
            basicalgortihm = new Algorithm();
            points = new CartesianPosition();
            points = new CartesianPosition(pose.Pose.Point.X,pose.Pose.Point.Y,pose.Pose.Point.Z,pose.Pose.Rx,pose.Pose.Ry, pose.Pose.Rz);
            ToRotTrans();
        }

        public KinematicsHuaShu(JointPosition Joints)
        {
            basicalgortihm = new Algorithm();
            points = new CartesianPosition();
            joints = Joints.Joints.ToArray();
            ToRotTrans();
        }

        public KinematicsHuaShu()
        {
            basicalgortihm = new Algorithm();
            points = new CartesianPosition();
        }

        private void ToRrot()
        {
            Rrot[0, 0] = eerot[0];
            Rrot[0, 1] = eerot[1];
            Rrot[0, 2] = eerot[2];
            Rrot[1, 0] = eerot[3];
            Rrot[1, 1] = eerot[4];
            Rrot[1, 2] = eerot[5];
            Rrot[2, 0] = eerot[6];
            Rrot[2, 1] = eerot[7];
            Rrot[2, 2] = eerot[8];
        }

        private void RrotToEerot()
        {
            eerot[0] = Rrot[0, 0];
            eerot[1] = Rrot[0, 1];
            eerot[2] = Rrot[0, 2];
            eerot[3] = Rrot[1, 0];
            eerot[4] = Rrot[1, 1];
            eerot[5] = Rrot[1, 2];
            eerot[6] = Rrot[2, 0];
            eerot[7] = Rrot[2, 1];
            eerot[8] = Rrot[2, 2];
        }

        // 将T 转换为eerot 和eetrans
        private void Matrix4x4ToRotTrans(Matrix4x4 T)
        {
            eerot[0] = T.M11;
            eerot[1] = T.M12;
            eerot[2] = T.M13;
            eetrans[0] = T.M14;
            eerot[3] = T.M21;
            eerot[4] = T.M22;
            eerot[5] = T.M23;
            eetrans[1] = T.M24;
            eerot[6] = T.M31;
            eerot[7] = T.M32;
            eerot[8] = T.M33;
            eetrans[2] = T.M34;

        }


        private void ToMatrix()
        {
            //赋值到RT
            Rt.M11 = (float)eerot[0];
            Rt.M12 = (float)eerot[1];
            Rt.M13 = (float)eerot[2];
            Rt.M14 = (float)eetrans[0];
            Rt.M21 = (float)eerot[3];
            Rt.M22 = (float)eerot[4];
            Rt.M23 = (float)eerot[5];
            Rt.M24 = (float)eetrans[1];
            Rt.M31 = (float)eerot[6];
            Rt.M32 = (float)eerot[7];
            Rt.M33 = (float)eerot[8];
            Rt.M34 = (float)eetrans[2];
            Rt.M41 = 0;
            Rt.M42 = 0;
            Rt.M43 = 0;
            Rt.M44 = 1;

        }

        // eerot 旋转矩阵转 ZYX 

        //旋转矩阵转欧拉角
        public void ToPose()
        {
            ToRrot();
            double[] Rxyz = basicalgortihm.RotMatrixToRxyz(Rrot);
            points.Point = new Point3D(eetrans[0], eetrans[1], eetrans[2]);
            points.Rx = Rxyz[0];
            points.Ry = Rxyz[1];
            points.Rz = Rxyz[2];

        }


        //欧拉角转旋转矩阵
        private void ToRotTrans()
        {
            Rrot = basicalgortihm.RxyzToRotMatrix(points.Rx, points.Ry, points.Rz);
            this.RrotToEerot();
            eetrans[0] = points.Point.X;
            eetrans[1] = points.Point.Y;
            eetrans[2] = points.Point.Z;

        }



        public KinematicsHuaShu(CartesianPosition pose)
        {
            points = pose;
            ToRotTrans();
        }

        

        //输入Tool_T 为 Grip2Tool 即以末端坐标系为基描述手坐标系
        private static void SolveFK_GP7_T(double[] joints, double[] eerot, double[] eetrans, Matrix4x4? Grip2Tool = null)
        {

            Matrix4x4 Grip2toolMatrix = Grip2Tool ?? Matrix4x4.Identity;

            SolveFK(joints, eerot, eetrans);
            Matrix4x4 T = new Matrix4x4(
                            (float)eerot[0], (float)eerot[1], (float)eerot[2], (float)eetrans[0],
                            (float)eerot[3], (float)eerot[4], (float)eerot[5], (float)eetrans[1],
                            (float)eerot[6], (float)eerot[7], (float)eerot[8], (float)eetrans[2],
                            0, 0, 0, 1
                            );
            Matrix4x4 RT = T * fame6ToGrip * Grip2toolMatrix;
            eerot[0] = RT.M11;
            eerot[1] = RT.M12;
            eerot[2] = RT.M13;
            eetrans[0] = RT.M14;
            eerot[3] = RT.M21;
            eerot[4] = RT.M22;
            eerot[5] = RT.M23;
            eetrans[1] = RT.M24;
            eerot[6] = RT.M31;
            eerot[7] = RT.M32;
            eerot[8] = RT.M33;
            eetrans[2] = RT.M34;

        }


        //输入Tool_T 为 Grip2Tool 即以末端坐标系为基描述手坐标系的逆
        private static bool SolveIK_GP7_T(double[] eerot, double[] eetrans, double[] joints, ref int num, Matrix4x4? Grip2Tool = null)
        {
            Matrix4x4 Grip2ToolMatrix = Grip2Tool ?? Matrix4x4.Identity;
            Matrix4x4 tool2GripMatrix = new Matrix4x4();
            Matrix4x4.Invert(Grip2ToolMatrix, out tool2GripMatrix);

            Matrix4x4 T = new Matrix4x4(
                            (float)eerot[0], (float)eerot[1], (float)eerot[2], (float)eetrans[0],
                            (float)eerot[3], (float)eerot[4], (float)eerot[5], (float)eetrans[1],
                            (float)eerot[6], (float)eerot[7], (float)eerot[8], (float)eetrans[2],
                            0, 0, 0, 1
                            );
            var Griptoframe6 = new Matrix4x4();
            Matrix4x4.Invert(fame6ToGrip, out Griptoframe6);
            Matrix4x4 RT = T * tool2GripMatrix * Griptoframe6;
            eerot[0] = RT.M11;
            eerot[1] = RT.M12;
            eerot[2] = RT.M13;
            eetrans[0] = RT.M14;
            eerot[3] = RT.M21;
            eerot[4] = RT.M22;
            eerot[5] = RT.M23;
            eetrans[1] = RT.M24;
            eerot[6] = RT.M31;
            eerot[7] = RT.M32;
            eerot[8] = RT.M33;
            eetrans[2] = RT.M34;

            try
            {
                bool da = SolveIK(eerot, eetrans, joints, ref num);
                return da;

            }
            catch (Exception e)
            {
                Console.WriteLine("逆解报错：" + e.Message);
                return false;
            }



        }

        private void FK(Matrix4x4? Grip2Tool = null)
        {
            SolveFK_GP7_T(joints, eerot, eetrans, Grip2Tool);
            ToPose();
        }

        // 类外正解
        public  CartesianPosition FkRad(double[] joints, Matrix4x4? Grip2Tool = null)
        {

            KinematicsHuaShu kinematics = new KinematicsHuaShu();
            SolveFK_GP7_T(joints, kinematics.eerot, kinematics.eetrans, Grip2Tool);
             kinematics.ToPose();
            return kinematics.points;
        }
        /// <summary>
        /// 正解 单位为m
        /// </summary>
        /// <param name="joints"></param>
        /// <param name="Grip2Tool"></param>
        /// <returns></returns>
        public  override CartesianPosition FkAngle(double[] joints,Matrix4x4?Grip2Tool=null)
        {
            KinematicsHuaShu kinematics = new KinematicsHuaShu();
            joints = basicalgortihm.AngleToRad(joints);
            SolveFK_GP7_T(joints, kinematics.eerot, kinematics.eetrans, Grip2Tool);
            kinematics.ToPose();
            return kinematics.points;
        }

        public override CartesianPosition FkAngle(double[] joints)
        {
            KinematicsHuaShu kinematics = new KinematicsHuaShu();
            joints= basicalgortihm.AngleToRad(joints);
            Matrix4x4? Grip2Tool = null;
            SolveFK_GP7_T(joints, kinematics.eerot, kinematics.eetrans, Grip2Tool);
            kinematics.ToPose();
            return kinematics.points;
        }



        private void IK(Matrix4x4? Grip2Tool = null)
        {

            SolveIK_GP7_T(eerot, eetrans, jointss, ref num, Grip2Tool);

        }
        public void IK(JointPosition joints, Matrix4x4? Tool2Grip = null)
        {
            SolveIK_GP7_T(eerot, eetrans, jointss, ref num, Tool2Grip);
            KinematicsHuaShu.FindNearest(joints, this);
        }


        public static Position IK(JointPosition joints, CartesianPosition endpose, Matrix4x4? Grip2Tool = null)
        {
            KinematicsHuaShu pathClass = new KinematicsHuaShu(endpose);
            pathClass.ToRotTrans();
            SolveIK_GP7_T(pathClass.eerot, pathClass.eetrans, pathClass.jointss, ref pathClass.num, Grip2Tool);
            FindNearest(joints, pathClass);
            Position position = new Position(pathClass.joints, Grip2Tool);
          
            return position;

        }





        // 寻找离当前关节位置 最近的解
        static private void FindNearest(KinematicsHuaShu jointlast, KinematicsHuaShu jointnew)
        {
            double[] joints_next = new double[6];

            double temp = 0;
            double temp1 = 0;

            double[] distance = new double[8];
            for (int i = 0; i < jointnew.num; i++)
            {
                distance[i] = 0;
                for (int j = 0; j < 6; j++)
                {
                    if (jointlast.joints[j] < jointnew.jointss[i * 6 + j])
                    {
                        temp = jointnew.jointss[i * 6 + j] - jointlast.joints[j];
                        temp1 = jointlast.joints[j] + 2 * Math.PI - jointnew.jointss[i * 6 + j];
                    }
                    else
                    {
                        temp = jointlast.joints[j] - jointnew.jointss[i * 6 + j];
                        temp1 = jointnew.jointss[i * 6 + j] + 2 * Math.PI - jointlast.joints[j];
                    }

                    //temp，temp1将较小的值 加到distance[i]
                    if (temp > temp1)
                        temp = temp1;

                    distance[i] += temp * temp;
                }
            }

            double min = distance[0];
            int min_index = 0;
            for (int i = 1; i < jointnew.num; i++)
            {
                if (distance[i] < min)
                {
                    min = distance[i];
                    min_index = i;
                }
            }

            // 存入 joints_next 
            for (int i = 0; i < 6; i++)
            {
                jointnew.joints[i] = jointnew.jointss[min_index * 6 + i];
            }

        }

        // 寻找离当前关节位置 最近的解
        static private void FindNearest(JointPosition jointlast, KinematicsHuaShu jointnew)
        {
            double[] joints_next = new double[6];

            double temp = 0;
            double temp1 = 0;

            double[] distance = new double[8];
            for (int i = 0; i < jointnew.num; i++)
            {
                distance[i] = 0;
                for (int j = 0; j < 6; j++)
                {
                    if (jointlast.Joints[j] < jointnew.jointss[i * 6 + j])
                    {
                        temp = jointnew.jointss[i * 6 + j] - jointlast.Joints[j];
                        temp1 = jointlast.Joints[j] + 2 * Math.PI - jointnew.jointss[i * 6 + j];
                    }
                    else
                    {
                        temp = jointlast.Joints[j] - jointnew.jointss[i * 6 + j];
                        temp1 = jointnew.jointss[i * 6 + j] + 2 * Math.PI - jointlast.Joints[j];
                    }

                    //temp，temp1将较小的值 加到distance[i]
                    if (temp > temp1)
                        temp = temp1;

                    distance[i] += temp * temp;
                }
            }

            double min = distance[0];
            int min_index = 0;
            for (int i = 1; i < jointnew.num; i++)
            {
                if (distance[i] < min)
                {
                    min = distance[i];
                    min_index = i;
                }
            }

            // 存入 joints_next 
            for (int i = 0; i < 6; i++)
            {
                jointnew.joints[i] = jointnew.jointss[min_index * 6 + i];
            }

        }

        // 从num组逆解中找到离当前joints 欧式距离最近的解
        static private double[] FindNearest(double[] joints, double[] jointss, int num)
        {
            double[] joints_next = new double[6];

            double[] distance = new double[num];
            for (int i = 0; i < num; i++)
            {
                distance[i] = 0;
                for (int j = 0; j < 6; j++)
                {
                    distance[i] += (joints[j] - jointss[i * 6 + j]) * (joints[j] - jointss[i * 6 + j]);
                }
            }

            double min = distance[0];
            int min_index = 0;
            for (int i = 1; i < num; i++)
            {
                if (distance[i] < min)
                {
                    min = distance[i];
                    min_index = i;
                }
            }

            // 存入 joints_next 
            for (int i = 0; i < 6; i++)
            {
                joints_next[i] = jointss[min_index * 6 + i];
            }

            return joints_next;
        }

        public override void KinematicsInit()
        {
            throw new NotImplementedException();
        }

        //public override CartesianPosition FkAngle(double[] angle)
        //{
        //    throw new NotImplementedException();
        //}

        public override CartesianPosition FkRad(double[] rad)
        {
            throw new NotImplementedException();
        }

        public override Position Ik(CartesianPosition position)
        {
            throw new NotImplementedException();
        }

       

       
    }


}

