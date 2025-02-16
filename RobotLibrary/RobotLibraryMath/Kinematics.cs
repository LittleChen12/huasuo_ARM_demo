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
    public abstract class Kinematics
    {
        public Algorithm basicalgortihm;
        //最优解，path.IK找出最优解，返回最优解的位姿
        public double[] joints = new double[6]; 
        public CartesianPosition point;
        public double[,] R = new double[3, 3];
        //定义4*4 位姿矩阵
        public double[,] T = new double[4, 4];
        public Matrix4x4 Rt = new Matrix4x4(
             1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1
        );
        public static Matrix4x4 fame6ToGrip = new Matrix4x4(
            1, 0, 0, 0f,
            0, -1, 0, 0f,
            0, 0, -1, -0.095f,
            0, 0, 0, 1.0f
            );

        public Kinematics()
        {
            basicalgortihm = new Algorithm();
            point = new CartesianPosition();
        }
        public abstract void KinematicsInit();
        public abstract CartesianPosition FkAngle(double[] angle);
        public abstract CartesianPosition FkRad(double[] rad);
        public abstract CartesianPosition FkAngle(double[] angles, Matrix4x4? Grip2Tool = null);
        public abstract Position Ik(CartesianPosition position);


    }

}
