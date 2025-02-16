using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;


namespace RobotLibraryAlgorithm.InterPolation
{
    public class MoveInterPolation
    {
        private Algorithm algorithm;
        //private KinematicsAlgorithm.KinematicsAlgorithm KinematicsAlgorithm;
        public MoveInterPolation()
        {
            algorithm = new Algorithm();
            //KinematicsAlgorithm=new KinematicsAlgorithm.KinematicsAlgorithm();
        }
        // MoveL 功能函数
        // 参数：
        // startPos ：起始点位姿（包含关节位置和笛卡尔位姿）
        // endPose ：终止点位姿
        // DensityIndex: 输出点插值密度
        // Grip2Tool :工具安装位姿矩阵
        public List<Position> MoveL(Position startPos, CartesianPosition endPose, int numPoints, Matrix4x4? Grip2Tool = null)
        {
            List<Position> interpolatedPoses = new List<Position>();
            Position endposition = new Position();  
            endposition = KinematicsAlgorithm.KinematicsHuaShu. IK(startPos.Joints, endPose, Grip2Tool);
            Position temp = new Position(endposition);
            double[,] Ri1 = new double[3, 3];
            var Param = new Algorithm.OrientInpParam();
            algorithm.InitialOrientInpParam( algorithm. RxyzToRotMatrix(startPos.Pose.Rx, startPos.Pose.Ry, startPos.Pose.Rz), algorithm.RxyzToRotMatrix(endPose.Rx, endPose.Ry, endPose.Rz), Param);
            double totalTheta = Param.theta; // Total rotation angle
            double dtheta = totalTheta / (numPoints - 1); // Angle change per step
            for (int i = 0; i <= numPoints; i++)
            {
                double t = (double)i / numPoints;

                temp.Pose = algorithm.Lerp(startPos.Pose, endPose, t);

               algorithm. QuaternionOrientInp(Param, dtheta, Ri1);
                double[] ddd = algorithm.RotMatrixToRxyz(Param.Ri);
                temp.Pose.Rx = ddd[0];
                temp.Pose.Ry = ddd[1];
                temp.Pose.Rz = ddd[2];
                temp = KinematicsAlgorithm.KinematicsHuaShu.IK(temp.Joints, temp.Pose, Grip2Tool);
                interpolatedPoses.Add(temp);
            }
            return interpolatedPoses;
        }
        // MoveL 功能函数
        // 参数：
        // startPos ：起始点位姿（包含关节位置和笛卡尔位姿）
        // endPose ：终止点位姿
        // DensityIndex: 输出点插值密度
        // Grip2Tool :工具安装位姿矩阵
        public List<Position> MoveL(Position startPos, CartesianPosition endPose, double DensityIndex, Matrix4x4? Grip2Tool = null)
        {//20240516  直接用继承  用基类进行统一（方式之一）    movel 重载   代码规范性
            List<Position> interpolatedPoses = new List<Position>();

            Position endposition = new Position();

            endposition = KinematicsAlgorithm.KinematicsHuaShu.IK(startPos.Joints, endPose, Grip2Tool);

            double distance = Math.Sqrt(Math.Pow((endPose.Point.X - startPos.Pose.Point.X) * 1000, 2) + Math.Pow((endPose.Point.Y - startPos.Pose.Point.Y) * 1000, 2) + Math.Pow((endPose.Point.Z - startPos.Pose.Point.Z) * 1000, 2));
            int numPoints = (int)(distance * DensityIndex);
            Position temp = new Position(endposition);
            double[,] Ri1 = new double[3, 3];
            var Param = new Algorithm. OrientInpParam();
            algorithm.InitialOrientInpParam(algorithm.RxyzToRotMatrix(startPos.Pose.Rx, startPos.Pose.Ry, startPos.Pose.Rz), algorithm.RxyzToRotMatrix(endPose.Rx, endPose.Ry, endPose.Rz), Param);
            double totalTheta = Param.theta; // Total rotation angle
            double dtheta = totalTheta / (numPoints - 1); // Angle change per step
            if (numPoints == 0)
            {
                interpolatedPoses.Add(startPos);
                return interpolatedPoses;
            }

            for (int i = 0; i <= numPoints; i++)
            {
                double t = (double)i / numPoints;

                temp.Pose = algorithm.Lerp(startPos.Pose, endPose, t);

                algorithm.QuaternionOrientInp(Param, dtheta, Ri1);
                double[] ddd = algorithm.RotMatrixToRxyz(Param.Ri);
                temp.Pose.Rx = ddd[0];
                temp.Pose.Ry = ddd[1];
                temp.Pose.Rz = ddd[2];
                temp = KinematicsAlgorithm.KinematicsHuaShu. IK(temp.Joints, temp.Pose, Grip2Tool);
                interpolatedPoses.Add(temp);
            }
            return interpolatedPoses;
        }
        // MoveJ 功能函数
        // 参数：
        // startPos ：起始点位姿（包含关节位置和笛卡尔位姿）
        // endPose ：终止点位姿
        // DensityIndex: 输出点插值密度
        // Grip2Tool :工具安装位姿矩阵
        public List<Position> MoveJ(Position startPos, CartesianPosition endPose, double DensityIndex, Matrix4x4? Grip2Tool = null)
        {
            List<Position> interpolatedPoses = new List<Position>();
            Position endpos = new Position();
            endpos = KinematicsAlgorithm.KinematicsHuaShu.IK(startPos.Joints, endPose, Grip2Tool);
            double distance = Math.Sqrt(Math.Pow((endPose.Point.X - startPos.Pose.Point.X) * 1000, 2) + Math.Pow((endPose.Point.Y - startPos.Pose.Point.Y) * 1000, 2) + Math.Pow((endPose.Point.Z - startPos.Pose.Point.Z) * 1000, 2));
            int numPoints = (int)(distance * DensityIndex);
            for (int i = 0; i <= numPoints; i++)
            {
                double t = (double)i / numPoints;
                Position temp = new Position(algorithm.Lerp(startPos.Joints, endpos.Joints, t), Grip2Tool);
                interpolatedPoses.Add(temp);
            }
            return interpolatedPoses;
        }
        public List<Position> MoveJ(Position startPos, JointPosition endJoint, int numPoints, Matrix4x4? Grip2Tool = null)
        {
            List<Position> interpolatedPoses = new List<Position>();
            for (int i = 0; i <= numPoints; i++)
            {
                double t = (double)i / numPoints;
                Position temp = new Position(algorithm.Lerp(startPos.Joints, endJoint, t), Grip2Tool);
                interpolatedPoses.Add(temp);
            }
            return interpolatedPoses;
        }
    }
}
