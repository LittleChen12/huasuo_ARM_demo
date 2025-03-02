using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Security.AccessControl;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using static RobotLibrary.BasicAlgorithm;

namespace RobotLibrary
{
    public class Position
    {
        private CartesianPosition pose;
        private JointPosition joints;
        public int[] ChaoSheng=new int[1025];

        public CartesianPosition Pose
        {
            get { return pose; }
            set { pose = value; }
        }

        public JointPosition Joints
        {
            get { return joints; }
            set { joints = value; }
        }



        public Position(double joint1, double joint2, double joint3, double joint4, double joint5, double joint6, Matrix4x4? Grip2Tool = null)
        {
            joints = new JointPosition(new double[] { joint1, joint2, joint3, joint4, joint5, joint6 });
            pose = new CartesianPosition(PathClass.FK(new double[] { joint1, joint2, joint3, joint4, joint5, joint6 }, Grip2Tool));
        }

        public Position(double[] joint, Matrix4x4? Grip2Tool = null)
        {
            joints = new JointPosition(joint);
            pose = new CartesianPosition(PathClass.FK(joint, Grip2Tool));
        }
        public Position(Position _temp)
        {
            pose = _temp.pose;
            joints = _temp.joints;
        }

        public Position()
        {
            joints = new JointPosition();
            pose = new CartesianPosition();
        }


        public static List<Position> MoveL(Position startPos, CartesianPosition endPose, int numPoints, Matrix4x4? Grip2Tool = null)
        {//20240516  直接用继承  用基类进行统一（方式之一）    movel 重载   代码规范性
            List<Position> interpolatedPoses = new List<Position>();

            Position endposition = new Position();

            endposition = PathClass.IK(startPos.joints, endPose, Grip2Tool);

            //List<CartesianPosition>
            Position temp = new Position(endposition);


            double[,] Ri1 = new double[3, 3];

            OrientInpParam Param = new OrientInpParam();

            InitialOrientInpParam(BasicAlgorithm.RxyzToRotMatrix(startPos.pose.RX, startPos.pose.RY, startPos.pose.RZ), BasicAlgorithm.RxyzToRotMatrix(endPose.RX, endPose.RY, endPose.RZ), Param);


            double totalTheta = Param.theta; // Total rotation angle
            double dtheta = totalTheta / (numPoints - 1); // Angle change per step



            for (int i = 0; i <= numPoints; i++)
            {
                double t = (double)i / numPoints;

                temp.pose = CartesianPosition.Lerp(startPos.pose, endPose, t);

                QuaternionOrientInp(Param, dtheta, Ri1);
                double[] ddd = BasicAlgorithm.RotMatrixToRxyz(Param.Ri);
                temp.pose.RX = ddd[0];
                temp.pose.RY = ddd[1];
                temp.pose.RZ = ddd[2];


                temp = PathClass.IK(temp.joints, temp.pose, Grip2Tool);

                //Position temp = new Position(JointPosition.Lerp(startPos.joints, endpos.joints, t), _arm);

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

        public static List<Position> MoveL(Position startPos, CartesianPosition endPose, double DensityIndex, Matrix4x4? Grip2Tool = null)
        {//20240516  直接用继承  用基类进行统一（方式之一）    movel 重载   代码规范性
            List<Position> interpolatedPoses = new List<Position>();

            Position endposition = new Position();

            endposition = PathClass.IK(startPos.joints, endPose, Grip2Tool);

            double distance = Math.Sqrt(Math.Pow((endPose.X - startPos.pose.X) * 1000, 2) + Math.Pow((endPose.Y - startPos.pose.Y) * 1000, 2) + Math.Pow((endPose.Z - startPos.pose.Z) * 1000, 2));
            int numPoints = (int)(distance * DensityIndex);

            //List<CartesianPosition>
            Position temp = new Position(endposition);


            double[,] Ri1 = new double[3, 3];

            OrientInpParam Param = new OrientInpParam();

            InitialOrientInpParam(BasicAlgorithm.RxyzToRotMatrix(startPos.pose.RX, startPos.pose.RY, startPos.pose.RZ), BasicAlgorithm.RxyzToRotMatrix(endPose.RX, endPose.RY, endPose.RZ), Param);


            double totalTheta = Param.theta; // Total rotation angle
            double dtheta = totalTheta / (numPoints - 1); // Angle change per step

            if(numPoints == 0)
            {
                interpolatedPoses.Add(endposition);
                return interpolatedPoses;
            }

            for (int i = 1; i <= numPoints; i++)
            {


                double t = (double)i / numPoints;

                temp.pose = CartesianPosition.Lerp(startPos.pose, endPose, t);

                QuaternionOrientInp(Param, dtheta, Ri1);
                double[] ddd = BasicAlgorithm.RotMatrixToRxyz(Param.Ri);
                temp.pose.RX = ddd[0];
                temp.pose.RY = ddd[1];
                temp.pose.RZ = ddd[2];


                temp = PathClass.IK(temp.joints, temp.pose, Grip2Tool);

                //Position temp = new Position(JointPosition.Lerp(startPos.joints, endpos.joints, t), _arm);

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

        public static List<Position> MoveJ(Position startPos, CartesianPosition endPose, double DensityIndex, Matrix4x4? Grip2Tool = null)
        {



            List<Position> interpolatedPoses = new List<Position>();

            Position endpos = new Position();

            endpos = PathClass.IK(startPos.joints, endPose, Grip2Tool);

            double distance = Math.Sqrt(Math.Pow((endPose.X - startPos.pose.X) * 1000, 2) + Math.Pow((endPose.Y - startPos.pose.Y) * 1000, 2) + Math.Pow((endPose.Z - startPos.pose.Z) * 1000, 2));
            int numPoints = (int)(distance * DensityIndex);

            //List<CartesianPosition>

            for (int i = 0; i <= numPoints; i++)
            {
                double t = (double)i / numPoints;

                Position temp = new Position(JointPosition.Lerp(startPos.joints, endpos.joints, t), Grip2Tool);

                interpolatedPoses.Add(temp);
            }

            return interpolatedPoses;
        }


        public static List<Position> MoveJ(Position startPos, JointPosition endJoint, int numPoints, Matrix4x4? Grip2Tool = null)
        {



            List<Position> interpolatedPoses = new List<Position>();

            //Position endpos = new Position();

            //endpos = PathClass.IK(startPos.joints, endPose, Grip2Tool);

            //double distance = Math.Sqrt(Math.Pow((endPose.X - startPos.pose.X) * 1000, 2) + Math.Pow((endPose.Y - startPos.pose.Y) * 1000, 2) + Math.Pow((endPose.Z - startPos.pose.Z) * 1000, 2));
            //int numPoints = (int)(distance * DensityIndex);

            //List<CartesianPosition>

            for (int i = 0; i <= numPoints; i++)
            {
                double t = (double)i / numPoints;

                Position temp = new Position(JointPosition.Lerp(startPos.joints, endJoint, t), Grip2Tool);

                interpolatedPoses.Add(temp);
            }

            return interpolatedPoses;
        }

        /****************************************************************************
        *@name	  : DisplacementInTool
        *@brief	  : 计算在工具坐标系下的X与Y方向的位移(虚拟两轴的位移)单位：m(由输入决定)
        *@param	  CartesianPosition startPose : 起始点 机械手末端位姿（不是工具末端位姿）
        *@param   CartesianPosition endPose ：终止点 机械手末端位姿（不是工具末端位姿）
        *@param	  Matrix4x4? Grip2Tool：机械臂末端坐标系到工具末端坐标系之间的转换矩阵
        *@return  Vector2 : 终止点工具末端位置 投影到起始点工具末端坐标系 的 X Y两轴的位移 单位：m(由输入决定)
        ****************************************************************************/
        public static Vector2 DisplacementInTool(double[] startAngle, double[] endAngle, Matrix4x4? Grip2Tool = null)
        {
            //Console.WriteLine("oldangle",startAngle);
            //Console.WriteLine("endangle",endAngle);
            Position startPosition = new Position(AngleToRad(startAngle));
            Position endPosition = new Position(AngleToRad(endAngle));
            Console.WriteLine("^^^^^^^^^^^");
            //Console.WriteLine("startx:"+startPosition.pose.X);
            //Console.WriteLine("endx"+endPosition.pose.X);
            Console.WriteLine("starty:" + startPosition.pose.Y);
            Console.WriteLine("endy:" + endPosition.pose.Y);

            Matrix4x4 Grip2ToolMatrix = Grip2Tool ?? Matrix4x4.Identity;

            // 
            Matrix4x4 startRT_BaseToGrip = startPosition.pose.ToTrans();
            Matrix4x4 startRT_BaseToTool = startRT_BaseToGrip * Grip2ToolMatrix;


            Matrix4x4 endRT_BaseToGrip = endPosition.pose.ToTrans();
            Matrix4x4 endRT_BaseToTool = endRT_BaseToGrip * Grip2ToolMatrix;

            Matrix4x4 startRT_ToolToBase = new Matrix4x4();
            Matrix4x4.Invert(startRT_BaseToTool, out startRT_ToolToBase);


            Vector4 vector = new Vector4(endRT_BaseToTool.M14, endRT_BaseToTool.M24, endRT_BaseToTool.M34, endRT_BaseToTool.M44);

            //Matrix4x4 result = startRT_ToolToBase * endRT_BaseToTool;
            //return new Vector2 { X = result.M14, Y = result.M24 };

            Matrix4x4 transposeMatrix = Matrix4x4.Transpose(startRT_ToolToBase);

            Vector4 result = Vector4.Transform(vector, transposeMatrix);
            Console.WriteLine("------------------");
            Console.WriteLine("Toolx:" + result.X);
            Console.WriteLine("Tooly:" + result.Y);

            return new Vector2 { X = result.X, Y = result.Y };

        }


        ///****************************************************************************
        //*@name	  : DisplacementInTool
        //*@brief	  : 计算在工具坐标系下的X与Y方向的位移(虚拟两轴的位移)单位：m(由输入决定)
        //*@param	  CartesianPosition startPose : 起始点 机械手末端位姿（不是工具末端位姿）
        //*@param   CartesianPosition endPose ：终止点 机械手末端位姿（不是工具末端位姿）
        //*@param	  Matrix4x4? Grip2Tool：机械臂末端坐标系到工具末端坐标系之间的转换矩阵
        //*@return  Vector2 : 终止点工具末端位置 投影到起始点工具末端坐标系 的 X Y两轴的位移 单位：m(由输入决定)
        //****************************************************************************/
        //public static Vector2 DisplacementInTool(double[] startAngle, double[] endAngle, Matrix4x4? Grip2Tool = null)
        //{
        //    //Console.WriteLine("oldangle",startAngle);
        //    //Console.WriteLine("endangle",endAngle);
        //    Position startPosition = new Position(AngleToRad(startAngle));
        //    Position endPosition = new Position(AngleToRad(endAngle));
        //    //Console.WriteLine("---------");
        //    //Console.WriteLine("startx:"+startPosition.pose.X);
        //    //Console.WriteLine("endx"+endPosition.pose.X);
        //    //Console.WriteLine("starty:"+ startPosition.pose.Y);
        //    //Console.WriteLine("endy:"+endPosition.pose.Y);
           
        //    Matrix4x4 Grip2ToolMatrix = Grip2Tool ?? Matrix4x4.Identity;

        //    // 
        //    Matrix4x4 startRT_BaseToGrip = startPosition.pose.ToTrans();
        //    Matrix4x4 startRT_BaseToTool = startRT_BaseToGrip * Grip2ToolMatrix;


        //    Matrix4x4 endRT_BaseToGrip = endPosition.pose.ToTrans();
        //    Matrix4x4 endRT_BaseToTool = endRT_BaseToGrip * Grip2ToolMatrix;

        //    Matrix4x4 startRT_ToolToBase = new Matrix4x4();
        //    Matrix4x4.Invert(startRT_BaseToTool, out startRT_ToolToBase);


        //    Vector4 vector = new Vector4(endRT_BaseToTool.M14, endRT_BaseToTool.M24, endRT_BaseToTool.M34, endRT_BaseToTool.M44);

        //    //Matrix4x4 result = startRT_ToolToBase * endRT_BaseToTool;
        //    //return new Vector2 { X = result.M14, Y = result.M24 };

        //    Matrix4x4 transposeMatrix = Matrix4x4.Transpose(startRT_ToolToBase);

        //    Vector4 result = Vector4.Transform(vector, transposeMatrix);
        //    Console.WriteLine("Toolx:"+result.X);
        //    Console.WriteLine("Tooly:"+result.Y);

        //    return new Vector2 { X = result.X, Y = result.Y };

        //}






        /****************************************************************************
        *@name	  : DisplacementInTool
        *@brief	  : 计算在工具坐标系下的X与Y方向的位移(虚拟两轴的位移)单位：m(由输入决定)
        *@param	  CartesianPosition startPose : 起始点 机械手末端位姿（不是工具末端位姿）
        *@param   CartesianPosition endPose ：终止点 机械手末端位姿（不是工具末端位姿）
        *@param	  Matrix4x4? Grip2Tool：机械臂末端坐标系到工具末端坐标系之间的转换矩阵
        *@return  Vector2 : 终止点工具末端位置 投影到起始点工具末端坐标系 的 X Y两轴的位移 单位：m(由输入决定)
        ****************************************************************************/
        public static Vector2 DisplacementInTool(CartesianPosition startPose, CartesianPosition endPose, Matrix4x4? Grip2Tool = null)
        {
            Matrix4x4 Grip2ToolMatrix = Grip2Tool ?? Matrix4x4.Identity;
            
            // 
            Matrix4x4 startRT_BaseToGrip = startPose.ToTrans();
            Matrix4x4 startRT_BaseToTool = startRT_BaseToGrip * Grip2ToolMatrix;


            Matrix4x4 endRT_BaseToGrip = endPose.ToTrans();
            Matrix4x4 endRT_BaseToTool = endRT_BaseToGrip * Grip2ToolMatrix;

            Matrix4x4 startRT_ToolToBase = new Matrix4x4();
            Matrix4x4.Invert(startRT_BaseToTool, out startRT_ToolToBase);


            Vector4 vector = new Vector4(endRT_BaseToTool.M14, endRT_BaseToTool.M24, endRT_BaseToTool.M34, endRT_BaseToTool.M44);

            //Matrix4x4 result = startRT_ToolToBase * endRT_BaseToTool;
            //return new double[2] { result.M14, result.M24 };

            Vector4 result = Vector4.Transform(vector, startRT_ToolToBase);

            return new Vector2 { X = result.X, Y = result.Y };

        }
        
        /****************************************************************************
        *@name	  : DisplacementInTool
        *@brief	  : 计算在工具坐标系下的X与Y方向的位移(虚拟两轴的位移)单位：m(由输入决定)
        *@param	  List<CartesianPosition> Poselist : 一系列点(大于等于2个点) 机械手末端位姿（不是工具末端位姿）
        *@param	  Matrix4x4? Grip2Tool：机械臂末端坐标系到工具末端坐标系之间的转换矩阵
        *@return  List<Vector2> : 终止点工具末端位置 投影到起始点工具末端坐标系 的 X Y两轴的位移 单位：m(由输入决定)
        ****************************************************************************/
        public static List<Vector2> DisplacementInTool(List<CartesianPosition> Poselist, Matrix4x4? Grip2Tool = null)
        {
            Matrix4x4 Grip2ToolMatrix = Grip2Tool ?? Matrix4x4.Identity;


            List<Vector2> Displacements = new List<Vector2>();

            if (Poselist.Count <= 1)
            {
                return Displacements;
            }
                
            Matrix4x4 startRT_BaseToGrip = Poselist[0].ToTrans();
            Matrix4x4 startRT_BaseToTool = startRT_BaseToGrip * Grip2ToolMatrix;


            for (int i = 1; i< Poselist.Count ;i++)
            {

                Matrix4x4 endRT_BaseToGrip = Poselist[i].ToTrans();
                Matrix4x4 endRT_BaseToTool = endRT_BaseToGrip * Grip2ToolMatrix;

                Matrix4x4 startRT_ToolToBase = new Matrix4x4();
                Matrix4x4.Invert(startRT_BaseToTool, out startRT_ToolToBase);

                Vector4 vector = new Vector4(endRT_BaseToTool.M14, endRT_BaseToTool.M24, endRT_BaseToTool.M34, endRT_BaseToTool.M44);

                //Matrix4x4 result = startRT_ToolToBase * endRT_BaseToTool;
                //return new double[2] { result.M14, result.M24 };

                Vector4 result = Vector4.Transform(vector, startRT_ToolToBase);


                Displacements.Add(new Vector2 { X = result.X, Y = result.Y });

                startRT_BaseToTool = endRT_BaseToTool;

            }

            return Displacements;

        }

    }
}
