using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace RobotLibraryAlgorithm.VirtualAxis
{
    public class VirtualAxes
    {
        private Algorithm basicalgortihm;
        public VirtualAxes()
        {
            basicalgortihm = new Algorithm();
        }

        /****************************************************************************
        *@name	  : DisplacementInTool
        *@brief	  : 计算在工具坐标系下的X与Y方向的位移(虚拟两轴的位移)单位：m(由输入决定)
        *@param	  CartesianPosition startPose : 起始点 机械手末端位姿（不是工具末端位姿）
        *@param   CartesianPosition endPose ：终止点 机械手末端位姿（不是工具末端位姿）
        *@param	  Matrix4x4? Grip2Tool：机械臂末端坐标系到工具末端坐标系之间的转换矩阵
        *@return  Vector2 : 终止点工具末端位置 投影到起始点工具末端坐标系 的 X Y两轴的位移 单位：m(由输入决定)
        ****************************************************************************/
        public Vector2 DisplacementInTool(double[] startAngle, double[] endAngle, Matrix4x4? Grip2Tool = null)
        {
            //Console.WriteLine("oldangle",startAngle);
            //Console.WriteLine("endangle",endAngle);
            Position startPosition = new Position(basicalgortihm.AngleToRad(startAngle));
            Position endPosition = new Position(basicalgortihm.AngleToRad(endAngle));
            //Console.WriteLine("---------");
            //Console.WriteLine("startx:" + startPosition.Pose.Point.X);
            //Console.WriteLine("endx" + endPosition.Pose.Point.X);
            //Console.WriteLine("starty:" + startPosition.Pose.Point.Y);
            //Console.WriteLine("endy:" + endPosition.Pose.Point.Y);

            Matrix4x4 Grip2ToolMatrix = Grip2Tool ?? Matrix4x4.Identity;

            // 
            Matrix4x4 startRT_BaseToGrip = basicalgortihm.ToTrans(startPosition.Pose);
            Matrix4x4 startRT_BaseToTool = startRT_BaseToGrip * Grip2ToolMatrix;


            Matrix4x4 endRT_BaseToGrip = basicalgortihm.ToTrans(endPosition.Pose);
            Matrix4x4 endRT_BaseToTool = endRT_BaseToGrip * Grip2ToolMatrix;

            Matrix4x4 startRT_ToolToBase = new Matrix4x4();
            Matrix4x4.Invert(startRT_BaseToTool, out startRT_ToolToBase);


            Vector4 vector = new Vector4(endRT_BaseToTool.M14, endRT_BaseToTool.M24, endRT_BaseToTool.M34, endRT_BaseToTool.M44);

            //Matrix4x4 result = startRT_ToolToBase * endRT_BaseToTool;
            //return new Vector2 { X = result.M14, Y = result.M24 };

            Matrix4x4 transposeMatrix = Matrix4x4.Transpose(startRT_ToolToBase);

            Vector4 result = Vector4.Transform(vector, transposeMatrix);
            //Console.WriteLine("Toolx:" + result.X);
            //Console.WriteLine("Tooly:" + result.Y);

            return new Vector2 { X = result.X, Y = result.Y };

        }






        /****************************************************************************
        *@name	  : DisplacementInTool
        *@brief	  : 计算在工具坐标系下的X与Y方向的位移(虚拟两轴的位移)单位：m(由输入决定)
        *@param	  CartesianPosition startPose : 起始点 机械手末端位姿（不是工具末端位姿）
        *@param   CartesianPosition endPose ：终止点 机械手末端位姿（不是工具末端位姿）
        *@param	  Matrix4x4? Grip2Tool：机械臂末端坐标系到工具末端坐标系之间的转换矩阵
        *@return  Vector2 : 终止点工具末端位置 投影到起始点工具末端坐标系 的 X Y两轴的位移 单位：m(由输入决定)
        ****************************************************************************/
        public  Vector2 DisplacementInTool(CartesianPosition startPose, CartesianPosition endPose, Matrix4x4? Grip2Tool = null)
        {
            Matrix4x4 Grip2ToolMatrix = Grip2Tool ?? Matrix4x4.Identity;

            // 
            Matrix4x4 startRT_BaseToGrip = basicalgortihm.ToTrans(startPose);
            Matrix4x4 startRT_BaseToTool = startRT_BaseToGrip * Grip2ToolMatrix;


            Matrix4x4 endRT_BaseToGrip = basicalgortihm.ToTrans(endPose);
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
        public  List<Vector2> DisplacementInTool(List<CartesianPosition> Poselist, Matrix4x4? Grip2Tool = null)
        {
            Matrix4x4 Grip2ToolMatrix = Grip2Tool ?? Matrix4x4.Identity;


            List<Vector2> Displacements = new List<Vector2>();

            if (Poselist.Count <= 1)
            {
                return Displacements;
            }

            Matrix4x4 startRT_BaseToGrip =  basicalgortihm.ToTrans(Poselist[0]);
            Matrix4x4 startRT_BaseToTool = startRT_BaseToGrip * Grip2ToolMatrix;


            for (int i = 1; i < Poselist.Count; i++)
            {

                Matrix4x4 endRT_BaseToGrip =basicalgortihm.ToTrans(Poselist[i]);
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
