using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace RobotLibrary
{
    public class TrajectoryPlanning
    {
        // 生成平面栅格扫查点（包含xyz RxRyRz）
        /* 
        * startPoint :起始点
        * endPoint : 终止点
        * dirVector : 平面法向量
        * width ： 扫查栅格宽度
        * length : 扫查栅格长度
        * reverse : 姿态与法向量同向（默认与法向量反向）
        * 输出: x y z Rx Ry Rz ：List<CartesianPosition>
        */
        public static List<CartesianPosition> GridPath(Vector3 startPoint, Vector3 endPoint, Vector3 dirVector, float width, float length, bool reverse = false)
        {
            List<CartesianPosition> pathList = new List<CartesianPosition>();

            //定义 startPoint 指向  endPoint 的向量
            Vector3 widthVector = Vector3.Normalize(endPoint - startPoint);

            //定义 dirVector × widthVector 的向量
            Vector3 lengthVector = Vector3.Normalize(Vector3.Cross(dirVector, widthVector));

            Vector3 baseZ = -Vector3.Normalize(Vector3.Cross(widthVector, lengthVector));

            //double[,] R = BasicAlgorithm.ZThetaToRotMatrix(baseZ, posture);


            if (reverse)
            {
                lengthVector = -lengthVector;
            }

            double[,] R = BasicAlgorithm.ZXToRotMatrix(baseZ, lengthVector);

            double[] Rxyz = BasicAlgorithm.RotMatrixToRxyz(R);

            //计算startPoint 到  endPoint 的距离
            float distance = Vector3.Distance(startPoint, endPoint);

            if (distance < width)
            {
                width = distance;
            }

            // 计算 distance / width 的 商数
            int num = (int)(distance / width);

            Vector3 currPoint = startPoint;
            pathList.Add(new CartesianPosition(startPoint.X, startPoint.Y, startPoint.Z, Rxyz[0], Rxyz[1], Rxyz[2]));

            for (int i = 0; i < num; i++)
            {
                //判断
                if (i % 2 == 0)
                {
                    currPoint = currPoint + (length * lengthVector);

                    pathList.Add(new CartesianPosition(currPoint.X, currPoint.Y, currPoint.Z, Rxyz[0], Rxyz[1], Rxyz[2]));

                    currPoint = currPoint + (width * widthVector);

                    pathList.Add(new CartesianPosition(currPoint.X, currPoint.Y, currPoint.Z, Rxyz[0], Rxyz[1], Rxyz[2]));
                }
                else
                {
                    currPoint = currPoint - (length * lengthVector);

                    pathList.Add(new CartesianPosition(currPoint.X, currPoint.Y, currPoint.Z, Rxyz[0], Rxyz[1], Rxyz[2]));

                    currPoint = currPoint + (width * widthVector);

                    pathList.Add(new CartesianPosition(currPoint.X, currPoint.Y, currPoint.Z, Rxyz[0], Rxyz[1], Rxyz[2]));
                }

            }

            if (num % 2 == 0)
            {
                currPoint = currPoint + (length * lengthVector);

                pathList.Add(new CartesianPosition(currPoint.X, currPoint.Y, currPoint.Z, Rxyz[0], Rxyz[1], Rxyz[2]));

                currPoint = endPoint + (length * lengthVector);

                pathList.Add(new CartesianPosition(currPoint.X, currPoint.Y, currPoint.Z, Rxyz[0], Rxyz[1], Rxyz[2]));
            }
            else
            {
                currPoint = currPoint - (length * lengthVector);

                pathList.Add(new CartesianPosition(currPoint.X, currPoint.Y, currPoint.Z, Rxyz[0], Rxyz[1], Rxyz[2]));
            }

            if (currPoint != endPoint)
            {
                pathList.Add(new CartesianPosition(endPoint.X, endPoint.Y, endPoint.Z, Rxyz[0], Rxyz[1], Rxyz[2]));
            }

            return pathList;
        }


        //直线
        public static List<CartesianPosition> LinePath(CartesianPosition start,CartesianPosition end,int num)
        {
            List<CartesianPosition > pathList = new List<CartesianPosition>();
            CartesianPosition temp=new CartesianPosition();
           for (int i = 0;i<num;i++)
            {
                double t = (double)i / num;
              temp=CartesianPosition.Lerp(start, end, t);
                if (i == num - 1)
                    pathList.Add(end);
                else
                    pathList.Add(temp);

            }
           return pathList;


        }





        // 生成平面栅格扫查点（只包含xyz）
        public static List<Vector3> CalculateZigzagPath(Vector3 startPoint, Vector3 endPoint, Vector3 dirVector, float width, float length, bool reverse = false)
        {

            List<Vector3> pathList = new List<Vector3>();

            //定义 startPoint 指向  endPoint 的向量
            Vector3 widthVector = Vector3.Normalize(endPoint - startPoint);

            //定义 dirVector × widthVector 的向量
            Vector3 lengthVector = Vector3.Normalize(Vector3.Cross(dirVector, widthVector));

            Vector3 normalVector = Vector3.Normalize(Vector3.Cross(widthVector, lengthVector));

            if (reverse)
            {
                lengthVector = -lengthVector;
            }

            //计算startPoint 到  endPoint 的距离
            float distance = Vector3.Distance(startPoint, endPoint);

            if (distance < width)
            {
                width = distance;
            }

            // 计算 distance / width 的 商数
            int num = (int)(distance / width);

            Vector3 currPoint = startPoint;
            pathList.Add(startPoint);
            for (int i = 0; i < num; i++)
            {
                //判断
                if (i % 2 == 0)
                {
                    currPoint = currPoint + (length * lengthVector);

                    pathList.Add(currPoint);

                    currPoint = currPoint + (width * widthVector);

                    pathList.Add(currPoint);
                }
                else
                {
                    currPoint = currPoint - (length * lengthVector);

                    pathList.Add(currPoint);

                    currPoint = currPoint + (width * widthVector);

                    pathList.Add(currPoint);
                }

            }

            if (num % 2 == 0)
            {
                currPoint = currPoint + (length * lengthVector);

                pathList.Add(currPoint);

                currPoint = endPoint + (length * lengthVector);

                pathList.Add(currPoint);
            }
            else
            {
                currPoint = currPoint - (length * lengthVector);

                pathList.Add(currPoint);
            }

            if (currPoint != endPoint)
            {
                pathList.Add(endPoint);
            }


            return pathList;

        }

        // 生成平面栅格扫查点（只包含xyz）引入 step （每两个点之间插入(distance / (width * step)) 个点）
        public static List<Vector3> CalculateZigzagPath(Vector3 startPoint, Vector3 endPoint, Vector3 dirVector, float step, int width, int length, bool reverse = false)
        {

            List<Vector3> pathList = new List<Vector3>();

            //定义 startPoint 指向  endPoint 的向量
            Vector3 widthVector = Vector3.Normalize(endPoint - startPoint);

            //定义 dirVector × widthVector 的向量
            Vector3 lengthVector = Vector3.Normalize(Vector3.Cross(dirVector, widthVector));

            //定义姿态向量
            //            Vector3 normalVector = Vector3.Normalize(Vector3.Cross(widthVector, widthVector));


            if (reverse)
            {
                lengthVector = -lengthVector;
            }

            //计算startPoint 到  endPoint 的距离
            float distance = Vector3.Distance(startPoint, endPoint);

            //if (distance < width * step)
            //{
            //    width = distance;
            //}

            // 计算 distance / width 的 商数
            int num = (int)(distance / (width * step));

            Vector3 currPoint = startPoint;
            pathList.Add(startPoint);
            for (int i = 0; i < num; i++)
            {
                //判断
                if (i % 2 == 0)
                {
                    for (int j = 0; j < length; j++)
                    {
                        currPoint = currPoint + (step * lengthVector);
                        pathList.Add(currPoint);
                    }

                    for (int j = 0; j < width; j++)
                    {
                        currPoint = currPoint + (step * widthVector);
                        pathList.Add(currPoint);
                    }

                }
                else
                {
                    for (int j = 0; j < length; j++)
                    {
                        currPoint = currPoint - (step * lengthVector);
                        pathList.Add(currPoint);
                    }

                    for (int j = 0; j < width; j++)
                    {
                        currPoint = currPoint + (step * widthVector);
                        pathList.Add(currPoint);
                    }
                }

            }

            if (num % 2 == 0)
            {
                for (int j = 0; j < length; j++)
                {
                    currPoint = currPoint + (step * lengthVector);
                    pathList.Add(currPoint);
                }


                for (int j = 0; j < (int)((distance - num * width * step) / step); j++)
                {
                    currPoint = currPoint + (step * widthVector);
                    pathList.Add(currPoint);
                }

                for (int j = 0; j < length; j++)
                {
                    currPoint = currPoint - (step * lengthVector);
                    pathList.Add(currPoint);
                }

            }
            else
            {
                for (int j = 0; j < length; j++)
                {
                    currPoint = currPoint - (step * lengthVector);
                    pathList.Add(currPoint);
                }
            }

            if (currPoint != endPoint)
            {
                pathList.Add(endPoint);
            }


            return pathList;

        }


        /* 
         * COCircle : 圆柱轴线上一点（底面圆心）
         * dirVector : 圆柱轴（方向向量）
         * radius : 圆柱半径
         * length : 圆柱长度（希望检测的长度）
         * round_num : 一个圆插几个点
         * reverse : 姿态朝向圆心还是圆外（默认朝向圆外）
         * posture : z轴确定下的姿态控制参数（控制x，y轴）单位：rad
         * 输出: x y z Rx Ry Rz 
         */
        public static List<CartesianPosition> RotundityPath(Vector3 COCircle, Vector3 NormalVector, float radius, float length, int length_num, int round_num, float posture = 0, bool reverse = false)
        {
            List<CartesianPosition> pathList = new List<CartesianPosition>();

            Vector3 currPoint;
            Vector3 currCircle = COCircle;

            NormalVector = Vector3.Normalize(NormalVector);

            Vector3 DirectionVector;
            double[,] R;
            double[] Rxyz;


            for (int i = 1; i <= length_num; i++)
            {
                DirectionVector = BasicAlgorithm.ZThetaToX(NormalVector, 0);
                //R = BasicAlgorithm.ZThetaToRotMatrix(-DirectionVector, 0);  // thate 添加
                if (reverse)
                {
                    R = BasicAlgorithm.ZThetaToRotMatrix(DirectionVector, posture);
                }
                else
                {
                    R = BasicAlgorithm.ZThetaToRotMatrix(-DirectionVector, posture);
                }

                for (int j = 0; j <= round_num; j++)
                {
                    //double[,] R1 = BasicAlgorithm.Rotation_Axi_theta(R, NormalVector, -(float)(2 * Math.PI / round_num * j));

                    DirectionVector = BasicAlgorithm.ZThetaToX(NormalVector, (float)(2 * Math.PI / round_num * j));
                    currPoint = currCircle + DirectionVector * radius;

                    if (reverse)
                    {
                        R = BasicAlgorithm.ZXToRotMatrix(DirectionVector, new Vector3((float)R[0, 0], (float)R[1, 0], (float)R[2, 0]));  // thate 添加
                    }
                    else
                    {
                        R = BasicAlgorithm.ZXToRotMatrix(-DirectionVector, new Vector3((float)R[0, 0], (float)R[1, 0], (float)R[2, 0]));  // thate 添加
                    }

                    //R = BasicAlgorithm.ZThetaToRotMatrix(-DirectionVector, 1);  // thate 添加
                    Rxyz = BasicAlgorithm.RotMatrixToRxyz(R);

                    pathList.Add(new CartesianPosition(currPoint.X, currPoint.Y, currPoint.Z, Rxyz[0], Rxyz[1], Rxyz[2]));
                }

                currCircle = COCircle + length * NormalVector / length_num * i;

            }

            return pathList;
        }


        /* 
        * COCircle : 圆柱轴线上一点（底面圆心）
        * dirVector : 圆柱轴（方向向量）
        * radius : 圆柱半径
        * length : 圆柱长度（希望检测的长度）
        * round_num : 一个圆插几个点
        * reverse : 姿态朝向圆心还是圆外（默认朝向圆外）
        */
        public static List<Vector3> CalculateRotundityPath(Vector3 COCircle, Vector3 dirVector, float radius, float length, int round_num, bool reverse = false)
        {
            List<Vector3> pathList = new List<Vector3>();

            Vector3 currPoint;
            Vector3 BaseVector;

            dirVector = Vector3.Normalize(dirVector);

            // 如果共线则换一个标准向量
            if (Vector3.Dot(dirVector, Vector3.UnitY) == 1 || Vector3.Dot(dirVector, Vector3.UnitY) == -1)
            {
                BaseVector = Vector3.Normalize(Vector3.Cross(dirVector, Vector3.UnitX));
            }
            else
            {
                BaseVector = Vector3.Normalize(Vector3.Cross(dirVector, Vector3.UnitY));
            }

            //定义 dirVector × widthVector 的向量
            //Vector3 lengthVector = Vector3.Normalize(Vector3.Cross(dirVector, BaseVector));

            //Matrix4x4.CreateFromAxisAngle(dirVector, (float)(2 * 3.1415926 / round_num));




            for (int i = 0; i < round_num; i++)
            {
                float angle = 2 * 3.1415926f * i / round_num; // 计算旋转角度
                Matrix4x4 rotationMatrix = Matrix4x4.CreateFromAxisAngle(dirVector, angle);
                // 需要输出默认位姿则输出 rotatedVector
                Vector3 rotatedVector = Vector3.Transform(BaseVector, rotationMatrix); // 应用旋转

                if (i % 2 == 0)
                {
                    currPoint = COCircle + radius * rotatedVector;
                    pathList.Add(currPoint);

                    currPoint = currPoint + length * dirVector;

                    pathList.Add(currPoint);
                }
                else
                {
                    currPoint = COCircle + radius * rotatedVector;

                    currPoint = currPoint + length * dirVector;

                    pathList.Add(currPoint);

                    currPoint = currPoint - length * dirVector;
                    pathList.Add(currPoint);
                }

            }


            return pathList;
        }


        /* 
        * COSphere : 球心
        * dirVector : 轨迹中心轴（方向向量）
        * radius : 球半径
        * startrad : 起始弧度
        * endrad : 终止弧度
        * rad_num ：起始弧度到终止弧度之间插值扫查个数
        * round_num : 一个圆插几个点
        * posture : z轴确定下的姿态控制参数（控制x，y轴）单位：rad
        * reverse : 姿态朝向球心还是球外（默认朝向球心）
        */
        public static List<CartesianPosition> SphericalPath(Vector3 COSphere, Vector3 dirVector, float radius, float startrad, float endrad, int rad_num, int round_num, float posture = 0, bool reverse = false)
        {
            List<Vector3> pathList = new List<Vector3>();
            List<CartesianPosition> PoseList = new List<CartesianPosition>();

            double thate = 0;
            double phi = 0;
            //x = r sin θcosφ
            //y=rsin θsinφ,
            //z = rcosθ.

            for (int i = 0; i <= rad_num; i++)
            {
                thate = startrad + (endrad - startrad) / rad_num * i;

                for (int j = 0; j <= round_num; j++)
                {
                    phi = 2 * Math.PI / round_num * j;

                    double x = radius * Math.Sin(thate) * Math.Cos(phi);
                    double y = radius * Math.Sin(thate) * Math.Sin(phi);
                    double z = radius * Math.Cos(thate);


                    pathList.Add(new Vector3((float)x, (float)y, (float)z));
                    if (thate == 0 || Math.Abs(Math.PI - thate) < 0.01)
                    {
                        break;
                    }
                }
            }

            //依据 COSphere 和 dirVector 生成姿态

            double[,] R;
            double[] Rxyz;

            R = BasicAlgorithm.ZThetaToRotMatrix(dirVector, 0);
            // 新建4*4矩阵
            Matrix4x4 matrix = new Matrix4x4();
            // 将R的值赋给matrix
            matrix.M11 = (float)R[0, 0];
            matrix.M12 = (float)R[0, 1];
            matrix.M13 = (float)R[0, 2];
            matrix.M14 = COSphere.X;
            matrix.M21 = (float)R[1, 0];
            matrix.M22 = (float)R[1, 1];
            matrix.M23 = (float)R[1, 2];
            matrix.M24 = COSphere.Y;
            matrix.M31 = (float)R[2, 0];
            matrix.M32 = (float)R[2, 1];
            matrix.M33 = (float)R[2, 2];
            matrix.M34 = COSphere.Z;
            matrix.M41 = 0;
            matrix.M42 = 0;
            matrix.M43 = 0;
            matrix.M44 = 1;




            // 将pathList 中的点依据 matrix 进行旋转
            for (int i = 0; i < pathList.Count; i++)
            {
                //Vector3 rotatedVector = new Vector3(pathList[i].X * matrix.M11 + pathList[i].Y * matrix.M21 + pathList[i].Z * matrix.M31 + matrix.M41, pathList[i].X * matrix.M12 + pathList[i].Y * matrix.M22 + pathList[i].Z * matrix.M32 + matrix.M42, pathList[i].X * matrix.M13 + pathList[i].Y * matrix.M23 + pathList[i].Z * matrix.M33 + matrix.M43);
                Vector3 rotatedVector = new Vector3(pathList[i].X * matrix.M11 + pathList[i].Y * matrix.M12 + pathList[i].Z * matrix.M13 + matrix.M14, pathList[i].X * matrix.M21 + pathList[i].Y * matrix.M22 + pathList[i].Z * matrix.M23 + matrix.M24, pathList[i].X * matrix.M31 + pathList[i].Y * matrix.M32 + pathList[i].Z * matrix.M33 + matrix.M34);

                Vector3 z;
                if (reverse == false)
                {
                    z = -Vector3.Normalize(rotatedVector - COSphere);
                }
                else
                {
                    z = Vector3.Normalize(rotatedVector - COSphere);
                }


                if (i == 0)
                {
                    R = BasicAlgorithm.ZThetaToRotMatrix(z, posture);
                }
                else
                {
                    R = BasicAlgorithm.ZXToRotMatrix(z, new Vector3((float)R[0, 0], (float)R[1, 0], (float)R[2, 0]));
                }

                Rxyz = BasicAlgorithm.RotMatrixToRxyz(R);

                PoseList.Add(new CartesianPosition(rotatedVector.X, rotatedVector.Y, rotatedVector.Z, Rxyz[0], Rxyz[1], Rxyz[2]));

            }

            return PoseList;
        }

    }
}
