using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Numerics;
using System.ServiceProcess;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

namespace RobotLibraryAlgorithm
{
    public class CartesianPosition
    {
        private Point3D point;
        public Point3D Point { get { return point; } set { point = value; } }
        public double Rx { get; set; }
        public double Ry { get; set; }
        public double Rz { get; set; }

        public CartesianPosition(double x_, double y_, double z_, double rx_, double ry_, double rz_)
        {
            point.X = x_;
            point.Y = y_;
            point.Z = z_;
            Rx = rx_;
            Ry = ry_;
            Rz = rz_;

        }
        //默认构造函数
        public CartesianPosition()
        {
            point.X = 0;
            point.Y = 0;
            point.Z = 0;
            Rx = 0;
            Ry = 0;
            Rz = 0;
        }
        // 构造函数：数组赋值
        public CartesianPosition(double[] pose)
        {
            point.X = pose[0];
            point.Y = pose[1];
            point.Z = pose[2];
            Rx = pose[3];
            Ry = pose[4];
            Rz = pose[5];
        }

        public CartesianPosition(double[] xyz, double[] Rxyz)
        {
            point.X = xyz[0];
            point.Y = xyz[1];
            point.Z = xyz[2];
            Rx = Rxyz[0];
            Ry = Rxyz[1];
            Rz = Rxyz[2];
        }


        //拷贝构造函数
        public CartesianPosition(CartesianPosition _pose)
        {
            Point = _pose.Point;
            Rx = _pose.Rx;
            Ry = _pose.Ry;
            Rz = _pose.Rz;
        }
        public CartesianPosition mmTom()
        { 
            return new CartesianPosition(
            point.X / 1000,
            point.Y / 1000,
             point.Z / 1000,
            Rx,
            Ry,
            Rz)
          ;

        }

        public CartesianPosition mTomm()
        {
            return new CartesianPosition(
            point.X * 1000,
            point.Y * 1000,
            point.Z * 1000,
            Rx,
            Ry,
            Rz)
          ;
        }
       
        public CartesianPosition AngleToRad()
        {
            return new CartesianPosition(
            point.X,
            point.Y,
            point.Z,
            Rx / 180 * Math.PI,
            Ry / 180 * Math.PI,
            Rz / 180 * Math.PI);
        }
    }
}
