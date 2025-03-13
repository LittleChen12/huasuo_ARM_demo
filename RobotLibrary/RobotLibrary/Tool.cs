using RobotLibraryAlgorithm;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Reflection.Metadata.Ecma335;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

namespace RobotLibrary
{
    public class Tool: JointBase 
    {
        public CartesianPosition CartesianPosition { get; set; }
        public TranslateTransform3D ToolTd;
        public Matrix4x4 GripToTool ;

        public Tool() { 
            CartesianPosition = new CartesianPosition();
            ToolTd=new TranslateTransform3D();
            GripToTool = new Matrix4x4(
                     1, 0, 0, 0,//
                     0, 1, 0, 0,
                     0, 0, 1, 0,
                     0, 0, 0, 1

                );
            model3D = new GeometryModel3D();
            modelvisual3D=new ModelVisual3D();
            jointpath = "";
            //        GripToTool = new Matrix4x4(
            //    1, 0, 0, 0,
            //    0, 1, 0, 0.111f,
            //    0, 0, 1, 0.042f,
            //     0, 0, 0, 1

            //);
        }
        public Tool(CartesianPosition cartesianPosition)
        {
            CartesianPosition = cartesianPosition;
            ToolTd = new TranslateTransform3D();
        }
        public Tool(double _x, double _y, double _z, double _Rx, double _Ry, double _Rz)
        {
            CartesianPosition = new CartesianPosition(_x, _y, _z, _Rx, _Ry, _Rz);
            ToolTd = new TranslateTransform3D();
        }
    }
}
