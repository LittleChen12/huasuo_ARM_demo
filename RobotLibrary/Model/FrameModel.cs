using HelixToolkit.Wpf;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Controls;
using System.Windows.Media.Media3D;

namespace Model
{
    public class FrameModel
    {
        public Model3DGroup FrameModelGroup;
        public ModelVisual3D FrameModelVisual;
        public FrameModel()
        {
            FrameModelVisual= new ModelVisual3D();
            FrameModelGroup = new Model3DGroup();
        }
        /// <summary>
        /// 创建坐标轴 根据位置创建坐标轴
        /// mm单位
        /// </summary>
        /// <param name="frame"></param>
        public void FrameModelInit(double[] frame)
        {
            MeshBuilder builderx = new MeshBuilder(false, false);
            builderx.AddCylinder(new Point3D(frame[0], frame[1], frame[2]), new Point3D(frame[0], frame[1], frame[2] + 80), 5, 36);
            builderx.AddCone(new Point3D(frame[0], frame[1], frame[2] + 75), new Point3D(frame[0], frame[1], frame[2] + 85), 7.5, false, 36);
            var gx = new GeometryModel3D(builderx.ToMesh(), Materials.Red);
            MeshBuilder bulidery = new MeshBuilder(false, false);
            bulidery.AddCylinder(new Point3D(frame[0], frame[1], frame[2]), new Point3D(frame[0], frame[1] - 80, frame[2]), 5, 36);
            bulidery.AddCone(new Point3D(frame[0], frame[1] - 75, frame[2]), new Point3D(frame[0], frame[1] - 85, frame[2]), 7.5, false, 36);
            var gy = new GeometryModel3D(bulidery.ToMesh(), Materials.Green);
            MeshBuilder buliderz = new MeshBuilder(false, false);
            buliderz.AddCylinder(new Point3D(frame[0], frame[1], frame[2]), new Point3D(frame[0] + 80, frame[1], frame[2]), 5, 36);
            buliderz.AddCone(new Point3D(frame[0] + 75, frame[1], frame[2]), new Point3D(frame[0] + 85, frame[1], frame[2]), 7.5, false, 36);
            var gz = new GeometryModel3D(buliderz.ToMesh(), Materials.Blue);


            FrameModelGroup.Children.Add(gx);
            FrameModelGroup.Children.Add(gy);
            FrameModelGroup.Children.Add(gz);
            FrameModelVisual.Content = FrameModelGroup;
        }
        public void FrameClear()
        {
            FrameModelGroup.Children.Clear();
            //viewPort3d.Children.Remove(FrameModelVisual);
        }
    }
}
