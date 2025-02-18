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
    public class SphereModel
    {
        public List<Model3D> SphereModels;
        public List<ModelVisual3D> SphereModelVisuals;
        public Model3D Spheremodel;
        public ModelVisual3D Spheremodelvisual;
        public SphereModel() 
        {
            SphereModelVisuals = new List<ModelVisual3D>();
            SphereModels = new List<Model3D>();
        }
        /// <summary>
        /// 初始化球体模型
        /// mm为单位
        /// </summary>
        /// <param name="position"></param>
       public void SphereModelInit(Point3D position)
       {
            var builder = new MeshBuilder(true, true);
            builder.AddSphere(position, 10);
            Spheremodel = new GeometryModel3D(builder.ToMesh(), Materials.Blue);
            ModelVisual3D v = new ModelVisual3D();
            SphereModels.Add(Spheremodel);
            v.Content = Spheremodel;
            SphereModelVisuals.Add(v);
        }
        public void SphereModelClear()
        {
            SphereModelVisuals.Clear();
            SphereModels.Clear();
            //foreach (var item in SphereModelVisuals.ToArray()) // 使用ToArray()来避免在迭代过程中修改集合  
            //{
            //    viewPort3d.Children.Remove(item);
            //}
        }
       

    }
}
