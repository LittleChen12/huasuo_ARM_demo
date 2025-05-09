﻿using HelixToolkit.Wpf;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

namespace Model
{
    public class BoxModel
    {
        public MeshGeometry3D Boxmodel;
        public ModelVisual3D Boxmodelvisual;
        public BoxModel()
        {
            Boxmodel=new MeshGeometry3D();
            Boxmodelvisual = new ModelVisual3D();
        }
        /// <summary>
        /// 初始化box模型
        /// mm为模型尺寸，_point为模型中心点
        /// </summary>
        /// <param name="_point"></param>
        /// <param name="position"></param>
        public void BoxModelInit(Point3D _point,double[] position)
        {
            MeshBuilder builder = new MeshBuilder(true, true);
            builder.AddBox(_point, position[0], position[1], position[2]);
            GeometryModel3D model = new GeometryModel3D(builder.ToMesh(), Materials.White);
            Boxmodel = builder.ToMesh();
            Boxmodelvisual.Content = model;
        }
    }
}
