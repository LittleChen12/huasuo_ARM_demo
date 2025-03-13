using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;
using System.Windows.Media;
using HelixToolkit.Wpf;

namespace RobotLibrary
{
    public class RobotStlLoad
    {
        private Model3DGroup RobotModel;
        public RobotStlLoad() 
        {
            RobotModel = new Model3DGroup();
        }

        private static string GetCurSourceFileName()
        {
            string path = Directory.GetCurrentDirectory();
            var p = Directory.GetParent(path);
            return p.Parent.Parent.FullName;
        }
        //不同3d数据格式12.12
        /*
        * @name	      : ModelImport
        * @brief	  : 加载3D模型
        * @param path ：机械臂各个关节路径  
        * @return Model3dGroup : 机械臂模型
        */
        public Model3DGroup JointsStlLoad(string path)
        {
            List<string> ModelNames = new List<string>();
            //获取该路径下所有文件地址
            string[] files = Directory.GetFiles(path);
            foreach (string file in files) { ModelNames.Add(file); }
            //模型材质
            var materialGroup = ModelMaterial();
            //关节轴STL文件->Model3DGroup
            List<GeometryModel3D> JointModels = new List<GeometryModel3D>();
            ModelImporter import = new ModelImporter();
            foreach (string modelname in ModelNames)
            {
                var joint_model = import.Load(modelname).Children[0] as GeometryModel3D;
                joint_model.Material = materialGroup;
                RobotModel.Children.Add(joint_model);
            }
            return RobotModel;
        }

        /// <summary>
        /// 加载单个STL文件
        /// </summary>
        /// <param name="path"></param>
        /// <returns></returns>
        public Model3D ModelSTLload(string path)
        {
            ModelImporter import = new ModelImporter();
            var model= import.Load(path).Children[0] as GeometryModel3D;
            return model;
        }
        /*
         * @name	  : ModelMaterial
         * @brief	  : 3D模型材质
         * @param	  ：无  
         * @return  MaterialGroup : 模型材质组合
         */
        private MaterialGroup ModelMaterial()
        {
            var materialGroup = new MaterialGroup();
            Color mainColor = Colors.White;
            EmissiveMaterial emissMat = new EmissiveMaterial(new SolidColorBrush(mainColor));
            DiffuseMaterial diffMat = new DiffuseMaterial(new SolidColorBrush(mainColor));
            SpecularMaterial specMat = new SpecularMaterial(new SolidColorBrush(mainColor), 200);
            materialGroup.Children.Add(emissMat);
            materialGroup.Children.Add(diffMat);
            materialGroup.Children.Add(specMat);
            return materialGroup;
        }
    }
}
