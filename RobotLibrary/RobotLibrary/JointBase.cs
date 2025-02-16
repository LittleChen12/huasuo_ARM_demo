using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

namespace RobotLibrary
{
    public abstract class JointBase
    {
        /// <summary>
        /// 变量类型 变量名称 变量解释
        /// Model3D  model3D  3D模型
        /// ModelVisual3D  modelvisual3D  3D模型显示
        /// string  jointpath  3D模型文件路径
        /// </summary>
        public Model3D model3D { get; set; }
        public ModelVisual3D modelvisual3D { get; set; }
        public string jointpath { get; set; }
        /// <summary>
        /// 设置3D模型
        /// </summary>
        /// <param name="model">Model3D</param>
        public void SetModel3D(Model3D model) 
        {
            model3D = model;
        }
        /// <summary>
        /// 设置3D模型显示
        /// </summary>
        /// <param name="modelvisual">ModelVisual3D</param>
        public void SetModelVisual3D(ModelVisual3D modelvisual)
        {
            modelvisual3D = modelvisual;
        }
        /// <summary>
        /// 设置3D模型文件路径
        /// </summary>
        /// <param name="path">string</param>
        public void SetJointPath(string path)
        {
            jointpath = path;
        }
    }
   
}
