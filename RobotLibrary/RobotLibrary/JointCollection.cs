using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Security.Policy;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Media.Media3D;

namespace RobotLibrary
{
    public class JointCollection : ICollection<Joint>
    {
        //robotlimit->用关节限定角度写成正运动学函数获取机械臂xyz--  7.23
        /// <summary>
        /// 变量类型 变量名称 变量解释
        /// List<Joint> joints 机器人关节集合
        /// RobotLimit  RobotLimit XYZRotA B C限位
        /// string basepath 机器人关节组模型路径
        /// Model3DGroup RobotModel 机器人关节组模型
        /// ModelVisual3D RobotModelVisual 机器人关节组模型显示
        /// </summary>
        private List<Joint> joints;
        public RobotLimit RobotLimit { get; set; }
        private string basepath;
        public string BasePath
        {
            get { return basepath; }
            set { basepath = value; }
        }
        public int Count => joints.Count;
        public Model3DGroup RobotModel;
        public ModelVisual3D RobotModelVisual;
        public bool IsReadOnly => throw new NotImplementedException();

        /// <summary>
        /// 构造函数
        /// 初始化joints集合
        /// </summary>
        public JointCollection()
        {
            joints = new List<Joint>();
            RobotLimit = new RobotLimit();
            basepath="";
            RobotModel = new Model3DGroup();
            RobotModelVisual = new ModelVisual3D();
        }
       
        /// <summary>
        ///  获取指定索引位置的关节
        /// </summary>
        /// <param name="index"></param>
        /// <returns></returns>
        /// <exception cref="IndexOutOfRangeException"></exception>
        public Joint this[int index]
        {
            get
            {
                if (index < 0 || index >= joints.Count)
                {
                    throw new IndexOutOfRangeException();
                }
                return joints[index];
            }
            set
            {
                if (index < 0 || index >= joints.Count)
                {
                    throw new IndexOutOfRangeException();
                }
                joints[index] = value;
            }
        }
        /// <summary>
        ///  添加关节
        /// </summary>
        /// <param name="item"></param>
        public void Add(Joint item)
        {
            joints?.Add(item);

        }
        /// <summary>
        /// 清空关节
        /// </summary>
        public void Clear()
        {
            joints?.Clear();
        }
        /// <summary>
        ///判断关节是否包含某关节
        /// </summary>
        /// <param name="item"></param>
        /// <returns></returns>
        public bool Contains(Joint item)
        {
            return joints.Contains(item);

        }
        public void CopyTo(Joint[] array, int arrayIndex)
        {
            joints?.CopyTo(array, arrayIndex);
        }

        public IEnumerator<JointBase> GetEnumerator()
        {
            return joints.GetEnumerator();
        }
        /// <summary>
        /// 移除关节
        /// </summary>
        /// <param name="item"></param>
        /// <returns></returns>
        public bool Remove(Joint item)
        {
            return joints.Remove(item);
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return joints.GetEnumerator();
        }

        IEnumerator<Joint> IEnumerable<Joint>.GetEnumerator()
        {
            throw new NotImplementedException();
        }
    }
}
