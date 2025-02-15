using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Documents;

namespace RobotLibraryAlgorithm
{

    //公共属性是6个角度
    //方法是迭代角度。给出末端角度，和当前位置角度，t是迭代速率
    // 关节角度类
    public class JointPosition
    {

        //ICollection
        private List<double> joints = new List<double>();
        private object root;
        // public List<double> joints = new List<double>();

        public List<double> Joints
        {
            get { return joints; }
        }



        public JointPosition(List<double> _joints)
        {
            joints = _joints;
            
        }

        public JointPosition(JointPosition _joints)
        {
            joints = _joints.joints;

        }

        public JointPosition(double[] _joints)
        {
            joints = _joints.ToList();

        }

        //默认构造函数
        public JointPosition() 
        { 
            
        }



        /*
         * 功能描述：[0,2pi]的连续 线性插值
        * 输入：
        * start：起始角度（单位：rad）
        * end ：终止角度（单位：rad）
        * t ：[0,1] 觉定插值的位置
        * 
        * 输出：double[6] 插值后的关节数据
        */
        public static double[] Lerp(JointPosition start, JointPosition end, double t)
        {//
            double[] ret = new double[start.joints.Count()];
            for(int i = 0; i < start.joints.Count(); i++)
            {
                ret[i] = LerpAngle(start.joints[i], end.joints[i], t);
            }
            return ret;
        }

        // 连续角度的线性插值
        private static double LerpAngle(double startAngle, double endAngle, double t)
        {
            double delta = WrapAngle(endAngle - startAngle);
            return WrapAngle(startAngle + delta * t);
        }

        // 确保angle在[0,2pi]
        private static double WrapAngle(double angle)
        {
            while (angle > Math.PI) angle -= 2 * Math.PI;
            while (angle < -Math.PI) angle += 2 * Math.PI;
            return angle;
        }

        public void CopyTo(Array array, int index)
        {
            throw new NotImplementedException();
        }

        public IEnumerator GetEnumerator()
        {
            throw new NotImplementedException();
        }
    }
}
