using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RobotLibraryAlgorithm
{
    public class JointPosition
    {
        private List<double> joints;
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
            joints = _joints.Joints;

        }

        public JointPosition(double[] _joints)
        {
            joints = _joints.ToList();

        }
        //默认构造函数
        public JointPosition()
        {
            joints = new List<double>();
        }
    }
}
