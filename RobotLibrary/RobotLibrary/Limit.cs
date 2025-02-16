using System;
using System.Collections.Generic;
using System.Linq;
using System.Security.Permissions;
using System.Text;
using System.Threading.Tasks;

namespace RobotLibrary
{
    public  class JointLimit
    {
        public int JointAngleMinium { get; set; }
        public int JointAngleMaxium { get;set; }
       
    }
    public class RobotLimit
    {
        public double RobotXMinium { get; set; }
        public double RobotXMaxium { get; set; }
        public double RobotYMinium { get; set; }
        public double RobotYMaxium { get; set; }
        public double RobotZMinium { get; set; }
        public double RobotZMaxium { get; set; }
        public double RobotAMinium { get; set; }
        public double RobotAMaxium { get; set; }
        public double RobotBMinium { get; set; }
        public double RobotBMaxium { get; set; }
        public double RobotCMinium { get; set; }
        public double RobotCMaxium { get; set; }

    }
}
