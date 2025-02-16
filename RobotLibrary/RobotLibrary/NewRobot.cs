using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RobotLibrary
{
    public class NewRobot : Robot
    {
        
        public NewRobot() 
        {
            RobotSportsParams = new double[6, 6];
            RobotLimParams = new double[4, 6];
        }

        public override void MoveControlInit()
        {
           
        }
        public override void KinematicsInit()
        {
            
        }
        public override void Command(List<string> CommandStrings)
        {
            
        }
    }
}
