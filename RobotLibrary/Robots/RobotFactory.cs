using RobotLibrary;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Robots
{
    public class RobotFactory
    {
        public static Robot CreateRobot(string RobotType)
        {
            switch (RobotType)
            {
                case "HuaShu":
                    return new Robot_HuaShu();
                case "AnChuan":
                    return new Robot_Anchuan();
                case "Roke":
                    return new Robot_Roke();
                case "NewRobot":
                    return new NewRobot();
                default:
                    return null;
            }
        }
    }
}
