using RobotLibrary;
using RobotLibraryAlgorithm.KinematicsAlgorithm;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Robots
{
    public class KinematicsFctory
    {
        public static Kinematics? CreateKinematics(string RobotType)
        {
            switch (RobotType)
            {
                case "HuaShu":
                    return new KinematicsHuaShu();
                case "AnChuan":
                    return new KinematicsAnChuan();
                case "Roke":
                    return new KinematicsRoke ();
                case "NewRobot":
                    return new KinematicsNewRobot();
                default:
                    return null;
            }
        }
    }
}
