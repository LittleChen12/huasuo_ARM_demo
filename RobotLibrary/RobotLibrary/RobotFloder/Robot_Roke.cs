using RobotLibrary;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using xCoreSDK_CSharp;

namespace Robots
{
    public class Robot_Roke:Robot
    {
        //络石机器人
        public Robot_Roke()
        {
            // 机器人参数 前三个参数是旋转轴xyz, 后三个参数是旋转中心坐标
            RobotSportsParams = new double[6, 6]
            {{0, 0, 1, 0, 0, 296 },
               {0, 1, 0, 0, 0, 296 },
               {0, -1, 0, 0, 0, 296+490 },
               {0,0 , 1, 0, 0, 296+490+360 },
               {0, -1, 0, 0, -150, 296+490+360 },
               {0, 0, 1, 0, -150, 296+490+360+127 }};
            //XYZABC
            RobotLimParams = new double[4, 6]
            {
               {-150,-85,-180,-180,-115,-360 },
               {150,117,60,180,115,360 },
               {-1005,-1005,0,-180,-180,-180 },
               {1005,1005,1402,180,180,180 }
            };
    }
        public override void MoveControlInit()
        {
            try
            {
                // might throw
                Program.xmate.connect("192.168.1.160");
            }
            catch (Exception e)
            {
                Console.WriteLine(e.ToString());
                return;
            }
        }
        public void Move()
        {
            // 执行示例
            Program.xmate.my_move();
        }
        public override void KinematicsInit()
        {
           
        }
        public override void Command(List<string> CommandStrings)
        {
           
        }
    }
}
