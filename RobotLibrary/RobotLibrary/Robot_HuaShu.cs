using Controller;
using Device;
using RobotLibraryAlgorithm;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Controls;

namespace RobotLibrary
{
    public class Robot_HuaShu:Robot
    {
        /// <summary>
        /// 变量属性 变量名称 变量解释
        /// Control_2183 Control2183 机械臂控制器
        /// SerialPortUtils SerialPort 串口、读取当前机械臂各个关节角度值
        /// </summary>
        private Control_2183 control2183;
        public Control_2183 Control2183
        {
            get { return control2183; }
            set { control2183 = value; }
        }
        private SerialPortUtils serialport;
        public SerialPortUtils SerialPort
        {
            get { return serialport; }
            set { serialport = value; }
        }
        public Robot_HuaShu() 
        {
            RobotSportsParams = new double[6, 6]
       {
             {0, 0, 1, 0, 0, 256 },
            {0, 1, 0, 60, 0, 456 },
            {0, 1, 0, 60, 0, 896 },
            {1,0 , 0, 180.5, 0, 931 },
            {0, 1, 0, 470, 0, 931 },
            {1, 0, 0, 565, 0, 931 }
        };
            RobotLimParams = new double[4, 6]
            {
           {-150,-85,-180,-180,-115,-360 },
           {150,117,60,180,115,360 },
           {-1005,-1005,0,-180,-180,-180 },
           {1005,1005,1402,180,180,180 },
            };
            control2183 = new Control_2183();
            serialport = new SerialPortUtils();
        }
        //运动学结果作为参数，做相应的运动  控制上写出父类，子类去继承
        public override void MoveControlInit()
        {
            
            //control2183.Connect2183();
            //serialport.OpenClosePort("COM3", 115200);
        }
        public override void KinematicsInit()
        {
            
        }
        public override void Command(List<string> CommandStrings)
        {
            
        }
        /// <summary>
        /// 读取串口数据:
        /// 上电获取各个关节角度编码器初始值
        /// </summary>
        /// <returns></returns>
        public double[] SerialPortOutAngles()
        {
            List<int> deviceAddresses = new List<int> { 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6 };
            serialport.LoadAddress(deviceAddresses);
            double[] angles = new double[6];
            Console.WriteLine("------------------");
            for (int i = 1; i < 7; i++)
            {
                bool single = true;
                double angle = 0;
                int numcircle = 0;
                serialport.StartPolling(single);
                Thread.Sleep(300);
                angle = serialport.Angle;
                serialport.Angle = 0;
                Console.WriteLine("angle：" + angle);
                Console.WriteLine("\n");
                single = false;
                serialport.StartPolling(single);
                Thread.Sleep(300);
                numcircle = serialport.NumCircle;
                Console.WriteLine("numcircle：" + numcircle);
                Console.WriteLine("\n");
                angles[i - 1] = Control_2183.control.InitAngle(numcircle, angle, i);
            }



            angles[0] = angles[0] - 27.702064183091167;// 2 1 3 4 6 5
            angles[1] = angles[1] - 3.2096049802147761 + 0.0102 + 0.0022;
            angles[2] = -(angles[2] + 3.5820997933253178 + 3.8846 + 0.0446);
            angles[3] = -(angles[3] - 1.2262836216633357 + 0.423 - 0.4023);
            angles[4] = -(angles[4] + 51.505606840471515 + 4.832 + 0.157);
            angles[5] = -(angles[5] - 10.550751123530128);

            //耦合补偿
            angles[5] = angles[5] + 0.0125 * angles[4];
            return angles;
        }

        public void Open2183()
        {
            if (Control_2183.control.IsConnect)
            {
                Control_2183.control.Open_servo();
            }
        }
        public void Close2183()
        {
            if (Control_2183.control.IsConnect)
            {
                Control_2183.control.Close_servo();
            }
        }
        public void Command2183Single(double[] angles,int speed)
        {
            Control_2183.control.Command_2183(angles, speed);
        }
        public void Command2183Continuous(List<Position> positions, int speed)
        {
            Control_2183.control.Command_2183(positions, speed);
        }
        
    }
}
