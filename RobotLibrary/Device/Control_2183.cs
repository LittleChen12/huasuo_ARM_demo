using RobotLibraryAlgorithm;
using System;
using System.Collections.Generic;
using System.IO;
using System.Numerics;
using System.Threading;
using System.Windows;

namespace Controller
{
    public class Control_2183
    {
        private Algorithm basicalgorithm=new Algorithm();
        private Galil.Galil control2183;
        public static Control_2183 control;
        //private double[] reductionratio = new double[6] {81.009705,100.986537,88.48931,80.033374,81.1374660,81.004865 };
        //private double[] reductionratio = 80.99350306, 101.016833, 88.51585679, 80.00136065, 80.98330481, 81.1263723};
        private double[] reductionratio = new double[6] { 81.00160241,
                            101.016833 ,
                            88.51585679,
                            80.45736841,
                            80.96710815,
                            81.1912734 };

        public bool IsConnect = false;
        public bool Isservo = false;
        private double VirAxis_k = 1000 * 1000;  //256脉冲/mm

        // 华数test
        public bool Issavefile = false;

        StreamWriter fw = null;
        public int Control2183Speed = 100;

        public void savepose(Position pose)
        {
            fw.Write(pose.Pose.Point. X + ",");
            fw.Write(pose.Pose.Point.Y + ",");
            fw.Write(pose.Pose.Point.Z + ",");
            fw.Write(pose.Pose.Rx + ",");
            fw.Write(pose.Pose.Ry + ",");
            fw.Write(pose.Pose.Rz + ",");
            fw.WriteLine();
        }

        public void opensave()
        {
            try
            {
                fw = new StreamWriter("D:\\data\\pose.csv", true);
                Issavefile = true;

            }
            catch (Exception e)
            {

                MessageBox.Show(e.Message);
            }

        }

        public void closesave()
        {
            fw.Close();
            Issavefile = false;
        }

        //
        //           StreamWriter fw = new StreamWriter(address_txt, true);

        //            for (int j = 0; j<pos.Length; j++)
        //            {
        //                fw.Write(pos[j] + " ");
        //            }
        //          fw.WriteLine();




        public Control_2183()
        {
            control2183 = new Galil.Galil();
            control = this;
        }

        public void Connect2183()
        {
            try
            {
                control2183.address = "";
                IsConnect = true;
            }
            catch (Exception)
            {

                MessageBox.Show("Connect is failed");
            }

        }

        private int[] IncrementAngles(double[] angles, bool VirAxis_IsX_Y)
        {
            int[] Incangles = new int[8];

            //!!!!!!!!!!!!!!!
            //for (int i = 0; i < 6; i++)
            //{

            //    Incangles[i] = (int)((angles[i] - MainWindow.mainwindow.OldAngles[i])
            //        / 360 * 10000 * reductionratio[i]);
            //}


            double[] Intangles = new double[6];
            for (int i = 0; i < 6; i++)
            {
                //!!!!!!!!!!!
                //Intangles[i] = (angles[i] - MainWindow.mainwindow.OldAngles[i]);
            }
            //耦合补偿
            Intangles[5] = Intangles[5] - (int)(0.0125 * Intangles[4]);

            for (int i = 0; i < 6; i++)
            {

                Incangles[i] = (int)(Intangles[i] / 360 * 10000 * reductionratio[i]);
            }

            //!!!!!!!!!!!!!!!!!!!!!!!!!!!
            //Vector2 VirAxis = position1.DisplacementInTool(MainWindow.mainwindow.OldAngles, angles);

            //Incangles[6] = (int)(VirAxis_k * VirAxis.X);
            //Incangles[7] = (int)(VirAxis_k * VirAxis.Y);


            //MainWindow.mainwindow.OldAngles = angles;
            return Incangles;
        }

        // angles减去当前位姿得角度，并转换到2183控制的脉冲个数
        private int[] IncrementAngles(double[] angles)
        {
            int[] Incangles = new int[6];
            for (int i = 0; i < Incangles.Length; i++)
            {
                ///!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                ////Incangles[i] = (int)((angles[i] - MainWindow.mainwindow.OldAngles[i])
                    ///// 360 * 10000 * reductionratio[i]);
            }
            //!!!!!!!!!!!!!!!!!!!!!!!!!
            //MainWindow.mainwindow.OldAngles = angles;
            return Incangles;
        }

        // 关闭使能
        public void Close_servo()
        {
            //control2183.command("MO", "\r", ":", true); //接收使能
            control2183.command("STS", "\r", ":", true);  // 停止运动
            Thread.Sleep(1000);


            control2183.command("CB1", "\r", ":", true); // 关闭抱闸

            Thread.Sleep(200);
            ////control2183.command("LE", "\r", ":", true); //直线插补结束
            control2183.command("MO", "\r", ":", true); //结束使能
        }

        // 打开使能
        public void Open_servo()
        {

            control2183.command("SH", "\r", ":", true); //运动使能

            Thread.Sleep(600);
            // 
            control2183.command("SB1", "\r", ":", true); //打开抱闸

            //if (Isservo)
            //{
            //    //control2183.command("MO", "\r", ":", true); //接收使能
            //    control2183.command("ST", "\r", ":", true);  // 停止运动
            //    control2183.command("CB1", "\r", ":", true); // 关闭抱闸
            //    Thread.Sleep(30);

            //    control2183.command("LE", "\r", ":", true); //直线插补结束
            //    control2183.command("MO", "\r", ":", true); //结束使能

            //    // Thread.Sleep(2000);

            //    //control2183.command("MO", "\r", ":", true); //运动使能

            //    Isservo = false;
            //}
            //else
            //{
            //    control2183.command("SH", "\r", ":", true); //运动使能

            //    Thread.Sleep(1000);
            //    // 
            //    control2183.command("SB1", "\r", ":", true); //打开抱闸
            //    Isservo = true;
            //}

            try
            {
                control2183.command("LMABCDEFGH", "\r", ":", true);
                //control2183.command("LMABCDEF", "\r", ":", true); //直线插补
                //control2183.command("LE", "\r", ":", true); //直线插补结束

                control2183.command("VA250000", "\r", ":", true); //第1、2、3、4、5、6轴旋转加速度
                control2183.command("VD250000", "\r", ":", true); //第1、2、3、4、5、6轴旋转减速度

                //control2183.command("VA2500", "\r", ":", true); //第1、2、3、4、5、6轴旋转加速度
                //control2183.command("VD2500", "\r", ":", true); //第1、2、3、4、5、6轴旋转减速度
                //control2183.command("BGS", "\r", ":", true); //开始运动
            }
            catch (Exception)
            {
                //MessageBox.Show("操作频繁");
            }
        }

        public void Command_speed(int speed)
        {
            try
            {//820
                control2183.command(string.Format("VS{0}", speed * 2000), "\r", ":", true); //第1、2、3、4、5、6轴旋转速度(脉冲数/秒)
                                                                                            //control2183.command("BGS", "\r", ":", true); //开始运动


            }
            catch (Exception)
            {
                MessageBox.Show("速度设置失败");
            }
        }




        public void Command_2183(double[] angles, int speed)
        {
            //return;
            //var oldangles = MainWindow.mainwindow.OldAngles;
            //string locationAngles = string.Format("PR{0},{1},{2},{3},{4},{5}", oldangles[0], oldangles[1], oldangles[2], oldangles[3], oldangles[4], oldangles[5]);
            //control2183.command(locationAngles, "\r", ":", true);
            try
            {
                //control2183.command("LMABCDEF", "\r", ":", true); //直线插补
                //int[] Incangles = new int[6];
                //Incangles = IncrementAngles(angles);
                //string li = string.Format("LI{0},{1},{2},{3},{4},{5}", -Incangles[2], Incangles[0], -Incangles[4], -Incangles[1], -Incangles[5], -Incangles[3]);
                //control2183.command(li, "\r", ":", true); //直线插补

                int[] Incangles = new int[8];
                Incangles = IncrementAngles(angles, true);
                string li = string.Format("LI{0},{1},{2},{3},{4},{5},{6},{7}", Incangles[7], Incangles[1], -Incangles[2], Incangles[6], -Incangles[4], -Incangles[5], -Incangles[3], -Incangles[0]);
                control2183.command(li, "\r", ":", true); //直线插补


                //control2183.command("LE", "\r", ":", true); //直线插补结束
                //control2183.command("SH", "\r", ":", true); //运动使能
                ////control2183.command("VA15000", "\r", ":", true); //第1、2、3、4、5、6轴旋转加速度
                ////control2183.command("VD15000", "\r", ":", true); //第1、2、3、4、5、6轴旋转减速度
                //control2183.command(string.Format("VS{0}", 100000), "\r", ":", true); //第1、2、3、4、5、6轴旋转速度(脉冲数/秒)
                //if (fl)
                //{
                //    
                //}
                //fl = false;
                control2183.command("BGS", "\r", ":", true); //开始运动
            }
            catch (Exception e)
            {
                Console.WriteLine(e.Message);
            }

        }
        public void Command_2183(List<Position> positions, int speed)
        {
            //return;
            try
            {
                //control2183.command("LMABCDEFGH", "\r", ":", true); //直线插补
                //control2183.command("LMABCDEF", "\r", ":", true); //直线插补
                int[] Incangles = new int[8];
                foreach (var position in positions)
                {
                    //double[] angles = position.Joints.Joints.ToArray();
                    //Incangles = IncrementAngles(angles);
                    //string li = string.Format("LI{0},{1},{2},{3},{4},{5}", -Incangles[2], Incangles[0], -Incangles[4], -Incangles[1], -Incangles[5], -Incangles[3]);
                    //control2183.command(li, "\r", ":", true); //直线插补
                    double[] angles = position.Joints.Joints.ToArray();

                    Incangles = IncrementAngles(basicalgorithm.RadToAngle(angles), true);
                    string li = string.Format("LI{0},{1},{2},{3},{4},{5},{6},{7}", Incangles[7], Incangles[1], -Incangles[2], Incangles[6], -Incangles[4], -Incangles[5], -Incangles[3], -Incangles[0]);
                    control2183.command(li, "\r", ":", true); //直线插补

                    // 华数test!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    if (Issavefile)
                    {
                        //position1 startPosition = new position1(angles, MainWindow.mainwindow.GripToTool);
                        //savepose(startPosition);
                    }

                    //Thread.Sleep(30);
                }
                //control2183.command("LE", "\r", ":", true); //直线插补结束
                //control2183.command(string.Format("VS{0}", speed * 820), "\r", ":", true);

                control2183.command("BGS", "\r", ":", true); //开始运动
            }
            catch (Exception e)
            {
                Console.WriteLine("line309" + e.Message);
            }

        }


        public double InitAngle(int numcircle, double angle, int i)
        {
            if (numcircle < 32767 && numcircle >= 0)
            {
                angle = (numcircle * 360 + angle) / reductionratio[i - 1];
            }
            else
            {
                angle = ((numcircle - 65536) * 360 + angle) / reductionratio[i - 1];
                //angle = (numcircle * 360 + angle) / reductionratio[i-1] - 360;
            }

            return angle;
        }


    }


}

