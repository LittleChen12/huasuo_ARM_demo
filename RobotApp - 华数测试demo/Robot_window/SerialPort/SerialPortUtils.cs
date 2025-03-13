using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Diagnostics;
using System.Threading;
using System.Windows;

namespace Robot_window
{
    internal class SerialPortUtils
    {
        public static string[] GetPortNames()
        {
            return SerialPort.GetPortNames();
        }


        public static SerialPort SerialPort = null;
        private List<int> DeviceAddresses;
        private int currentDeviceIndex = 0;
        private const int PollingInterval = 1000;

        private double anglereceve;
        int[] angle = new int[3];

        public double Angle
        {
            get { return anglereceve; }
            set { anglereceve = value; }
        }
        private int numcircle;

        public int NumCircle
        {
            get { return numcircle; }
            set { numcircle = value; }
        }


        public SerialPort OpenClosePortYaLi(string comName, int baud)
        {
            //串口未打开
            if (SerialPort == null || !SerialPort.IsOpen)
            {
                SerialPort = new SerialPort();
                //串口名称
                SerialPort.PortName = comName;
                //波特率
                SerialPort.BaudRate = baud;
                //数据位
                SerialPort.DataBits = 8;
                //停止位
                SerialPort.StopBits = StopBits.One;
                //校验位
                SerialPort.Parity = Parity.None;
                //打开串口
                SerialPort.Open();

                //串口数据接收事件实现
                SerialPort.DataReceived += new SerialDataReceivedEventHandler(SynReceiveData);


                return SerialPort;
            }
            //串口已经打开
            else
            {
                SerialPort.Close();
                return SerialPort;
            }
        }


        private void SynReceiveData(object sender, SerialDataReceivedEventArgs e)
        {
            char[] receive_char = new char[9];
            string data;

            while (true)
            {
                try
                {
                    data = SerialPort.ReadLine();
                }
                catch (Exception ex)
                {
                    break;
                }
                if (data.Length == 9)
                {
                    receive_char = data.ToArray();
                    if (receive_char[0] == 'a')
                    {

                        angle[0] = (short)((short)(receive_char[1] << 8) + (short)receive_char[2]);  //Pitch
                        angle[1] = (short)((short)(receive_char[4] << 8) + (short)receive_char[5]);  //Roll
                        angle[2] = (short)((short)(receive_char[7] << 8) + (short)receive_char[8]);  // Yaw 
                                                                                                     //myGLView.X = (int)angle[0];   
                                                                                                     //myGLView.Y = (int)angle[1];
                                                                                                     //myGLView.Z = (int)angle[2];
                                                                                                     //SetText(angle[0].ToString(), angle[1].ToString(), angle[2].ToString());  
                    }
                }
            }
        }

        public SerialPort OpenClosePort(string comName, int baud)
        {
            //串口未打开
            if (SerialPort == null || !SerialPort.IsOpen)
            {
                SerialPort = new SerialPort();
                //串口名称
                SerialPort.PortName = comName;
                //波特率
                SerialPort.BaudRate = baud;
                //数据位
                SerialPort.DataBits = 8;
                //停止位
                SerialPort.StopBits = StopBits.One;
                //校验位
                SerialPort.Parity = Parity.None;
                //打开串口
                SerialPort.Open();

                //串口数据接收事件实现
                SerialPort.DataReceived += new SerialDataReceivedEventHandler(ReceiveData);


                return SerialPort;
            }
            //串口已经打开
            else
            {
                SerialPort.Close();
                return SerialPort;
            }
        }
        public void LoadAddress(List<int> Address)
        {
            DeviceAddresses = Address;
        }

        public void StartPolling(bool single)
        {
            new Thread(() =>
            {
                PollDevice(single);
                Thread.Sleep(PollingInterval);
            }).Start();
        }

        private void PollDevice(bool single)
        {
            if (currentDeviceIndex >= DeviceAddresses.Count)
                currentDeviceIndex = 0; // 如果需要循环轮询，重置索引         
            int address = DeviceAddresses[currentDeviceIndex];
            byte[] requestData = CreateRequest(address, single); // 创建请求数据，这里只是一个示例方法，需要您实现  
            SendData(requestData); // 发送请求到设备  
            currentDeviceIndex++; // 移动到下一个设备  
        }

        private byte[] CreateRequest(int address, bool single)
        {
            byte[] command = new byte[8];
            command[0] = (byte)address;
            command[1] = 0x03;
            command[2] = 0x20;
            if (single)
                command[3] = 0x06;
            else
                command[3] = 0x00;
            command[4] = 0x00;
            command[5] = 0x02;
            ushort crc = CalculateCRC(command, 6); // 计算CRC校验
            command[6] = (byte)(crc & 0xFF); // CRC低字节
            command[7] = (byte)(crc >> 8); // CRC高字节

            return command;
        }
        private static ushort CalculateCRC(byte[] data, int length)
        {
            // 计算Modbus RTU CRC校验
            ushort crc = 0xFFFF;
            for (int i = 0; i < length; i++)
            {
                crc ^= data[i];
                for (int j = 0; j < 8; j++)
                {
                    if ((crc & 1) != 0)
                    {
                        crc >>= 1;
                        crc ^= 0xA001;
                    }
                    else
                    {
                        crc >>= 1;
                    }
                }
            }
            return crc;
        }





        // 接收返回的数据
        public void ReceiveData(object sender, SerialDataReceivedEventArgs e)
        {
            SerialPort _SerialPort = (SerialPort)sender;
            string sss;
            int _bytesToRead = _SerialPort.BytesToRead;
            //byte[] recvData = new byte[_bytesToRead];
            byte[] recvData = new byte[9];
            _SerialPort.Read(recvData, 0, _bytesToRead);
            if (recvData[0] != 0)
            {
                //单圈处理
                byte[] reversedBytes = new byte[] { recvData[6], recvData[5], recvData[4], recvData[3] };
                // 将调整后的字节数组转换为32位有符号整数
                uint result = BitConverter.ToUInt32(reversedBytes, 0);
                double angle = result / Math.Pow(2, 17) * 360;
                Angle = angle;
                //Console.WriteLine("joint" + recvData[0]);
                //Console.WriteLine(angle);
                //Console.WriteLine("\n");
                //Console.WriteLine("------------------");
                //多圈处理
                NumCircle = (int)result;
                //Console.WriteLine(numcircle);
            }

        }



        public static bool SendData(byte[] data)
        {
            if (SerialPort != null && SerialPort.IsOpen)
            {
                SerialPort.Write(data, 0, data.Length);
                //Console.WriteLine("发送数据：" + data);
                return true;
            }
            else
            {
                return false;
            }
        }

        public static bool SendData(string data)
        {
            if (SerialPort != null && SerialPort.IsOpen)
            {
                SerialPort.Write(data);
                Console.WriteLine("发送数据：" + data);
                return true;
            }
            else
            {
                return false;
            }
        }

    }


    public class ByteArrayConvert
    {

        /// <summary>
        /// byte数组转string
        /// </summary>
        /// <param name="data"></param>
        /// <returns></returns>
        public static string byteArrayToString(byte[] data)
        {


            return Encoding.Default.GetString(data);
        }



        /// <summary>
        /// string转 byte数组
        /// </summary>
        /// <param name="data"></param>
        /// <returns></returns>
        public static byte[] stringToByteArray(string data)
        {

            return Encoding.Default.GetBytes(data);
        }





        /// <summary>
        /// byte数组转16进制字符串
        /// </summary>
        /// <param name="data"></param>
        /// <returns></returns>
        public static string byteArrayToHexString(byte[] data)
        {
            StringBuilder builder = new StringBuilder();
            for (int i = 0; i < data.Length; i++)
            {
                builder.Append(string.Format("{0:X2} ", data[i]));
            }
            return builder.ToString().Trim();
        }

        /// <summary>
        /// 16进制字符串转byte数组
        /// </summary>
        /// <param name="data"></param>
        /// <returns></returns>
        public static byte[] hexStringToByteArray(string data)
        {
            string[] chars = data.Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
            byte[] returnBytes = new byte[chars.Length];
            //逐个字符变为16进制字节数据
            for (int i = 0; i < chars.Length; i++)
            {
                returnBytes[i] = Convert.ToByte(chars[i], 16);
            }
            return returnBytes;
        }



        /// <summary>
        /// byte数组转10进制字符串
        /// </summary>
        /// <param name="data"></param>
        /// <returns></returns>
        public static string byteArrayToDecString(byte[] data)
        {

            StringBuilder builder = new StringBuilder();
            for (int i = 0; i < data.Length; i++)
            {
                builder.Append(data[i] + " ");
            }
            return builder.ToString().Trim();
        }

        /// <summary>
        /// 10进制字符串转byte数组
        /// </summary>
        /// <param name="data"></param>
        /// <returns></returns>
        public static byte[] decStringToByteArray(string data)
        {

            string[] chars = data.Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
            byte[] returnBytes = new byte[chars.Length];
            //逐个字符变为10进制字节数据
            for (int i = 0; i < chars.Length; i++)
            {
                returnBytes[i] = Convert.ToByte(chars[i], 10);
            }
            return returnBytes;
        }




        /// <summary>
        /// byte数组转八进制字符串
        /// </summary>
        /// <param name="data"></param>
        /// <returns></returns>
        public static string byteArrayToOtcString(byte[] data)
        {

            StringBuilder builder = new StringBuilder();
            for (int i = 0; i < data.Length; i++)
            {
                builder.Append(Convert.ToString(data[i], 8) + " ");
            }
            return builder.ToString().Trim();
        }

        /// <summary>
        /// 八进制字符串转byte数组
        /// </summary>
        /// <param name="data"></param>
        /// <returns></returns>
        public static byte[] otcStringToByteArray(string data)
        {

            string[] chars = data.Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
            byte[] returnBytes = new byte[chars.Length];
            //逐个字符变为8进制字节数据
            for (int i = 0; i < chars.Length; i++)
            {
                returnBytes[i] = Convert.ToByte(chars[i], 8);
            }
            return returnBytes;
        }





        /// <summary>
        /// 二进制字符串转byte数组
        /// </summary>
        /// <param name="data"></param>
        /// <returns></returns>
        public static byte[] binStringToByteArray(string data)
        {

            string[] chars = data.Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
            byte[] returnBytes = new byte[chars.Length];
            //逐个字符变为2进制字节数据
            for (int i = 0; i < chars.Length; i++)
            {
                returnBytes[i] = Convert.ToByte(chars[i], 2);
            }
            return returnBytes;
        }



        /// <summary>
        /// byte数组转二进制字符串
        /// </summary>
        /// <param name="data"></param>
        /// <returns></returns>
        public static string byteArrayToBinString(byte[] data)
        {
            StringBuilder builder = new StringBuilder();
            for (int i = 0; i < data.Length; i++)
            {
                builder.Append(Convert.ToString(data[i], 2) + " ");
            }
            return builder.ToString().Trim();
        }
    }
}
