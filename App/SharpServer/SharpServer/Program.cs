using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Net;
using System.Net.Sockets;
using System.IO;

namespace SharpServer
{
    class Program
    {
        static void Main(string[] args)
        {
            IPHostEntry ipHostInfo = Dns.Resolve(Dns.GetHostName());
            IPAddress ipAddress = ipHostInfo.AddressList[0];
            IPEndPoint localEndPoint = new IPEndPoint(ipAddress, 11000);

            Socket mylistener = new Socket(AddressFamily.InterNetwork,
            SocketType.Stream, ProtocolType.Tcp);


            try
            {

                mylistener.Bind(localEndPoint);
                mylistener.Listen(100);
                Console.WriteLine("Waiting for a connection...");
                while (true)
                {
                    //get connected socket
                    Socket conn = mylistener.Accept();
                    BinaryWriter writer;
                    //format:
                    /*first 4 bytes: int number of pictures
                     *for each picture:
                     *      256 bytes: calibration matrix row wise
                     *      4 bytes: int that specifies length of png file
                     *      actual png
                     *
                     *4 bytes: int that specifies length of obj file
                     * actual obj file
                     * 
                     * 
                     * 
                     * 
                     * 
                     * 
                     * 
                     * 
                     * 
                     * */
                    //receive number of 
                    while (conn.Connected)
                    {
                        int numberOfPics = 0;
                        try
                        {
                            //TAKE DATA
                            byte[] ByteBuffer = new byte[4];
                            conn.Receive(ByteBuffer, 4, SocketFlags.None);
                            numberOfPics = BitConverter.ToInt32(ByteBuffer, 0);
                            for (int i = 0; i < numberOfPics; i++)
                            {
                                ByteBuffer = new byte[260];
                                conn.Receive(ByteBuffer, 260, SocketFlags.None);
                                double[] matrixValues = new double[32];
                                for (int ii = 0; ii < 32; ii++)
                                {
                                    matrixValues[ii] = BitConverter.ToDouble(ByteBuffer, ii * 8);
                                }
                                //write MatrixValues to file
                                //Write byteValues to file
                                writer = new BinaryWriter(File.Open("view" + i + ".matr", FileMode.Create));
                                for (int iii = 0; iii < 256; iii++)
                                    writer.Write(ByteBuffer[iii]);
                                writer.Close();


                                //picture size
                                int picSize = BitConverter.ToInt32(ByteBuffer, 256);
                                ByteBuffer = new byte[picSize];
                                conn.Receive(ByteBuffer, picSize, SocketFlags.None);

                                //write pic to file
                                writer = new BinaryWriter(File.Open("view" + i + ".png", FileMode.Create));
                                writer.Write(ByteBuffer);
                                writer.Close();
                            }

                            //load obj
                            ByteBuffer = new byte[4];
                            conn.Receive(ByteBuffer, 4, SocketFlags.None);
                            int objSize = BitConverter.ToInt32(ByteBuffer, 4);
                            ByteBuffer = new byte[objSize];
                            conn.Receive(ByteBuffer, objSize, SocketFlags.None);
                            //save obj file
                            writer = new BinaryWriter(File.Open("unrefined.obj", FileMode.Create));
                            writer.Write(ByteBuffer);
                            writer.Close();
                        }
                        catch(Exception e)
                        {
                            Console.WriteLine("Error while receiving: " + e.ToString());
                        }
                        //WAIT FOR COMPUTATION
                        while (!File.Exists("refined.obj")) Thread.Yield();
                        try
                        {
                            //send the object back
                            BinaryReader reader = new BinaryReader(File.Open("refined.obj", FileMode.Open));
                            Byte[] objData = reader.ReadBytes(int.MaxValue);
                            conn.Send(BitConverter.GetBytes(objData.Length));
                            conn.Send(objData);

                            //delete all the data
                            for(int iii = 0; iii<numberOfPics; iii++)
                            {
                                File.Delete("view" + iii + ".png");
                                File.Delete("view" + iii + ".matr");
                            }
                            File.Delete("unrefined.obj");
                            File.Delete("refined.obj");
                        }
                        catch (Exception e)
                        {
                            Console.WriteLine("Error while sending: " + e.ToString());
                        }
                    }
                }
            }
            catch
            {
                Console.WriteLine("Error");
            }
        }
    }
}
