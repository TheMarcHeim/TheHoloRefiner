using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Net;
using System.Net.Sockets;

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
                    byte[] ByteBuffer = new byte[4];
                    conn.Receive(ByteBuffer, 4, SocketFlags.None);
                    int numberOfPics = BitConverter.ToInt32(ByteBuffer, 0);
                    for(int i = 0; i < numberOfPics; i++)
                    {
                        ByteBuffer = new byte[260];
                        conn.Receive(ByteBuffer, 260, SocketFlags.None);
                        double[] matrixValues=new double[32];
                        for(int ii = 0; ii < 32; ii++)
                        {
                            matrixValues[ii] = BitConverter.ToDouble(ByteBuffer, ii * 8);
                        }
                        //write MatrixValues to file
                        //TODOOOOOOOOOOOOOOOOOOOOOOO
                        
                        //picture size
                        int picSize = BitConverter.ToInt32(ByteBuffer, 256);
                        ByteBuffer = new byte[picSize];
                        conn.Receive(ByteBuffer, picSize, SocketFlags.None);

                        //write pic to file
                        //TODOOOOOOOOOOOOOOOOOOOOOOO
                    }

                    //load obj
                    ByteBuffer = new byte[4];
                    conn.Receive(ByteBuffer, 4, SocketFlags.None);
                    int objSize = BitConverter.ToInt32(ByteBuffer, 4);
                    ByteBuffer = new byte[objSize];
                    conn.Receive(ByteBuffer, objSize, SocketFlags.None);
                    //save obj file
                    //TODOOOOO
                    
                    //
                }

            }
            catch
            {
                Console.WriteLine("Error");
            }
        }
    }
}
