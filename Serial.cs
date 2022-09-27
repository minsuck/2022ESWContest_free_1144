using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.IO.Ports;
using System;
using System.Threading;//thread 사용을 위해
using System.Threading.Tasks;



public class Serial : MonoBehaviour
{
    private SerialPort stream;
    private string rcv_data = null;     // save data which is from MCU

    Thread HWThread;    // Thread


    // variable which is separate fsr sensor value
    private int fsr_1 = 0;
    private int fsr_2 = 0;

    // variable which is decide aim mode & shoot
    public static bool Aim_mode = false;
    public static bool Shoot = false;

    void Start()
    {
        // Start Serial Communication
        stream = new SerialPort("COM3", 115200, Parity.None, 8, StopBits.One);
        //stream.Open();

        HWThread = new Thread(new ThreadStart(Serial_communication));   // This Thread is Update function "Serial_Communication"
        HWThread.Start();

    }

    void Update()
    {
    }

    private void tryConnectPort()
    {
        stream = new SerialPort("COM3", 115200, Parity.None, 8, StopBits.One);
        //stream.ReadTimeout = 2;
        stream.Open();


    }


    private void Serial_communication()     // have to check every frame to communication
    {
        try
        {
            while (HWThread.IsAlive)
            {
                if (!stream.IsOpen)     // if serial is not open, try to connect again
                {
                    tryConnectPort();
                }
                else
                {
                    rcv_data = stream.ReadLine();
                    separateData(rcv_data);
                }
            }
        }

        catch (Exception e)
        {
        }
    }


    private void separateData(string data)  // separate received data
    {
        char[] sep = { '$', ',', '#' };
        string[] tmp = rcv_data.Split(sep, StringSplitOptions.RemoveEmptyEntries);

        fsr_1 = int.Parse(tmp[0]);
        fsr_2 = int.Parse(tmp[1]);

        //Debug.Log(fsr_1 + ", " + fsr_2);


        if (fsr_1 > 4000)
        {
            Shoot = true;
        }
        else
        {
            Shoot = false;
        }

        if (fsr_2 > 800)
        {
            Aim_mode = true;
        }
        else
        {
            Aim_mode = false;
        }

        //Debug.Log(fsr_1 + ", " + fsr_2);
    }


    private void OnApplicationQuit()
    {
        stream.Close();
        HWThread.Abort();
    }
}