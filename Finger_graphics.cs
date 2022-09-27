using System;
using UnityEngine;
using Assets.Scripts.dynamixel_sdk;
using System.Collections;
using System.IO;
using System.IO.Ports;


public class Finger_graphics : MonoBehaviour
{

    Rigidbody rb;

    int[] motor = new int[4];

    Vector3 proxyPos;
    Vector3 spidarPosition;
    public static Vector3 HIP;
    Vector3 currentPos;
    int bCollide_Count = 0;
    string m_strPath;

    string record = "";
    FileStream f;
    StreamWriter writer;


    public float shake_decay = 0.3f;
    public float shake_intensity = .3f;



    double stiffness = 150.0f;
    Vector3 penetration;
    float moveSpeed = 5.0f;
    Vector3 delta;
    bool doWriteData = true;

    [SerializeField]
    float time_set = 2f;

    public GameObject gm;

    //int time_flag = 1;

    float Fx = 0, Fy = 0, Fz = 0;

    public int moter2_tu = 5;
    public int operate_initial_tension = 20;
    int flag = 1;

    public float initial_N_high = 0.8f;
    public float initial_N_low = -0.4f;

    public float goal_input_force = 0;
    public int getkeyboard = 0;

    [SerializeField]
    static public int initial_boundary_flag = 0;

    public bool collision_flag = false;

    public GameObject Finger_ring_POS;
    public Gun_Wall_Control goc;

    float before_x = 0, before_y = 0, before_z = 0;

    public int hz = 1;

    bool start_flag = false;

    void Start()
    {
        for (int i = 0; i < 4; i++)
        {
            motor[i] = 0;
        }
        rb = transform.GetComponent<Rigidbody>();
        // create virtual proxy
        proxyPos = transform.position;

        before_x = HIP.x;
        before_y = HIP.y;
        before_z = HIP.z;
    }

    void Calculate_depth()      //??
    {
        currentPos = Vector3.Lerp(this.transform.position, HIP, 3.0f * Time.deltaTime);

        // Calculate penetration depth
        delta = HIP - transform.position;
        delta.Normalize();
        rb.AddForce(delta * moveSpeed * Time.deltaTime * 1.5f);
        rb.velocity = delta * moveSpeed;
        transform.position = currentPos;

        //double mag = 0;
        if (bCollide_Count == 1)
        {
            penetration = proxyPos - HIP;

            // Force - Hooke's Law (with surface normal)
            double mag = penetration.sqrMagnitude;

            //Force - Hooke's Law (with penetration vector)
            Fy = (int)(penetration.x * stiffness);
            Fz = (int)(penetration.y * stiffness);
            Fx = (int)(penetration.z * stiffness);
        }
    }

    void Update()
    {
        spidarPosition.x = SyncWrite_kinesthetic.leftFinger_position_x;
        spidarPosition.y = -(SyncWrite_kinesthetic.leftFinger_position_y);
        spidarPosition.z = SyncWrite_kinesthetic.leftFinger_position_z;

        HIP = new Vector3(spidarPosition.x, spidarPosition.y, spidarPosition.z); // get the position data from motor controller

        Calculate_depth();
    }

    void OnApplicationQuit()
    {
        if (doWriteData)
        {
            f = new FileStream(m_strPath + "data.txt", FileMode.Append, FileAccess.Write);

            writer = new StreamWriter(f, System.Text.Encoding.Unicode);

            writer.Write(record);
            writer.Close();
        }
    }
}