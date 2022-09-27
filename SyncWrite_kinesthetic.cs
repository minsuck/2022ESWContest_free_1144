using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO.Ports;
using System.IO;
using System.Text;
using UnityEngine.UI;
using System.Runtime.InteropServices;
using Assets.Scripts.dynamixel_sdk;
using System.Diagnostics;
using Debug = UnityEngine.Debug;
using System.Timers;

public class SyncWrite_kinesthetic : MonoBehaviour
{
    // ************************************************  Dynamixel initialize  ************************************************
    #region Dynamixel Value Initialize

    // Control table address
    public const int ADDR_PRO_TORQUE_ENABLE = 64;                  
    public const int ADDR_PRO_GOAL_POSITION = 102;                 
    public const int ADDR_PRO_PRESENT_POSITION = 132;                 

    // Data Byte Length
    public const int LEN_PRO_GOAL_POSITION = 2;
    public const int LEN_PRO_PRESENT_POSITION = 4;

    // Protocol version
    public const int PROTOCOL_VERSION = 2;                  

    // Default setting
    public const int DXL1_ID = 1;                   // Dynamixel ID: 1
    public const int DXL2_ID = 2;                   // Dynamixel ID: 2
    public const int DXL3_ID = 3;                   // Dynamixel ID: 3
    public const int DXL4_ID = 4;                   // Dynamixel ID: 4
    public const int DXL5_ID = 5;                   // Dynamixel ID: 1
    public const int DXL6_ID = 6;                   // Dynamixel ID: 2
    public const int DXL7_ID = 7;                   // Dynamixel ID: 3
    public const int DXL8_ID = 8;                   // Dynamixel ID: 4
    public const int BAUDRATE = 4500000;

    // Communication Settings
    public string DEVICENAME_LEFTCONTROL = "COM4";           
    public string DEVICENAME_RIGHTCONTROL = "COM6";
    public const byte ESC_ASCII_VALUE = 0x1b;
    public const byte ASCII_VALUE_1 = 0x31;
    public const int COMM_SUCCESS = 0;                   // Communication Success result value
    public const int COMM_TX_FAIL = -1001;               // Communication Tx Failed

    //  torque
    public const int TORQUE_ENABLE = 1;                   // Value for enabling the torque
    public const int TORQUE_DISABLE = 0;                   // Value for disabling the torque
    public const int DXL_MOVING_STATUS_THRESHOLD = 20;                  // Dynamixel moving status threshold
    static int dxl_initial_torque = 5;

    // port num
    static int port_leftControl = 0;
    static int write_LeftControl = 0;
    static int read_LeftControl = 0;
    static int port_rightControl = 0;
    static int write_RightControl = 0;

    // Ring object Position Value
    public static float wire_angle1 = 0f, wire_angle2 = 0f, wire_angle3 = 0f, wire_angle4 = 0f;
    static float wire_length1 = 0f, wire_length2 = 0f, wire_length3 = 0f, wire_length4 = 0f;
    static float frame_line_length = 420f;
    public static float leftFinger_position_x = 0f, leftFinger_position_y = 0f, leftFinger_position_z = 0f;
    static int init_length1 = 0, init_length2 = 0, init_length3 = 0, init_length4 = 0;

    // ETC Value
    static int dxl_comm_result = COMM_TX_FAIL;                                  
    static int dxl_comm_result2 = COMM_TX_FAIL;
    static bool dxl_addparam_result = false;  
    static bool dxl_addparam_result2 = false;  

    public static Int32 dxl1_present_position = 0, dxl2_present_position = 0, dxl3_present_position = 0, dxl4_present_position = 0;
    #endregion
    private static bool _isRight = true;
    // ************************************************  Dynamixel initialize  ************************************************






    //***********************************************  Timers (for thread)  *************************************************
    Timer timer;
    int timer_start = 0;
    int timer_count = 0;
    //***********************************************  Timers (for thread)  *************************************************






    // ************************************************  Kinesthetic feedback (variable) ************************************************
    //Motor tension
    public int defaultTension = 8;
    public int maxTension = 250;
    public int minTension = 8;

    //Timer mode
    int time_flag = 1;

    public bool input_force_flag = false;
    public bool collision_flag = false;

    // Target directional force
    static float N_to_input = 0.0161f;
    public float goal_N_high = 0f;
    public float goal_N_low = 0f;
    public float goal_N_right = 0f;
    public float goal_N_left = 0f;
    public float goal_N_forward = 0f;
    public float goal_N_backward = 0f;


    //moter location
    static float[,] moter = new float[9, 3] {{0, 0, 0},     
                                    { -12, 12, 12 },
                                    { 12, 12, -12 },
                                    { -12, -12, -12 },
                                    { 12, -12, 12 },
                                    { -12, 12, -12 },
                                    { 12, -12, -12 },
                                    { 12, 12, 12 },
                                    { -12, -12, 12 }};
    // vector which is TargetObject to moters
    float[,] MO_to_moter = new float[9, 3] { { 0, 0, 0},
                                            { 0, 0, 0},
                                            { 0, 0, 0},
                                            { 0, 0, 0},
                                            { 0, 0, 0},
                                            { 0, 0, 0},
                                            { 0, 0, 0},
                                            { 0, 0, 0},
                                            { 0, 0, 0}};
    //dot product up to the final motor
    float[,] m_cos = new float[9, 3] { { 0, 0, 0 },         
                                        { 0, 0, 0 },
                                        { 0, 0, 0 },
                                        { 0, 0, 0 },
                                        { 0, 0, 0 },
                                        { 0, 0, 0 },
                                        { 0, 0, 0 },
                                        { 0, 0, 0 },
                                        { 0, 0, 0 }};

    //the force of a motor in the target direction
    float[] answer_up1 = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };             
    float[] answer_up2 = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    float[] answer_down1 = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    float[] answer_down2 = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    float[] answer_left1 = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    float[] answer_left2 = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    float[] answer_right1 = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    float[] answer_right2 = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    float[] answer_forward1 = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    float[] answer_forward2 = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    float[] answer_backward1 = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    float[] answer_backward2 = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };


    //x, y, z Unit Vectors
    float[] unit_Vector_x = new float[3] { 1, 0, 0 };       
    float[] unit_Vector_y = new float[3] { 0, 1, 0 };
    float[] unit_Vector_z = new float[3] { 0, 0, 1 };

    //actual force entering a motor;
    public int[] MS_moter = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    private float last_position_x = 0.0f;
    private float last_position_y = 0.0f;
    private float last_position_z = 0.0f;
    // ************************************************  Kinesthetic feedback (variable) ************************************************






    // ************************************************  Kinesthetic feedback (function) ************************************************
    // calculate vector which is TargetObject to moters
    public void calcuate_MO_to_moter(float x, float y, float z)         
    {
        for (int i = 1; i < 9; i++)
        {
            MO_to_moter[i, 0] = moter[i, 0] - x;
            MO_to_moter[i, 1] = moter[i, 1] - y;
            MO_to_moter[i, 2] = moter[i, 2] - z;
        }

    }

    //calculate dot product up to the final motor
    public void get_xyz_cos()                  
    {
        for (int i = 1; i < 9; i++)
        {
            m_cos[i, 0] = calculateVector(new float[3] { MO_to_moter[i, 0], MO_to_moter[i, 1], MO_to_moter[i, 2] }, unit_Vector_x);
            m_cos[i, 1] = calculateVector(new float[3] { MO_to_moter[i, 0], MO_to_moter[i, 1], MO_to_moter[i, 2] }, unit_Vector_y);
            m_cos[i, 2] = calculateVector(new float[3] { MO_to_moter[i, 0], MO_to_moter[i, 1], MO_to_moter[i, 2] }, unit_Vector_z);
        }
    }

    // calculate dot product
    public float calculateVector(float[] v1, float[] v2)   
    {
        float result = 0;

        float v1_magnitude = 0;
        float v2_magnitude = 0;

        for (int i = 0; i < v1.Length; i++)
        {
            result += v1[i] * v2[i];    // a*b

            v1_magnitude += v1[i] * v1[i];  // |a|*|b|
            v2_magnitude += v2[i] * v2[i];
        }

        v1_magnitude = Mathf.Sqrt(v1_magnitude);
        v2_magnitude = Mathf.Sqrt(v2_magnitude);

        result /= (v1_magnitude * v2_magnitude);

        return result;
    }

    //using inverse matrix, calcuate the force of a motor in the target direction
    public void calculateMoter_up(int a, int b, int c, int flag)   
    {
        float[,] matrix = new float[3, 3] {
            {m_cos[a, 0], m_cos[b, 0], m_cos[c, 0]}
            ,{m_cos[a, 1], m_cos[b, 1], m_cos[c, 1]}
            ,{m_cos[a, 2], m_cos[b, 2], m_cos[c, 2]} };

        float[,] inverseMatrix = new float[3, 3];

        // calculate_InverseMatrix();
        float K = matrix[0, 0] * matrix[1, 1] * matrix[2, 2]
            - matrix[0, 0] * matrix[1, 2] * matrix[2, 1]
            - matrix[0, 1] * matrix[1, 0] * matrix[2, 2]
            + matrix[0, 1] * matrix[1, 2] * matrix[2, 0]
            + matrix[0, 2] * matrix[1, 0] * matrix[2, 1]
            - matrix[0, 2] * matrix[1, 1] * matrix[2, 0];

        inverseMatrix[0, 0] = 1 / K * (matrix[1, 1] * matrix[2, 2] - matrix[1, 2] * matrix[2, 1]);
        inverseMatrix[0, 1] = 1 / K * (matrix[0, 2] * matrix[2, 1] - matrix[0, 1] * matrix[2, 2]);
        inverseMatrix[0, 2] = 1 / K * (matrix[0, 1] * matrix[1, 2] - matrix[0, 2] * matrix[1, 1]);

        inverseMatrix[1, 0] = 1 / K * (matrix[1, 2] * matrix[2, 0] - matrix[1, 0] * matrix[2, 2]);
        inverseMatrix[1, 1] = 1 / K * (matrix[0, 0] * matrix[2, 2] - matrix[0, 2] * matrix[2, 0]);
        inverseMatrix[1, 2] = 1 / K * (matrix[0, 2] * matrix[1, 0] - matrix[0, 0] * matrix[1, 2]);

        inverseMatrix[2, 0] = 1 / K * (matrix[1, 0] * matrix[2, 1] - matrix[1, 1] * matrix[2, 0]);
        inverseMatrix[2, 1] = 1 / K * (matrix[0, 1] * matrix[2, 0] - matrix[0, 0] * matrix[2, 1]);
        inverseMatrix[2, 2] = 1 / K * (matrix[0, 0] * matrix[1, 1] - matrix[0, 1] * matrix[1, 0]);

        float goal_tension_x = 0;
        float goal_tension_y = (goal_N_high) / 2;
        float goal_tension_z = 0;

        float[] save_result = { inverseMatrix[0, 0] * goal_tension_x + inverseMatrix[0, 1] * goal_tension_y + inverseMatrix[0, 2] * goal_tension_z,
                                inverseMatrix[1, 0] * goal_tension_x + inverseMatrix[1, 1] * goal_tension_y + inverseMatrix[1, 2] * goal_tension_z,
                                inverseMatrix[2, 0] * goal_tension_x + inverseMatrix[2, 1] * goal_tension_y + inverseMatrix[2, 2] * goal_tension_z
                              };


        if (flag == 0)
        {
            answer_up1[a] = Mathf.Sqrt(Mathf.Pow(save_result[0], 2)) / N_to_input;
            answer_up1[b] = Mathf.Sqrt(Mathf.Pow(save_result[1], 2)) / N_to_input;
            answer_up1[c] = Mathf.Sqrt(Mathf.Pow(save_result[2], 2)) / N_to_input;
        }
        else if (flag == 1)
        {
            answer_up2[a] = Mathf.Sqrt(Mathf.Pow(save_result[0], 2)) / N_to_input;
            answer_up2[b] = Mathf.Sqrt(Mathf.Pow(save_result[1], 2)) / N_to_input;
            answer_up2[c] = Mathf.Sqrt(Mathf.Pow(save_result[2], 2)) / N_to_input;
        }
    }
    public void calculateMoter_down(int a, int b, int c, int flag)
    {
        float[,] matrix = new float[3, 3] {
            {m_cos[a, 0], m_cos[b, 0], m_cos[c, 0]}
            ,{m_cos[a, 1], m_cos[b, 1], m_cos[c, 1]}
            ,{m_cos[a, 2], m_cos[b, 2], m_cos[c, 2]} };

        float[,] inverseMatrix = new float[3, 3];

        float K = matrix[0, 0] * matrix[1, 1] * matrix[2, 2]
            - matrix[0, 0] * matrix[1, 2] * matrix[2, 1]
            - matrix[0, 1] * matrix[1, 0] * matrix[2, 2]
            + matrix[0, 1] * matrix[1, 2] * matrix[2, 0]
            + matrix[0, 2] * matrix[1, 0] * matrix[2, 1]
            - matrix[0, 2] * matrix[1, 1] * matrix[2, 0];

        inverseMatrix[0, 0] = 1 / K * (matrix[1, 1] * matrix[2, 2] - matrix[1, 2] * matrix[2, 1]);
        inverseMatrix[0, 1] = 1 / K * (matrix[0, 2] * matrix[2, 1] - matrix[0, 1] * matrix[2, 2]);
        inverseMatrix[0, 2] = 1 / K * (matrix[0, 1] * matrix[1, 2] - matrix[0, 2] * matrix[1, 1]);

        inverseMatrix[1, 0] = 1 / K * (matrix[1, 2] * matrix[2, 0] - matrix[1, 0] * matrix[2, 2]);
        inverseMatrix[1, 1] = 1 / K * (matrix[0, 0] * matrix[2, 2] - matrix[0, 2] * matrix[2, 0]);
        inverseMatrix[1, 2] = 1 / K * (matrix[0, 2] * matrix[1, 0] - matrix[0, 0] * matrix[1, 2]);

        inverseMatrix[2, 0] = 1 / K * (matrix[1, 0] * matrix[2, 1] - matrix[1, 1] * matrix[2, 0]);
        inverseMatrix[2, 1] = 1 / K * (matrix[0, 1] * matrix[2, 0] - matrix[0, 0] * matrix[2, 1]);
        inverseMatrix[2, 2] = 1 / K * (matrix[0, 0] * matrix[1, 1] - matrix[0, 1] * matrix[1, 0]);

        float goal_tension_x = 0;
        float goal_tension_y = goal_N_low / 2;
        float goal_tension_z = 0;

        float[] save_result = { inverseMatrix[0, 0] * goal_tension_x + inverseMatrix[0, 1] * goal_tension_y + inverseMatrix[0, 2] * goal_tension_z,
                                inverseMatrix[1, 0] * goal_tension_x + inverseMatrix[1, 1] * goal_tension_y + inverseMatrix[1, 2] * goal_tension_z,
                                inverseMatrix[2, 0] * goal_tension_x + inverseMatrix[2, 1] * goal_tension_y + inverseMatrix[2, 2] * goal_tension_z
                              };

        if (flag == 0)
        {
            answer_down1[a] = Mathf.Sqrt(Mathf.Pow(save_result[0], 2)) / N_to_input;
            answer_down1[b] = Mathf.Sqrt(Mathf.Pow(save_result[1], 2)) / N_to_input;
            answer_down1[c] = Mathf.Sqrt(Mathf.Pow(save_result[2], 2)) / N_to_input;
        }
        else if (flag == 1)
        {
            answer_down2[a] = Mathf.Sqrt(Mathf.Pow(save_result[0], 2)) / N_to_input;
            answer_down2[b] = Mathf.Sqrt(Mathf.Pow(save_result[1], 2)) / N_to_input;
            answer_down2[c] = Mathf.Sqrt(Mathf.Pow(save_result[2], 2)) / N_to_input;
        }
    }
    public void calculateMoter_left(int a, int b, int c, int flag)
    {
        float[,] matrix = new float[3, 3] {
            {m_cos[a, 0], m_cos[b, 0], m_cos[c, 0]}
            ,{m_cos[a, 1], m_cos[b, 1], m_cos[c, 1]}
            ,{m_cos[a, 2], m_cos[b, 2], m_cos[c, 2]} };

        float[,] inverseMatrix = new float[3, 3];

        float K = matrix[0, 0] * matrix[1, 1] * matrix[2, 2]
            - matrix[0, 0] * matrix[1, 2] * matrix[2, 1]
            - matrix[0, 1] * matrix[1, 0] * matrix[2, 2]
            + matrix[0, 1] * matrix[1, 2] * matrix[2, 0]
            + matrix[0, 2] * matrix[1, 0] * matrix[2, 1]
            - matrix[0, 2] * matrix[1, 1] * matrix[2, 0];

        inverseMatrix[0, 0] = 1 / K * (matrix[1, 1] * matrix[2, 2] - matrix[1, 2] * matrix[2, 1]);
        inverseMatrix[0, 1] = 1 / K * (matrix[0, 2] * matrix[2, 1] - matrix[0, 1] * matrix[2, 2]);
        inverseMatrix[0, 2] = 1 / K * (matrix[0, 1] * matrix[1, 2] - matrix[0, 2] * matrix[1, 1]);

        inverseMatrix[1, 0] = 1 / K * (matrix[1, 2] * matrix[2, 0] - matrix[1, 0] * matrix[2, 2]);
        inverseMatrix[1, 1] = 1 / K * (matrix[0, 0] * matrix[2, 2] - matrix[0, 2] * matrix[2, 0]);
        inverseMatrix[1, 2] = 1 / K * (matrix[0, 2] * matrix[1, 0] - matrix[0, 0] * matrix[1, 2]);

        inverseMatrix[2, 0] = 1 / K * (matrix[1, 0] * matrix[2, 1] - matrix[1, 1] * matrix[2, 0]);
        inverseMatrix[2, 1] = 1 / K * (matrix[0, 1] * matrix[2, 0] - matrix[0, 0] * matrix[2, 1]);
        inverseMatrix[2, 2] = 1 / K * (matrix[0, 0] * matrix[1, 1] - matrix[0, 1] * matrix[1, 0]);

        float goal_tension_x = goal_N_left / 2;
        float goal_tension_y = 0;
        float goal_tension_z = 0;

        float[] save_result = { inverseMatrix[0, 0] * goal_tension_x + inverseMatrix[0, 1] * goal_tension_y + inverseMatrix[0, 2] * goal_tension_z,
                                inverseMatrix[1, 0] * goal_tension_x + inverseMatrix[1, 1] * goal_tension_y + inverseMatrix[1, 2] * goal_tension_z,
                                inverseMatrix[2, 0] * goal_tension_x + inverseMatrix[2, 1] * goal_tension_y + inverseMatrix[2, 2] * goal_tension_z
                              };

        if (flag == 0)
        {
            answer_left1[a] = Mathf.Sqrt(Mathf.Pow(save_result[0], 2)) / N_to_input;
            answer_left1[b] = Mathf.Sqrt(Mathf.Pow(save_result[1], 2)) / N_to_input;
            answer_left1[c] = Mathf.Sqrt(Mathf.Pow(save_result[2], 2)) / N_to_input;
        }
        else if (flag == 1)
        {
            answer_left2[a] = Mathf.Sqrt(Mathf.Pow(save_result[0], 2)) / N_to_input;
            answer_left2[b] = Mathf.Sqrt(Mathf.Pow(save_result[1], 2)) / N_to_input;
            answer_left2[c] = Mathf.Sqrt(Mathf.Pow(save_result[2], 2)) / N_to_input;
        }
    }
    public void calculateMoter_right(int a, int b, int c, int flag)
    {
        float[,] matrix = new float[3, 3] {
            {m_cos[a, 0], m_cos[b, 0], m_cos[c, 0]}
            ,{m_cos[a, 1], m_cos[b, 1], m_cos[c, 1]}
            ,{m_cos[a, 2], m_cos[b, 2], m_cos[c, 2]} };

        float[,] inverseMatrix = new float[3, 3];

        float K = matrix[0, 0] * matrix[1, 1] * matrix[2, 2]
            - matrix[0, 0] * matrix[1, 2] * matrix[2, 1]
            - matrix[0, 1] * matrix[1, 0] * matrix[2, 2]
            + matrix[0, 1] * matrix[1, 2] * matrix[2, 0]
            + matrix[0, 2] * matrix[1, 0] * matrix[2, 1]
            - matrix[0, 2] * matrix[1, 1] * matrix[2, 0];

        inverseMatrix[0, 0] = 1 / K * (matrix[1, 1] * matrix[2, 2] - matrix[1, 2] * matrix[2, 1]);
        inverseMatrix[0, 1] = 1 / K * (matrix[0, 2] * matrix[2, 1] - matrix[0, 1] * matrix[2, 2]);
        inverseMatrix[0, 2] = 1 / K * (matrix[0, 1] * matrix[1, 2] - matrix[0, 2] * matrix[1, 1]);

        inverseMatrix[1, 0] = 1 / K * (matrix[1, 2] * matrix[2, 0] - matrix[1, 0] * matrix[2, 2]);
        inverseMatrix[1, 1] = 1 / K * (matrix[0, 0] * matrix[2, 2] - matrix[0, 2] * matrix[2, 0]);
        inverseMatrix[1, 2] = 1 / K * (matrix[0, 2] * matrix[1, 0] - matrix[0, 0] * matrix[1, 2]);

        inverseMatrix[2, 0] = 1 / K * (matrix[1, 0] * matrix[2, 1] - matrix[1, 1] * matrix[2, 0]);
        inverseMatrix[2, 1] = 1 / K * (matrix[0, 1] * matrix[2, 0] - matrix[0, 0] * matrix[2, 1]);
        inverseMatrix[2, 2] = 1 / K * (matrix[0, 0] * matrix[1, 1] - matrix[0, 1] * matrix[1, 0]);

        float goal_tension_x = goal_N_right / 2;
        float goal_tension_y = 0;
        float goal_tension_z = 0;

        float[] save_result = { inverseMatrix[0, 0] * goal_tension_x + inverseMatrix[0, 1] * goal_tension_y + inverseMatrix[0, 2] * goal_tension_z,
                                inverseMatrix[1, 0] * goal_tension_x + inverseMatrix[1, 1] * goal_tension_y + inverseMatrix[1, 2] * goal_tension_z,
                                inverseMatrix[2, 0] * goal_tension_x + inverseMatrix[2, 1] * goal_tension_y + inverseMatrix[2, 2] * goal_tension_z
                              };


        if (flag == 0)
        {
            answer_right1[a] = Mathf.Sqrt(Mathf.Pow(save_result[0], 2)) / N_to_input;
            answer_right1[b] = Mathf.Sqrt(Mathf.Pow(save_result[1], 2)) / N_to_input;
            answer_right1[c] = Mathf.Sqrt(Mathf.Pow(save_result[2], 2)) / N_to_input;
        }
        else if (flag == 1)
        {
            answer_right2[a] = Mathf.Sqrt(Mathf.Pow(save_result[0], 2)) / N_to_input;
            answer_right2[b] = Mathf.Sqrt(Mathf.Pow(save_result[1], 2)) / N_to_input;
            answer_right2[c] = Mathf.Sqrt(Mathf.Pow(save_result[2], 2)) / N_to_input;
        }
    }
    public void calculateMoter_forward(int a, int b, int c, int flag)
    {
        float[,] matrix = new float[3, 3] {
            {m_cos[a, 0], m_cos[b, 0], m_cos[c, 0]}
            ,{m_cos[a, 1], m_cos[b, 1], m_cos[c, 1]}
            ,{m_cos[a, 2], m_cos[b, 2], m_cos[c, 2]} };

        float[,] inverseMatrix = new float[3, 3];

        float K = matrix[0, 0] * matrix[1, 1] * matrix[2, 2]
            - matrix[0, 0] * matrix[1, 2] * matrix[2, 1]
            - matrix[0, 1] * matrix[1, 0] * matrix[2, 2]
            + matrix[0, 1] * matrix[1, 2] * matrix[2, 0]
            + matrix[0, 2] * matrix[1, 0] * matrix[2, 1]
            - matrix[0, 2] * matrix[1, 1] * matrix[2, 0];

        inverseMatrix[0, 0] = 1 / K * (matrix[1, 1] * matrix[2, 2] - matrix[1, 2] * matrix[2, 1]);
        inverseMatrix[0, 1] = 1 / K * (matrix[0, 2] * matrix[2, 1] - matrix[0, 1] * matrix[2, 2]);
        inverseMatrix[0, 2] = 1 / K * (matrix[0, 1] * matrix[1, 2] - matrix[0, 2] * matrix[1, 1]);

        inverseMatrix[1, 0] = 1 / K * (matrix[1, 2] * matrix[2, 0] - matrix[1, 0] * matrix[2, 2]);
        inverseMatrix[1, 1] = 1 / K * (matrix[0, 0] * matrix[2, 2] - matrix[0, 2] * matrix[2, 0]);
        inverseMatrix[1, 2] = 1 / K * (matrix[0, 2] * matrix[1, 0] - matrix[0, 0] * matrix[1, 2]);

        inverseMatrix[2, 0] = 1 / K * (matrix[1, 0] * matrix[2, 1] - matrix[1, 1] * matrix[2, 0]);
        inverseMatrix[2, 1] = 1 / K * (matrix[0, 1] * matrix[2, 0] - matrix[0, 0] * matrix[2, 1]);
        inverseMatrix[2, 2] = 1 / K * (matrix[0, 0] * matrix[1, 1] - matrix[0, 1] * matrix[1, 0]);

        float goal_tension_x = 0;
        float goal_tension_y = 0;
        float goal_tension_z = goal_N_forward / 2;

        float[] save_result = { inverseMatrix[0, 0] * goal_tension_x + inverseMatrix[0, 1] * goal_tension_y + inverseMatrix[0, 2] * goal_tension_z,
                                inverseMatrix[1, 0] * goal_tension_x + inverseMatrix[1, 1] * goal_tension_y + inverseMatrix[1, 2] * goal_tension_z,
                                inverseMatrix[2, 0] * goal_tension_x + inverseMatrix[2, 1] * goal_tension_y + inverseMatrix[2, 2] * goal_tension_z
                              };

        if (flag == 0)
        {
            answer_forward1[a] = Mathf.Sqrt(Mathf.Pow(save_result[0], 2)) / N_to_input;
            answer_forward1[b] = Mathf.Sqrt(Mathf.Pow(save_result[1], 2)) / N_to_input;
            answer_forward1[c] = Mathf.Sqrt(Mathf.Pow(save_result[2], 2)) / N_to_input;
        }
        else if (flag == 1)
        {
            answer_forward2[a] = Mathf.Sqrt(Mathf.Pow(save_result[0], 2)) / N_to_input;
            answer_forward2[b] = Mathf.Sqrt(Mathf.Pow(save_result[1], 2)) / N_to_input;
            answer_forward2[c] = Mathf.Sqrt(Mathf.Pow(save_result[2], 2)) / N_to_input;
        }
    }
    public void calculateMoter_backward(int a, int b, int c, int flag)
    {
        float[,] matrix = new float[3, 3] {
            {m_cos[a, 0], m_cos[b, 0], m_cos[c, 0]}
            ,{m_cos[a, 1], m_cos[b, 1], m_cos[c, 1]}
            ,{m_cos[a, 2], m_cos[b, 2], m_cos[c, 2]} };

        float[,] inverseMatrix = new float[3, 3];

        float K = matrix[0, 0] * matrix[1, 1] * matrix[2, 2]
            - matrix[0, 0] * matrix[1, 2] * matrix[2, 1]
            - matrix[0, 1] * matrix[1, 0] * matrix[2, 2]
            + matrix[0, 1] * matrix[1, 2] * matrix[2, 0]
            + matrix[0, 2] * matrix[1, 0] * matrix[2, 1]
            - matrix[0, 2] * matrix[1, 1] * matrix[2, 0];

        inverseMatrix[0, 0] = 1 / K * (matrix[1, 1] * matrix[2, 2] - matrix[1, 2] * matrix[2, 1]);
        inverseMatrix[0, 1] = 1 / K * (matrix[0, 2] * matrix[2, 1] - matrix[0, 1] * matrix[2, 2]);
        inverseMatrix[0, 2] = 1 / K * (matrix[0, 1] * matrix[1, 2] - matrix[0, 2] * matrix[1, 1]);

        inverseMatrix[1, 0] = 1 / K * (matrix[1, 2] * matrix[2, 0] - matrix[1, 0] * matrix[2, 2]);
        inverseMatrix[1, 1] = 1 / K * (matrix[0, 0] * matrix[2, 2] - matrix[0, 2] * matrix[2, 0]);
        inverseMatrix[1, 2] = 1 / K * (matrix[0, 2] * matrix[1, 0] - matrix[0, 0] * matrix[1, 2]);

        inverseMatrix[2, 0] = 1 / K * (matrix[1, 0] * matrix[2, 1] - matrix[1, 1] * matrix[2, 0]);
        inverseMatrix[2, 1] = 1 / K * (matrix[0, 1] * matrix[2, 0] - matrix[0, 0] * matrix[2, 1]);
        inverseMatrix[2, 2] = 1 / K * (matrix[0, 0] * matrix[1, 1] - matrix[0, 1] * matrix[1, 0]);

        float goal_tension_x = 0;
        float goal_tension_y = 0;
        float goal_tension_z = goal_N_backward / 2;

        float[] save_result = { inverseMatrix[0, 0] * goal_tension_x + inverseMatrix[0, 1] * goal_tension_y + inverseMatrix[0, 2] * goal_tension_z,
                                inverseMatrix[1, 0] * goal_tension_x + inverseMatrix[1, 1] * goal_tension_y + inverseMatrix[1, 2] * goal_tension_z,
                                inverseMatrix[2, 0] * goal_tension_x + inverseMatrix[2, 1] * goal_tension_y + inverseMatrix[2, 2] * goal_tension_z
                              };

        if (flag == 0)
        {
            answer_backward1[a] = Mathf.Sqrt(Mathf.Pow(save_result[0], 2)) / N_to_input;
            answer_backward1[b] = Mathf.Sqrt(Mathf.Pow(save_result[1], 2)) / N_to_input;
            answer_backward1[c] = Mathf.Sqrt(Mathf.Pow(save_result[2], 2)) / N_to_input;
        }
        else if (flag == 1)
        {
            answer_backward2[a] = Mathf.Sqrt(Mathf.Pow(save_result[0], 2)) / N_to_input;
            answer_backward2[b] = Mathf.Sqrt(Mathf.Pow(save_result[1], 2)) / N_to_input;
            answer_backward2[c] = Mathf.Sqrt(Mathf.Pow(save_result[2], 2)) / N_to_input;
        }
    }

    // Transfer the calculated value to the motor
    public void operating_8_motor(int n1, int n2, int n3, int n4, int n5, int n6, int n7, int n8) 
    {
        SyncWrite_Tension(n1 - 1, n2 - 1, n3 - 1, n4 - 1);
        SyncWrite_Tension2(n5 - 1, n6 - 1, n7 - 1, n8 - 1);
    }
    // answer value initializing
    public void initializing_answer()
    {
        for (int i = 0; i < 9; i++)
        {
            answer_up1[i] = 0;
            answer_up2[i] = 0;
            answer_down1[i] = 0;
            answer_down2[i] = 0;
            answer_left1[i] = 0;
            answer_left2[i] = 0;
            answer_right1[i] = 0;
            answer_right2[i] = 0;
            answer_forward1[i] = 0;
            answer_forward2[i] = 0;
            answer_backward1[i] = 0;
            answer_backward2[i] = 0;
        }
    }

    //Combine the calculated values and enter the force applied to the motor
    public void Moving_Test()      
    {

        if (time_flag == 1 && input_force_flag == true)
        {
            for (int i = 1; i < 9; i++)
            {
                MS_moter[i] = Math.Min((int)(
                    +answer_up1[i] + answer_up2[i]
                    + answer_down1[i] + answer_down2[i]
                    + answer_left1[i] + answer_left2[i]
                    + answer_right1[i] + answer_right2[i]
                    + answer_forward1[i] + answer_forward2[i]
                    + answer_backward1[i] + answer_backward2[i]
                    ), maxTension);
                MS_moter[i] = Math.Max(MS_moter[i], minTension);
            }

            operating_8_motor(MS_moter[1], MS_moter[2], MS_moter[3], MS_moter[4], MS_moter[5], MS_moter[6], MS_moter[7], MS_moter[8]);

        }
        else
        {
            operating_8_motor(defaultTension, defaultTension, defaultTension, defaultTension, defaultTension, defaultTension, defaultTension, defaultTension);
            for (int q = 0; q < 9; q++)
            {
                MS_moter[q] = defaultTension;
            }
        }
    }

    //Functions used in the Invoke() function
    public void Time_control_limit()       
    {
        goal_N_high = 0;
        goal_N_low = 0;
        goal_N_right = 0f;
        goal_N_left = 0f;
        goal_N_forward = 0f;
        goal_N_backward = 0f;

        input_force_flag = false;
    }

    //Calculate the required force by combining the motors in the defined area for each direction
    public void make_tenstion_up()     
    {
        if (leftFinger_position_x <= 0 && (Mathf.Abs(leftFinger_position_x) >= Mathf.Abs(leftFinger_position_z)))
        {
            calculateMoter_up(1, 2, 5, 0);
            calculateMoter_up(1, 5, 7, 1);
        }
        else if (leftFinger_position_z <= 0 && (Mathf.Abs(leftFinger_position_x) <= Mathf.Abs(leftFinger_position_z)))
        {
            calculateMoter_up(1, 2, 5, 0);
            calculateMoter_up(2, 5, 7, 1);
        }
        else if (leftFinger_position_x >= 0 && (Mathf.Abs(leftFinger_position_x) >= Mathf.Abs(leftFinger_position_z)))
        {
            calculateMoter_up(1, 2, 7, 0);
            calculateMoter_up(2, 5, 7, 1);
        }
        else if (leftFinger_position_z >= 0 && (Mathf.Abs(leftFinger_position_x) <= Mathf.Abs(leftFinger_position_z)))
        {
            calculateMoter_up(1, 2, 7, 0);
            calculateMoter_up(1, 5, 7, 1);
        }
    }
    public void make_tenstion_down()
    {
        if (leftFinger_position_x <= 0 && (Mathf.Abs(leftFinger_position_x) >= Mathf.Abs(leftFinger_position_z)))
        {
            calculateMoter_down(3, 6, 8, 0);
            calculateMoter_down(3, 4, 8, 1);
        }
        else if (leftFinger_position_z >= 0 && (Mathf.Abs(leftFinger_position_x) <= Mathf.Abs(leftFinger_position_z)))
        {
            calculateMoter_down(4, 6, 8, 0);
            calculateMoter_down(3, 4, 8, 1);
        }
        else if (leftFinger_position_x >= 0 && (Mathf.Abs(leftFinger_position_x) >= Mathf.Abs(leftFinger_position_z)))
        {
            calculateMoter_down(4, 6, 8, 0);
            calculateMoter_down(3, 4, 6, 1);
        }
        else if (leftFinger_position_z <= 0 && (Mathf.Abs(leftFinger_position_x) <= Mathf.Abs(leftFinger_position_z)))
        {
            calculateMoter_down(3, 6, 8, 0);
            calculateMoter_down(3, 4, 6, 1);
        }

    }
    public void make_tenstion_left()
    {
        if (leftFinger_position_y >= 0 && (Mathf.Abs(leftFinger_position_y) >= Mathf.Abs(leftFinger_position_z)))
        {
            calculateMoter_left(1, 3, 5, 0);
            calculateMoter_left(1, 5, 8, 1);
        }
        else if (leftFinger_position_z >= 0 && (Mathf.Abs(leftFinger_position_y) <= Mathf.Abs(leftFinger_position_z)))
        {
            calculateMoter_left(1, 5, 8, 0);
            calculateMoter_left(1, 3, 8, 1);
        }
        if (leftFinger_position_y <= 0 && (Mathf.Abs(leftFinger_position_y) >= Mathf.Abs(leftFinger_position_z)))
        {
            calculateMoter_left(1, 3, 8, 0);
            calculateMoter_left(3, 5, 8, 1);
        }
        else if (leftFinger_position_z <= 0 && (Mathf.Abs(leftFinger_position_y) <= Mathf.Abs(leftFinger_position_z)))
        {
            calculateMoter_left(3, 5, 8, 0);
            calculateMoter_left(1, 3, 5, 1);

        }
    }
    public void make_tenstion_right()
    {
        if (leftFinger_position_y >= 0 && (Mathf.Abs(leftFinger_position_y) >= Mathf.Abs(leftFinger_position_z)))
        {
            calculateMoter_right(2, 6, 7, 0);
            calculateMoter_right(2, 4, 7, 1);
        }
        else if (leftFinger_position_z >= 0 && (Mathf.Abs(leftFinger_position_y) <= Mathf.Abs(leftFinger_position_z)))
        {
            calculateMoter_right(2, 4, 7, 0);
            calculateMoter_right(4, 6, 7, 1);
        }
        if (leftFinger_position_y <= 0 && (Mathf.Abs(leftFinger_position_y) >= Mathf.Abs(leftFinger_position_z)))
        {
            calculateMoter_right(4, 6, 7, 0);
            calculateMoter_right(2, 4, 6, 1);
        }
        else if (leftFinger_position_z <= 0 && (Mathf.Abs(leftFinger_position_y) <= Mathf.Abs(leftFinger_position_z)))
        {
            calculateMoter_right(2, 4, 6, 0);
            calculateMoter_right(2, 6, 7, 1);
        }
    }
    public void make_tenstion_forward()
    {
        if (leftFinger_position_y >= 0 && (Mathf.Abs(leftFinger_position_y) >= Mathf.Abs(leftFinger_position_x)))
        {
            calculateMoter_forward(1, 7, 8, 0);
            calculateMoter_forward(1, 4, 7, 1);
        }
        else if (leftFinger_position_x >= 0 && (Mathf.Abs(leftFinger_position_y) <= Mathf.Abs(leftFinger_position_x)))
        {
            calculateMoter_forward(1, 4, 7, 0);
            calculateMoter_forward(4, 7, 8, 1);
        }
        if (leftFinger_position_y <= 0 && (Mathf.Abs(leftFinger_position_y) >= Mathf.Abs(leftFinger_position_x)))
        {
            calculateMoter_forward(4, 7, 8, 0);
            calculateMoter_forward(1, 4, 8, 1);
        }
        else if (leftFinger_position_x <= 0 && (Mathf.Abs(leftFinger_position_y) <= Mathf.Abs(leftFinger_position_x)))
        {
            calculateMoter_forward(1, 4, 8, 0);
            calculateMoter_forward(1, 7, 8, 1);
        }
    }
    public void make_tenstion_backward()
    {
        if (leftFinger_position_y >= 0 && (Mathf.Abs(leftFinger_position_y) >= Mathf.Abs(leftFinger_position_x)))
        {
            calculateMoter_backward(2, 3, 5, 0);
            calculateMoter_backward(2, 5, 6, 1);
        }
        else if (leftFinger_position_x >= 0 && (Mathf.Abs(leftFinger_position_y) <= Mathf.Abs(leftFinger_position_x)))
        {
            calculateMoter_backward(2, 5, 6, 0);
            calculateMoter_backward(2, 3, 6, 1);
        }
        if (leftFinger_position_y <= 0 && (Mathf.Abs(leftFinger_position_y) >= Mathf.Abs(leftFinger_position_x)))
        {
            calculateMoter_backward(2, 3, 6, 0);
            calculateMoter_backward(3, 5, 6, 1);
        }
        else if (leftFinger_position_x <= 0 && (Mathf.Abs(leftFinger_position_y) <= Mathf.Abs(leftFinger_position_x)))
        {
            calculateMoter_backward(3, 5, 6, 0);
            calculateMoter_backward(2, 3, 5, 1);
        }
    }
    // ************************************************  Kinesthetic feedback (function) ************************************************

    void Start()
    {
        timer = new Timer();
        timer.Interval = 1;
        timer.Elapsed += OnTimerEvent;
        timer.AutoReset = true;
        timer.Enabled = false;
    }






    //*************************************************  Motor Seeting functions  ***************************************
    static void Openport(int portNumber)
    {
        if (dynamixel.openPort(portNumber))
        {
            Debug.Log("Succeeded to open the port!" + portNumber);
        }
        else
        {
            Debug.Log("Failed to open the port!");
            Debug.Log("Press any key to terminate...");


        }
    }
    static void Setportbaudrate(int portNumber)
    {
        if (dynamixel.setBaudRate(portNumber, BAUDRATE))
        {
            Debug.Log("Succeeded to change the baudrate!");
        }
        else
        {
            Debug.Log("Failed to change the baudrate!");
            Debug.Log("Press any key to terminate...");


        }
    }

    static void EnableDynamixelTorque(int portNumber, int DXL_ID)
    {
        int dxl_comm_result = COMM_TX_FAIL;                                   // Communication result
        byte dxl_error = 0;                                                   // Dynamixel error

        dynamixel.write1ByteTxRx(portNumber, PROTOCOL_VERSION, (byte)DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
        if ((dxl_comm_result = dynamixel.getLastTxRxResult(portNumber, PROTOCOL_VERSION)) != COMM_SUCCESS)
        {
            Debug.Log(Marshal.PtrToStringAnsi(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result)));
        }
        else if ((dxl_error = dynamixel.getLastRxPacketError(portNumber, PROTOCOL_VERSION)) != 0)
        {
            Debug.Log(Marshal.PtrToStringAnsi(dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error)));
        }
        else
        {
            if (_isRight) Debug.Log("R_Finger : Dynamixel" + (byte)DXL_ID + " has been successfully connected ");
            else Debug.Log("L_Finger : Dynamixel" + (byte)DXL_ID + " has been successfully connected ");
        }
    }
    static void DisableDynamixelTorque(int portNumber, int DXL_ID)
    {
        int dxl_comm_result = COMM_TX_FAIL;                                   // Communication result
        byte dxl_error = 0;                                                   // Dynamixel error

        dynamixel.write1ByteTxRx(portNumber, PROTOCOL_VERSION, (byte)DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
        if ((dxl_comm_result = dynamixel.getLastTxRxResult(portNumber, PROTOCOL_VERSION)) != COMM_SUCCESS)
        {
            Debug.Log(Marshal.PtrToStringAnsi(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result)));
        }
        else if ((dxl_error = dynamixel.getLastRxPacketError(portNumber, PROTOCOL_VERSION)) != 0)
        {
            Debug.Log(Marshal.PtrToStringAnsi(dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error)));
        }
    }
    static void SyncRead(int port_num, int groupread_num)
    {
        int dxl_comm_result = COMM_TX_FAIL;                                     // Communication result
        bool dxl_getdata_result = true;                                        // GetParam result

        // Syncread present position

 
            dynamixel.groupSyncReadTxRxPacket(groupread_num);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
                Debug.Log(Marshal.PtrToStringAnsi(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result)));



        #region  Check if groupsyncread data of Dynamixel is available
        // Check if groupsyncread data of Dynamixel#1 is available

        dxl_getdata_result = dynamixel.groupSyncReadIsAvailable(groupread_num, DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        if (dxl_getdata_result != true)
            {
                Debug.Log("[ID: {" + DXL1_ID + "}] groupSyncRead getdata failed");
                return;
            }

        // Check if groupsyncread data of Dynamixel#2 is available
        dxl_getdata_result = dynamixel.groupSyncReadIsAvailable(groupread_num, DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        if (dxl_getdata_result != true)
            {
                Debug.Log("[ID: {" + DXL2_ID + "}] groupSyncRead getdata failed");
                return;
            }

        dxl_getdata_result = dynamixel.groupSyncReadIsAvailable(groupread_num, DXL3_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        // Check if groupsyncread data of Dynamixel#3 is available
        if (dxl_getdata_result != true)
            {
                Debug.Log("[ID: {" + DXL3_ID + "}] groupSyncRead getdata failed");
                return;
            }

        dxl_getdata_result = dynamixel.groupSyncReadIsAvailable(groupread_num, DXL4_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        // Check if groupsyncread data of Dynamixel#4 is available
        if (dxl_getdata_result != true)
            {
                Debug.Log("[ID: {" + DXL4_ID + "}] groupSyncRead getdata failed");
                return;
            }
    
            #endregion
        
    }
    public static void SyncWrite_Tension(int m1, int m2, int m3, int m4)
    {
            #region Add Dynamixel goal position value to the Syncwrite storage
            // Add Dynamixel#1 goal position value to the Syncwrite storage
            dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_LeftControl, DXL1_ID, (UInt32)m1, LEN_PRO_GOAL_POSITION);
            if (dxl_addparam_result != true)
            {
                Debug.Log("[ID: {" + DXL1_ID + "}] groupSyncWrite addparam failed");
                return;
            }

            // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
            dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_LeftControl, DXL2_ID, (UInt32)m2, LEN_PRO_GOAL_POSITION);
            if (dxl_addparam_result != true)
            {
                Debug.Log("[ID: {" + DXL2_ID + "}] groupSyncWrite addparam failed");
                return;
            }

            // Add Dynamixel#3 goal position value to the Syncwrite parameter storage
            dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_LeftControl, DXL3_ID, (UInt32)m3, LEN_PRO_GOAL_POSITION);
            if (dxl_addparam_result != true)
            {
                Debug.Log("[ID: {" + DXL3_ID + "}] groupSyncWrite addparam failed");
                return;
            }

            // Add Dynamixel#4 goal position value to the Syncwrite parameter storage
            dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_LeftControl, DXL4_ID, (UInt32)m4, LEN_PRO_GOAL_POSITION);
            if (dxl_addparam_result != true)
            {
                Debug.Log("[ID: {" + DXL4_ID + "}] groupSyncWrite addparam failed");
                return;
            }
            #endregion

            // Syncwrite goal position
            dynamixel.groupSyncWriteTxPacket(write_LeftControl);
            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_leftControl, PROTOCOL_VERSION)) != COMM_SUCCESS)
                Debug.Log(Marshal.PtrToStringAnsi(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result)));

            // Clear syncwrite parameter storage
            dynamixel.groupSyncWriteClearParam(write_LeftControl);
      
    }
    public static void SyncWrite_Tension2(int m1, int m2, int m3, int m4)
    {
            #region Add Dynamixel goal position value to the Syncwrite storage
            // Add Dynamixel#1 goal position value to the Syncwrite storage
            dxl_addparam_result2 = dynamixel.groupSyncWriteAddParam(write_RightControl, DXL5_ID, (UInt32)m1, LEN_PRO_GOAL_POSITION);
            if (dxl_addparam_result2 != true)
            {
                Debug.Log("[ID: {" + DXL5_ID + "}] groupSyncWrite addparam failed");
                return;
            }

            // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
            dxl_addparam_result2 = dynamixel.groupSyncWriteAddParam(write_RightControl, DXL6_ID, (UInt32)m2, LEN_PRO_GOAL_POSITION);
            if (dxl_addparam_result2 != true)
            {
                Debug.Log("[ID: {" + DXL6_ID + "}] groupSyncWrite addparam failed");
                return;
            }

            // Add Dynamixel#3 goal position value to the Syncwrite parameter storage
            dxl_addparam_result2 = dynamixel.groupSyncWriteAddParam(write_RightControl, DXL7_ID, (UInt32)m3, LEN_PRO_GOAL_POSITION);
            if (dxl_addparam_result2 != true)
            {
                Debug.Log("[ID: {" + DXL7_ID + "}] groupSyncWrite addparam failed");
                return;
            }

            // Add Dynamixel#4 goal position value to the Syncwrite parameter storage
            dxl_addparam_result2 = dynamixel.groupSyncWriteAddParam(write_RightControl, DXL8_ID, (UInt32)m4, LEN_PRO_GOAL_POSITION);
            if (dxl_addparam_result2 != true)
            {
                Debug.Log("[ID: {" + DXL8_ID + "}] groupSyncWrite addparam failed");
                return;
            }

            #endregion

            // Syncwrite goal position
            dynamixel.groupSyncWriteTxPacket(write_RightControl);
            if ((dxl_comm_result2 = dynamixel.getLastTxRxResult(port_rightControl, PROTOCOL_VERSION)) != COMM_SUCCESS)
            {
                Debug.Log(Marshal.PtrToStringAnsi(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result2)));
            }

            // Clear syncwrite parameter storage
            dynamixel.groupSyncWriteClearParam(write_RightControl);
    }
    void Initializing()
    {
        #region Initializing

        port_leftControl = dynamixel.portHandler(DEVICENAME_LEFTCONTROL);
        port_rightControl = dynamixel.portHandler(DEVICENAME_RIGHTCONTROL);


        dynamixel.packetHandler();

        write_LeftControl = dynamixel.groupSyncWrite(port_leftControl, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
        read_LeftControl = dynamixel.groupSyncRead(port_leftControl, PROTOCOL_VERSION, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);


        write_RightControl = dynamixel.groupSyncWrite(port_rightControl, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
        #endregion


        #region Initializing_Custom
        //// Open port
        Openport(port_leftControl);
        Openport(port_rightControl);
        // Set port baudrate
        Setportbaudrate(port_leftControl);
        Setportbaudrate(port_rightControl);

        // Enable Dynamixel#1 Torque
        EnableDynamixelTorque(port_leftControl, DXL1_ID);
        // Enable Dynamixel#2 Torque
        EnableDynamixelTorque(port_leftControl, DXL2_ID);
        // Enable Dynamixel#3 Torque
        EnableDynamixelTorque(port_leftControl, DXL3_ID);
        // Enable Dynamixel#4 Torque
        EnableDynamixelTorque(port_leftControl, DXL4_ID);

        // Enable Dynamixel#5 Torque
        EnableDynamixelTorque(port_rightControl, DXL5_ID);
        // Enable Dynamixel#6 Torque
        EnableDynamixelTorque(port_rightControl, DXL6_ID);
        // Enable Dynamixel#7 Torque
        EnableDynamixelTorque(port_rightControl, DXL7_ID);
        // Enable Dynamixel#8 Torque
        EnableDynamixelTorque(port_rightControl, DXL8_ID);
        #endregion


        #region Add parameter storage for Dynamixel present position value
        // Add parameter storage for Dynamixel#1 present position value
        dxl_addparam_result = dynamixel.groupSyncReadAddParam(read_LeftControl, DXL1_ID);
        if (dxl_addparam_result != true)
        {
            Debug.Log("[ID: " + DXL1_ID + "] groupSyncRead addparam failed");
            return;
        }

        // Add parameter storage for Dynamixel#2 present position value
        dxl_addparam_result = dynamixel.groupSyncReadAddParam(read_LeftControl, DXL2_ID);
        if (dxl_addparam_result != true)
        {
            Debug.Log("[ID: " + DXL2_ID + "] groupSyncRead addparam failed");
            return;
        }

        // Add parameter storage for Dynamixel#3 present position value
        dxl_addparam_result = dynamixel.groupSyncReadAddParam(read_LeftControl, DXL3_ID);
        if (dxl_addparam_result != true)
        {
            Debug.Log("[ID: " + DXL3_ID + "] groupSyncRead addparam failed");
            return;
        }

        // Add parameter storage for Dynamixel#4 present position value
        dxl_addparam_result = dynamixel.groupSyncReadAddParam(read_LeftControl, DXL4_ID);
        if (dxl_addparam_result != true)
        {
            Debug.Log("[ID: " + DXL4_ID + "] groupSyncRead addparam failed");
            return;
        }
        #endregion
    }
    //*************************************************  Motor Seeting functions  ***************************************







    //*************************************************  Code update section  ***************************************
    void OnTimerEvent(System.Object source, ElapsedEventArgs e)
    {
        // Initialing moter & encoder
        if (timer_start == 1)
        {
            timer_start = 2;
            _isRight = true;
            Initializing();

            #region SyncWrite Initial Torque
            #region Add Dynamixel goal position value to the Syncwrite storage
            // Add Dynamixel#1 goal position value to the Syncwrite storage
            dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_LeftControl, DXL1_ID, (UInt32)dxl_initial_torque, LEN_PRO_GOAL_POSITION);
            if (dxl_addparam_result != true)
            {
                Console.WriteLine("[ID: {0}] groupSyncWrite addparam failed", DXL1_ID);
                return;
            }

            // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
            dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_LeftControl, DXL2_ID, (UInt32)dxl_initial_torque, LEN_PRO_GOAL_POSITION);
            if (dxl_addparam_result != true)
            {
                Console.WriteLine("[ID: {0}] groupSyncWrite addparam failed", DXL2_ID);
                return;
            }

            // Add Dynamixel#3 goal position value to the Syncwrite parameter storage
            dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_LeftControl, DXL3_ID, (UInt32)dxl_initial_torque, LEN_PRO_GOAL_POSITION);
            if (dxl_addparam_result != true)
            {
                Console.WriteLine("[ID: {0}] groupSyncWrite addparam failed", DXL3_ID);
                return;
            }

            // Add Dynamixel#4 goal position value to the Syncwrite parameter storage
            dxl_addparam_result = dynamixel.groupSyncWriteAddParam(write_LeftControl, DXL4_ID, (UInt32)dxl_initial_torque, LEN_PRO_GOAL_POSITION);
            if (dxl_addparam_result != true)
            {
                Console.WriteLine("[ID: {0}] groupSyncWrite addparam failed", DXL4_ID);
                return;
            }

            #endregion

            // Syncwrite goal position
            dynamixel.groupSyncWriteTxPacket(write_LeftControl);
            dynamixel.groupSyncWriteTxPacket(write_RightControl);

            if ((dxl_comm_result = dynamixel.getLastTxRxResult(port_leftControl, PROTOCOL_VERSION)) != COMM_SUCCESS)
                Console.WriteLine(Marshal.PtrToStringAnsi(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result)));

            if ((dxl_comm_result2 = dynamixel.getLastTxRxResult(port_rightControl, PROTOCOL_VERSION)) != COMM_SUCCESS)
                Console.WriteLine(Marshal.PtrToStringAnsi(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result2)));

            // Clear syncwrite parameter storage
            dynamixel.groupSyncWriteClearParam(write_LeftControl);
            dynamixel.groupSyncWriteClearParam(write_RightControl);
            #endregion

            SyncRead(port_leftControl, read_LeftControl);

            #region Get Dynamixel present position value

            // Get Dynamixel#1 present position value
            dxl1_present_position = (Int32)dynamixel.groupSyncReadGetData(read_LeftControl, DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

            // Get Dynamixel#2 present position value
            dxl2_present_position = (Int32)dynamixel.groupSyncReadGetData(read_LeftControl, DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

            // Get Dynamixel#2 present position value
            dxl3_present_position = (Int32)dynamixel.groupSyncReadGetData(read_LeftControl, DXL3_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

            // Get Dynamixel#2 present position value
            dxl4_present_position = (Int32)dynamixel.groupSyncReadGetData(read_LeftControl, DXL4_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
            #endregion
            //Get Dynamixel#1 present position value

            init_length1 = dxl1_present_position;
            init_length2 = dxl2_present_position;
            init_length3 = dxl3_present_position;
            init_length4 = dxl4_present_position;

        }

        timer_count++;
        if(timer_count>=4)
        {
            // Calculate Moving_Object location & Moter tension
            timer_count = 0;
            if (timer_start == 3)
            {
                //********************************************  Calculate Moving_Object location  *****************************************

                SyncRead(port_leftControl, read_LeftControl);
                #region Get Dynamixel present position value

                // Get Dynamixel#1 present position value
                dxl1_present_position = (Int32)dynamixel.groupSyncReadGetData(read_LeftControl, DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

                // Get Dynamixel#2 present position value
                dxl2_present_position = (Int32)dynamixel.groupSyncReadGetData(read_LeftControl, DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

                // Get Dynamixel#2 present position value
                dxl3_present_position = (Int32)dynamixel.groupSyncReadGetData(read_LeftControl, DXL3_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

                // Get Dynamixel#2 present position value
                dxl4_present_position = (Int32)dynamixel.groupSyncReadGetData(read_LeftControl, DXL4_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);


                wire_angle1 = (float)(init_length1 - dxl1_present_position) / 4096 * 360;
                wire_angle2 = (float)(init_length2 - dxl2_present_position) / 4096 * 360;
                wire_angle3 = (float)(init_length3 - dxl3_present_position) / 4096 * 360;
                wire_angle4 = (float)(init_length4 - dxl4_present_position) / 4096 * 360;

                wire_length1 = 250 + (124 * wire_angle1 / 360);
                wire_length2 = 250 + (124 * wire_angle2 / 360);
                wire_length3 = 250 + (124 * wire_angle3 / 360);
                wire_length4 = 250 + (124 * wire_angle4 / 360);


                leftFinger_position_x = ((wire_length1 * wire_length1) - (wire_length2 * wire_length2) + (wire_length3 * wire_length3) - (wire_length4 * wire_length4)) / (4 * frame_line_length);
                leftFinger_position_y = ((-1) * (wire_length1 * wire_length1) - (wire_length2 * wire_length2) + (wire_length3 * wire_length3) + (wire_length4 * wire_length4)) / (4 * frame_line_length);
                leftFinger_position_z = ((wire_length1 * wire_length1) - (wire_length2 * wire_length2) - (wire_length3 * wire_length3) + (wire_length4 * wire_length4)) / (4 * frame_line_length);


                leftFinger_position_x = leftFinger_position_x / 10f;
                leftFinger_position_y = -leftFinger_position_y / 10f;
                leftFinger_position_z = -leftFinger_position_z / 10f;



                if (Mathf.Abs(Mathf.Abs(last_position_x) - Mathf.Abs(leftFinger_position_x)) >= 0.5f)
                {
                    leftFinger_position_x = last_position_x;
                }
                else
                {
                    last_position_x = leftFinger_position_x;
                }

                if (Mathf.Abs(Mathf.Abs(last_position_y) - Mathf.Abs(leftFinger_position_y)) >= 0.5f)
                {
                    leftFinger_position_y = last_position_y;
                }
                else
                {
                    last_position_y = leftFinger_position_y;
                }

                if (Mathf.Abs(Mathf.Abs(last_position_z) - Mathf.Abs(leftFinger_position_z)) >= 0.5f)
                {
                    leftFinger_position_z = last_position_z;
                }
                else
                {
                    last_position_z = leftFinger_position_z;
                }

                //********************************************  Calculate Moving_Object location  *****************************************




                //********************************************  Calculate Moter tension  *****************************************
                // calculate vector which is TargetObject to moters
                calcuate_MO_to_moter(leftFinger_position_x, leftFinger_position_y, leftFinger_position_z);

                //calculate dot product up to the final motor
                get_xyz_cos();

                // Initialze answer values.
                initializing_answer();

                //using inverse matrix, calcuate the force of a motor in the target direction
                make_tenstion_up();
                make_tenstion_down();
                make_tenstion_left();
                make_tenstion_right();
                make_tenstion_forward();
                make_tenstion_backward();

                //Combine the calculated values and enter the force applied to the motor
                Moving_Test();
                //********************************************  Calculate Moter tension  *****************************************
            }
        }
    }

    void Update()
    {
        //*******************************  You first press the button-N before press the button-M to operate normally  ********************************
        // Press N : Initialize moters
        if (Input.GetKeyDown(KeyCode.N) && timer_start == 0)
        {
            timer_start = 1;

            timer.Enabled = true;
            Debug.Log("Timer_Start");
        }


        // Press M : Calculating moters
        if (Input.GetKeyDown(KeyCode.M) && timer_start == 2)
        {
            input_force_flag = true;
            timer_start = 3;
            Debug.Log("Timer_Update");
        }

        // Initializing position of the target object to (0,-12,-12)
        if (Input.GetKeyDown(KeyCode.BackQuote) == true)
        {
            Object_going_0();
        }
    }
    //*************************************************  Code update section  ***************************************





    //*************************************************  Extra functions  *******************************************
    //Setting the Initial Location of an Object
    public static void Object_going_0()
    {
        dynamixel.groupSyncWriteClearParam(write_LeftControl);

        init_length1 = dxl1_present_position + 7470;
        init_length2 = dxl2_present_position + 3560;
        init_length3 = dxl3_present_position - 2730;
        init_length4 = dxl4_present_position + 3490;


        Debug.Log("\t\t x : " + dxl1_present_position + " " + dxl2_present_position + " " + dxl3_present_position + " " + dxl4_present_position);

    }
    

    private void OnApplicationQuit()
    {
        timer.Stop();
        timer.Dispose();
        timer.Close();
        timer.EndInit();
        timer.Enabled = false;

        #region Disable Dynamixel Torque
        // Disable Dynamixel#1 Torque
        DisableDynamixelTorque(port_leftControl, DXL1_ID);

        // Disable Dynamixel#2 Torque
        DisableDynamixelTorque(port_leftControl, DXL2_ID);

        // Disable Dynamixel#3 Torque
        DisableDynamixelTorque(port_leftControl, DXL3_ID);

        // Disable Dynamixel#2 Torque
        DisableDynamixelTorque(port_leftControl, DXL4_ID);

        // Disable Dynamixel#1 Torque
        DisableDynamixelTorque(port_rightControl, DXL5_ID);

        // Disable Dynamixel#2 Torque
        DisableDynamixelTorque(port_rightControl, DXL6_ID);

        // Disable Dynamixel#3 Torque
        DisableDynamixelTorque(port_rightControl, DXL7_ID);

        // Disable Dynamixel#2 Torque
        DisableDynamixelTorque(port_rightControl, DXL8_ID);
        #endregion

        // Close port

        dynamixel.closePort(port_leftControl);
        dynamixel.closePort(port_rightControl);

    }
    #endregion
    //*************************************************  Extra functions  *******************************************
}