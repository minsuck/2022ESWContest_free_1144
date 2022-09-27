using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEngine.XR;

public class GunMoving : MonoBehaviour
{
    public GameObject Finger;
    public GameObject XROri;
    public Camera VRcamera;

    public float lerpAimModeX;

    private Vector3 VRcamLocation;

    private Vector3 fingerLocation;
    private Vector3 lastLocation;
    private Vector3 saveLocation;
    private Vector3 pre_saveLocation;

    public int MAsize = 10;
    private float[] MovingAverageX = new float[1000];
    private float[] MovingAverageY = new float[1000];
    private float[] MovingAverageZ = new float[1000];
    private int pivot = 0;

    private int pre_saveLocation_check_cnt = 0;
    private bool pre_saveLocation_flag = false;
    private bool pre_saveLocation_change_flag = false;

    // Start is called before the first frame update
    void Start()
    {
        for(int i=0; i< MAsize; i++)
        {
            MovingAverageX[i] = 0;
            MovingAverageY[i] = 0;
            MovingAverageZ[i] = 0;
        }
        fingerLocation = Finger.transform.position;
        lastLocation = Finger.transform.position;
        this.transform.position = fingerLocation;

        pre_saveLocation.x = 0f;
        pre_saveLocation.y = 0f;
        pre_saveLocation.z = 0f;
    }

    // Update is called once per frame
    void Update()
    {
        if (Serial.Aim_mode == true)
        {
            //VRcamLocation.x = CommonUsages.rightEyePosition.a;

            saveLocation.x = 0;
            saveLocation.y = 0;
            saveLocation.z = 0;

            if (pivot < MAsize)
            {
                MovingAverageX[pivot] = Finger.transform.position.x;
                MovingAverageY[pivot] = Finger.transform.position.y;
                MovingAverageZ[pivot] = Finger.transform.position.z;
                pivot++;
            }
            else
            {
                pivot = 0;
            }

            for (int i = 0; i < MAsize; i++)
            {
                saveLocation.x += (MovingAverageX[i]);
                saveLocation.y += (MovingAverageY[i]);
                saveLocation.z += (MovingAverageZ[i]);
            }


            saveLocation.x = saveLocation.x / (float)MAsize;
            saveLocation.y = saveLocation.y / (float)MAsize;
            saveLocation.z = saveLocation.z / (float)MAsize;

            fingerLocation = new Vector3(saveLocation.x, saveLocation.y + 10.0f, saveLocation.z);
            XROri.transform.position = Vector3.Lerp(this.transform.position, fingerLocation, 0.5f);

            lerpAimModeX = 0.06f;
            VRcamLocation = VRcamera.transform.position;
            VRcamLocation.x += lerpAimModeX;
            VRcamLocation.y -= 0.77f;
            VRcamLocation.z += 0.22f;

            this.transform.position = Vector3.Lerp(this.transform.position, VRcamLocation, 1.0f);
        }
        else
        {
            saveLocation.x = 0;
            saveLocation.y = 0;
            saveLocation.z = 0;

            if (pivot < MAsize)
            {
                MovingAverageX[pivot] = Finger.transform.position.x / 12.0f;
                MovingAverageY[pivot] = Finger.transform.position.y / 12.0f;
                MovingAverageZ[pivot] = Finger.transform.position.z / 12.0f;
                pivot++;
            }
            else
            {
                pivot = 0;
            }

            for (int i = 0; i < MAsize; i++)
            {
                saveLocation.x += (MovingAverageX[i]);
                saveLocation.y += (MovingAverageY[i]);
                saveLocation.z += (MovingAverageZ[i]);
            }


            saveLocation.x = saveLocation.x / (float)MAsize;
            saveLocation.y = saveLocation.y / (float)MAsize;
            saveLocation.z = saveLocation.z / (float)MAsize;

            fingerLocation = new Vector3(saveLocation.x, saveLocation.y, saveLocation.z);
            this.transform.position = Vector3.Lerp(this.transform.position, fingerLocation, 0.5f);
            XROri.transform.position = new Vector3(-0.3f, 0.7f, -0.5f);
        }
    }
}
