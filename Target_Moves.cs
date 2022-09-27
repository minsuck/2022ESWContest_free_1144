using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Target_Moves : MonoBehaviour
{
    public static int MAX = 3;
    public float turnSpeed = 1.0f;
    private float worldTime = 0.0f;

    private int targetCount = 0;
    private int TARGETNUM = 0;
    private int[] TargetSequence = { 0, 1, 2, 0, 1};
    private int targetPivot = 0;

    private float checkTime = 0.0f;
    private float standingTime = 10.0f;
    private float restTime = 2.0f;

    private bool gameStart_flag = false;
    public static bool targetShooted = false;
    public static bool targetActive = false;

    Quaternion targetLie = Quaternion.Euler(new Vector3(90, 0, 0));
    Quaternion targetStand = Quaternion.Euler(new Vector3(0, 0, 0));

    public GameObject[] target = new GameObject[MAX];

    // Start is called before the first frame update
    void Start()
    {
        for (int i = 0; i < MAX; i++)
        {
            //target[i].transform.rotation = targetLie;

            target[i].transform.rotation = targetStand;
        }
    }

    // Update is called once per frame
    void Update()
    {
        worldTime += Time.deltaTime;    // Time counting after game started.

        if (Input.GetKeyDown(KeyCode.A))    //  'A' is game start code.
        {
            gameStart_flag = true;
            targetActive = false;
            targetShooted = false;
            targetCount = 0;
            worldTime = 0.0f;
            checkTime = 0.0f;
            targetPivot = 0;
            //TARGETNUM = Random.Range(0, MAX);
            TARGETNUM = TargetSequence[targetPivot];
            Debug.Log("Game Start");
        }

        if (gameStart_flag == true)
        {
            checkTime += Time.deltaTime;

            if (targetCount < 5)            // target give you 10 chances
            {
                targetSystemWorking();

                //Debug.Log("Time : " + checkTime + ", Target passed : " + targetCount + ", TargetNum : " + TARGETNUM);
            }
            else
            {
                gameStart_flag = false;
                Debug.Log("Game Over");
            }
        }
        else
        {
            targetLying();
        }
    }
    void targetSystemWorking()
    {
        if (checkTime < restTime)
        {
            targetLying();
        }
        else if (checkTime < standingTime && targetShooted == false)
        {
            targetActive = true;
            targetStanding();
        }
        else
        {
            checkTime = 0.0f;
            targetShooted = false;
            targetActive = false;

            targetCount++;
            targetPivot++;
            TARGETNUM = TargetSequence[targetPivot];
        }
    }

    void targetStanding()
    {
        target[TARGETNUM].transform.rotation = Quaternion.Slerp(target[TARGETNUM].transform.rotation, targetStand, turnSpeed * Time.deltaTime);
    }
    void targetLying()
    {
        for (int i = 0; i < MAX; i++)
        {
            target[i].transform.rotation = Quaternion.Slerp(target[i].transform.rotation, targetLie, turnSpeed * Time.deltaTime);
        }
    }
}
