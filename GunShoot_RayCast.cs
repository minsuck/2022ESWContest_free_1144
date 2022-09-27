using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GunShoot_RayCast : MonoBehaviour
{
    public GameObject k2Muzzle;
    public SyncWrite_kinesthetic fs;


    public float force_Gun = 5.0f;
    public float time_gun = 0.3f;
    public int MAX_BULLET = 10;
    private int Bullet_Left;
    private bool nowReload = false;

    // Start is called before the first frame update
    void Start()
    {
        Bullet_Left = MAX_BULLET;
    }

    // Update is called once per frame
    void Update()
    {
        if (Serial.Shoot == true)
        {
            if (nowReload == false)
            {
                raycast_gunShoot();
                rebound_gunShoot();
                Bullet_Left--;
                nowReload = true;
            }
        }
        else
        {
            nowReload = false;
        }

        if (Bullet_Left == 0)
        {
            Debug.Log("Bullet run out!");
        }

        if (Input.GetKeyDown(KeyCode.R))
        {
            Bullet_Left = MAX_BULLET;
        }
    }

    void raycast_gunShoot()
    {
        RaycastHit hit = new RaycastHit();
        
        Ray ray = new Ray(k2Muzzle.transform.position, transform.forward);

        Debug.DrawRay(ray.origin, ray.direction * 100.0f, Color.red, 0.1f);

        if (Physics.Raycast(ray.origin, ray.direction, out hit))
        {
            if (hit.collider.tag == "target")
            {
                Target_Moves.targetShooted = true;
                Debug.Log(hit.collider.name);
            }
        }
    }
    void rebound_gunShoot()
    {
        fs.input_force_flag = true;
        fs.goal_N_high = force_Gun * 2;
        fs.goal_N_backward = (-1) * force_Gun * 2;

        Invoke("gun_invoke", time_gun);
    }

    public void gun_invoke()       //Functions used in the Invoke() function
    {
        fs.Time_control_limit();
    }
}
