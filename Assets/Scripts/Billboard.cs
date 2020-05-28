using UnityEngine;
using System.Collections;

public class Billboard : MonoBehaviour
{
    void Update() 
    {
        if (Camera.main.orthographic == true)
        {
            transform.rotation = Quaternion.Euler(-90, Camera.main.transform.rotation.eulerAngles.y + 180, 0);
        }
        else
        {
            transform.localScale = new Vector3(-1, 1, 1);
            // transform.localScale *= (Vector3.Distance(Camera.main.transform.position, transform.position)) * 0.1f;
            transform.LookAt(Camera.main.transform.position, Camera.main.transform.up);
        }
    }
}
