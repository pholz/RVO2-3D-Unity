using System;
using System.Collections;
using System.Collections.Generic;
using RVO;
using UnityEngine;
using Random = System.Random;


public class GameAgent : MonoBehaviour
{
    [HideInInspector] public int sid = -1;

    public Vector3 goal = new Vector3();

    public GameObject orcaObj;

    /** Random number generator. */
    private Random mrandom = new Random();
    // Use this for initialization
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
        if (sid >= 0)
        {
            Vector3 pos = Simulator.Instance.getAgentPosition(sid);
            Vector3 vel = Simulator.Instance.getAgentPrefVelocity(sid);
            transform.position = new Vector3(pos.x, transform.position.y, pos.y);
            if (Math.Abs(vel.x) > 0.01f && Math.Abs(vel.y) > 0.01f)
                transform.forward = new Vector3(vel.x, 0, vel.y).normalized;
        }

        GetComponentInChildren<TextMesh>().text = "n: " + Simulator.Instance.getAgentNumAgentNeighbors(sid);
        GetComponentInChildren<LineRenderer>().SetPositions(new Vector3[] { transform.position, goal });

        Vector3 goalTf = new Vector3(goal.x, goal.z, goal.y);
        Vector3 goalVec = goalTf - Simulator.Instance.getAgentPosition(sid);
        if (RVOMath.absSq(goalVec) > 1.0f)
        {
            goalVec = RVOMath.normalize(goalVec);
        }

        Simulator.Instance.setAgentPrefVelocity(sid, goalVec);

        float angle_ = (float)mrandom.NextDouble() * 2.0f * (float)Math.PI;
        float dist_ = (float)mrandom.NextDouble() * 0.0001f;

        Simulator.Instance.setAgentPrefVelocity(sid, Simulator.Instance.getAgentPrefVelocity(sid) +
                                                     dist_ *
                                                     new Vector3((float)Math.Cos(angle_), (float)Math.Sin(angle_)));




        //foreach (var c in orcaObj.GetComponentsInChildren<LineRenderer>())
        //{
        //    Destroy(c.gameObject);
        //}

        //var oLines = Simulator.Instance.getAgentOrcaLines(sid);

        //foreach (Line line in oLines)
        //{
        //    GameObject go = new GameObject();
        //    LineRenderer r = go.AddComponent<LineRenderer>();
        //    go.transform.parent = orcaObj.transform;
        //    var p1 = orcaObj.transform.position + line.point - line.direction * 20.0f;
        //    var p2 = orcaObj.transform.position + line.point + line.direction * 20.0f;
        //    p1.z = p1.y;
        //    p1.y = 0.0f;
        //    p2.z = p2.y;
        //    p2.y = 0.0f;
        //    r.SetPositions(new Vector3[] { p1, p2 });
        //}




        return;

        if (!Input.GetMouseButton(1))
        {
            Simulator.Instance.setAgentPrefVelocity(sid, new Vector3(0, 0));
            return;
        }

        Vector3 goalVector = GameMainManager.Instance.mousePosition - Simulator.Instance.getAgentPosition(sid);
        if (RVOMath.absSq(goalVector) > 1.0f)
        {
            goalVector = RVOMath.normalize(goalVector);
        }

        Simulator.Instance.setAgentPrefVelocity(sid, goalVector);

        /* Perturb a little to avoid deadlocks due to perfect symmetry. */
        float angle = (float) mrandom.NextDouble()*2.0f*(float) Math.PI;
        float dist = (float) mrandom.NextDouble()*0.0001f;

        Simulator.Instance.setAgentPrefVelocity(sid, Simulator.Instance.getAgentPrefVelocity(sid) +
                                                     dist*
                                                     new Vector3((float) Math.Cos(angle), (float) Math.Sin(angle)));
    }
}