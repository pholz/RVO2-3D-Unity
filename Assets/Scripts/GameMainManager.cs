﻿using System;
using System.Collections;
using System.Collections.Generic;
using Lean;
using RVO;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.Assertions.Comparers;
using UnityEngine.Experimental.UIElements;
using Random = System.Random;


public class GameMainManager : SingletonBehaviour<GameMainManager>
{
    public GameObject agentPrefab;

    [HideInInspector] public Vector3 mousePosition;

    private Plane mhPlane = new Plane(Vector3.up, Vector3.zero);
    private Dictionary<int, GameAgent> magentMap = new Dictionary<int, GameAgent>();

    [Range(0.0f, 10.0f)]
    public float AgentMaxSpeed = 2.0f;
    float m_agentMaxSpeed = 2.0f;

    // Use this for initialization
    void Start()
    {
        Simulator.Instance.setTimeStep(0.25f);
        Simulator.Instance.setAgentDefaults(30.0f, 10, 35.0f, 15.0f, 6.0f, 2.0f, Vector3.zero);
        // add in awake
        Simulator.Instance.processObstacles();
    }

    private void UpdateMousePosition()
    {
        Vector3 position = Vector3.zero;
        Ray mouseRay = Camera.main.ScreenPointToRay(Input.mousePosition);
        float rayDistance;
        if (mhPlane.Raycast(mouseRay, out rayDistance))
            position = mouseRay.GetPoint(rayDistance);

        mousePosition.x = position.x;
        mousePosition.y = position.z;
    }

    void DeleteAgent()
    {

        int agentNo = Simulator.Instance.queryNearAgent(mousePosition, 1.5f);
        if (agentNo == -1 || !magentMap.ContainsKey(agentNo))
            return;

        Simulator.Instance.delAgent(agentNo);
        LeanPool.Despawn(magentMap[agentNo].gameObject);
        magentMap.Remove(agentNo);
    }
    void CreatAgent(Vector3 v3)
    {
        int sid = Simulator.Instance.addAgent(v3);
        if (sid >= 0)
        {
            GameObject go = LeanPool.Spawn(agentPrefab, v3, Quaternion.identity);
            GameAgent ga = go.GetComponent<GameAgent>();
            Assert.IsNotNull(ga);
            ga.sid = sid;
            magentMap.Add(sid, ga);
        }
    }
    void CreatAgent()
    {
        int sid = Simulator.Instance.addAgent(mousePosition);
        if (sid >= 0)
        {
            GameObject go = LeanPool.Spawn(agentPrefab, new Vector3(mousePosition.x, 0, mousePosition.y), Quaternion.identity);
            GameAgent ga = go.GetComponent<GameAgent>();
            Assert.IsNotNull(ga);
            ga.sid = sid;
            magentMap.Add(sid, ga);
            Simulator.Instance.setAgentMaxSpeed(sid, m_agentMaxSpeed);
            //Simulator.Instance.setAgentNeighborDist(sid, 5.0f);
        }

    }

    // Update is called once per frame
    private void Update()
    {
        UpdateMousePosition();
        if (Input.GetMouseButtonUp(0) && Input.GetKey(KeyCode.LeftShift))
        {
            if (Input.GetKey(KeyCode.Delete))
            {
                DeleteAgent();
            }
            else
            {
                CreatAgent();
            }
        }

        Simulator.Instance.doStep();

        if (m_agentMaxSpeed != AgentMaxSpeed)
        {
            m_agentMaxSpeed = AgentMaxSpeed;

            foreach (Agent agt in Simulator.Instance.agents)
            {
                Simulator.Instance.setAgentMaxSpeed(agt.id, m_agentMaxSpeed);
            }
        }

        //foreach (Agent agt in Simulator.Instance.agents)
        //{

        //    magentMap[agt.id].goal = GameObject.Find("Gooal").transform.position;
        //}

    }

    public void CreateNewTargets()
    {
        int ct = Simulator.Instance.agents.Count;
        int i = 0;
        int rd = (int)(UnityEngine.Random.Range(0.0f, 100.0f));
        List<int> randoms = new List<int>();
        foreach (Agent agt in Simulator.Instance.agents)
        {
            // Vector3 dst = UnityEngine.Random.insideUnitSphere;
            int r;
            do
            {
                r = (int)(UnityEngine.Random.Range(0.0f, (float)(ct)));
            } while (randoms.Contains(r));

            randoms.Add(r);

            double rad = Math.PI * 2.0 * r / ct;
            Vector3 dst = new Vector3((float)Math.Cos(rad), 0.0f, (float)Math.Sin(rad));

            dst.y = 0.0f;
            dst.x *= 50.0f;
            dst.z *= 50.0f;
            magentMap[agt.id].goal = dst;

            i++;
        }
    }
}