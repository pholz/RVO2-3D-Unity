/*
 * Agent.cs
 * RVO2 Library C#
 *
 * Copyright 2008 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

using System;
using UnityEngine;
using System.Collections.Generic;

namespace RVO
{
    /**
     * <summary>Defines an agent in the simulation.</summary>
     */
    internal class Agent
    {
        internal IList<KeyValuePair<float, Agent>> agentNeighbors = new List<KeyValuePair<float, Agent>>();
        internal IList<KeyValuePair<float, Obstacle>> obstacleNeighbors = new List<KeyValuePair<float, Obstacle>>();
        internal IList<Plane> orcaPlanes = new List<Plane>();
        internal Vector3 position;
        internal Vector3 prefVelocity;
        internal Vector3 velocity;
        internal int id = 0;
        internal int maxNeighbors = 0;
        internal float maxSpeed = 0.0f;
        internal float neighborDist = 0.0f;
        internal float radius = 0.0f;
        internal float timeHorizon = 0.0f;
        internal float timeHorizonObst = 0.0f;
        internal bool needDelete = false;

        private Vector3 newVelocity;

        /**
         * <summary>Computes the neighbors of this agent.</summary>
         */
        internal void computeNeighbors()
        {
            obstacleNeighbors.Clear();
            float rangeSq = RVOMath.sqr(timeHorizonObst * maxSpeed + radius);
            Simulator.Instance.kdTree.computeObstacleNeighbors(this, rangeSq);

            agentNeighbors.Clear();

            if (maxNeighbors > 0)
            {
                rangeSq = RVOMath.sqr(neighborDist);
                Simulator.Instance.kdTree.computeAgentNeighbors(this, ref rangeSq);
            }
        }

        /**
         * <summary>Computes the new velocity of this agent.</summary>
         */
        internal void computeNewVelocity()
        {
            orcaPlanes.Clear();

            float invTimeHorizonObst = 1.0f / timeHorizonObst;

            int numObstLines = orcaPlanes.Count;

            float invTimeHorizon = 1.0f / timeHorizon;

            Debug.Log("agt " + id + " num neighbours: " + agentNeighbors.Count);

            /* Create agent ORCA lines. */
            for (int i = 0; i < agentNeighbors.Count; ++i)
            {
                Agent other = agentNeighbors[i].Value;

                Vector3 relativePosition = other.position - position;
                Vector3 relativeVelocity = velocity - other.velocity;
                float distSq = RVOMath.absSq(relativePosition);
                float combinedRadius = radius + other.radius;
                float combinedRadiusSq = RVOMath.sqr(combinedRadius);

                Plane plane;
                Vector3 u;

                if (distSq > combinedRadiusSq)
                {
                    /* No collision. */
                    Vector3 w = relativeVelocity - invTimeHorizon * relativePosition;

                    /* Vector from cutoff center to relative velocity. */
                    float wLengthSq = RVOMath.absSq(w);
                    float dotProduct1 = Vector3.Dot(w , relativePosition);

                    if (dotProduct1 < 0.0f && RVOMath.sqr(dotProduct1) > combinedRadiusSq * wLengthSq)
                    {
                        /* Project on cut-off circle. */
                        float wLength = RVOMath.sqrt(wLengthSq);
                        Vector3 unitW = w / wLength;

                        plane.normal = unitW;
                        u = (combinedRadius * invTimeHorizon - wLength) * unitW;
                    }
                    else
                    {
                        float a = distSq;
                        float b = Vector3.Dot(relativePosition, relativeVelocity);
                        float c = RVOMath.absSq(relativeVelocity) -
                            RVOMath.absSq(Vector3.Cross(relativePosition, relativeVelocity)) / (distSq - combinedRadiusSq);
                        float t = (b + RVOMath.sqrt(b * b - a * c)) / a;
                        Vector3 w2 = relativeVelocity - relativePosition * t;
                        float wLength = RVOMath.abs(w2);
                        Vector3 unitW = w2 / wLength;

                        plane.normal = unitW;
                        u = (combinedRadius * t - wLength) * unitW;
                    }
                }
                else
                {
                    /* Collision. Project on cut-off circle of time timeStep. */
                    float invTimeStep = 1.0f / Simulator.Instance.timeStep;

                    /* Vector from cutoff center to relative velocity. */
                    Vector3 w = relativeVelocity - invTimeStep * relativePosition;

                    float wLength = RVOMath.abs(w);
                    Vector3 unitW = w / wLength;

                    plane.normal = unitW;
                    u = (combinedRadius * invTimeStep - wLength) * unitW;
                }

                plane.point = velocity + 0.5f * u;
                orcaPlanes.Add(plane);
            }

            int planeFail = linearProgram3(orcaPlanes, maxSpeed, prefVelocity, false, ref newVelocity);

            if (planeFail < orcaPlanes.Count)
            {
                linearProgram4(orcaPlanes, planeFail, maxSpeed, ref newVelocity);
            }
        }

        /**
         * <summary>Inserts an agent neighbor into the set of neighbors of this
         * agent.</summary>
         *
         * <param name="agent">A pointer to the agent to be inserted.</param>
         * <param name="rangeSq">The squared range around this agent.</param>
         */
        internal void insertAgentNeighbor(Agent agent, ref float rangeSq)
        {
            if (this != agent)
            {
                float distSq = RVOMath.absSq(position - agent.position);

                if (distSq < rangeSq)
                {
                    if (agentNeighbors.Count < maxNeighbors)
                    {
                        agentNeighbors.Add(new KeyValuePair<float, Agent>(distSq, agent));
                    }

                    int i = agentNeighbors.Count - 1;

                    while (i != 0 && distSq < agentNeighbors[i - 1].Key)
                    {
                        agentNeighbors[i] = agentNeighbors[i - 1];
                        --i;
                    }

                    agentNeighbors[i] = new KeyValuePair<float, Agent>(distSq, agent);

                    if (agentNeighbors.Count == maxNeighbors)
                    {
                        rangeSq = agentNeighbors[agentNeighbors.Count - 1].Key;
                    }
                }
            }
        }

        /**
         * <summary>Inserts a static obstacle neighbor into the set of neighbors
         * of this agent.</summary>
         *
         * <param name="obstacle">The number of the static obstacle to be
         * inserted.</param>
         * <param name="rangeSq">The squared range around this agent.</param>
         */
        internal void insertObstacleNeighbor(Obstacle obstacle, float rangeSq)
        {
            Obstacle nextObstacle = obstacle.next;

            float distSq = RVOMath.distSqPointLineSegment(obstacle.point, nextObstacle.point, position);

            if (distSq < rangeSq)
            {
                obstacleNeighbors.Add(new KeyValuePair<float, Obstacle>(distSq, obstacle));

                int i = obstacleNeighbors.Count - 1;

                while (i != 0 && distSq < obstacleNeighbors[i - 1].Key)
                {
                    obstacleNeighbors[i] = obstacleNeighbors[i - 1];
                    --i;
                }
                obstacleNeighbors[i] = new KeyValuePair<float, Obstacle>(distSq, obstacle);
            }
        }

        /**
         * <summary>Updates the two-dimensional position and two-dimensional
         * velocity of this agent.</summary>
         */
        internal void update()
        {
            Vector3 accel = (newVelocity - velocity) * Simulator.Instance.timeStep;
            velocity += accel;
            position += velocity * Simulator.Instance.timeStep;
        }

        /**
         * <summary>Solves a one-dimensional linear program on a specified line
         * subject to linear constraints defined by lines and a circular
         * constraint.</summary>
         *
         * <returns>True if successful.</returns>
         *
         * <param name="lines">Lines defining the linear constraints.</param>
         * <param name="lineNo">The specified line constraint.</param>
         * <param name="radius">The radius of the circular constraint.</param>
         * <param name="optVelocity">The optimization velocity.</param>
         * <param name="directionOpt">True if the direction should be optimized.
         * </param>
         * <param name="result">A reference to the result of the linear program.
         * </param>
         */
        private bool linearProgram1(IList<Plane> planes, int planeNo, Line line, 
            float radius, Vector3 optVelocity, bool directionOpt, ref Vector3 result)
        {
            float dotProduct = Vector3.Dot(line.point , line.direction);
            float discriminant = RVOMath.sqr(dotProduct) + RVOMath.sqr(radius) - RVOMath.absSq(line.point);

            if (discriminant < 0.0f)
            {
                /* Max speed circle fully invalidates line lineNo. */
                return false;
            }

            float sqrtDiscriminant = RVOMath.sqrt(discriminant);
            float tLeft = -dotProduct - sqrtDiscriminant;
            float tRight = -dotProduct + sqrtDiscriminant;

            for (int i = 0; i < planeNo; ++i)
            {
                float numerator = Vector3.Dot((planes[i].point - line.point), planes[i].normal);
                float denominator = Vector3.Dot(line.direction, planes[i].normal);

                if (RVOMath.sqr(denominator) <= RVOMath.RVOEPSILON)
                {
                    /* Lines lineNo and i are (almost) parallel. */
                    if (numerator > 0.0f)
                    {
                        return false;
                    }

                    continue;
                }

                float t = numerator / denominator;

                if (denominator >= 0.0f)
                {
                    /* Plane i bounds line on the left. */
                    tLeft = Math.Max(tLeft, t);

                }
                else
                {
                    /* Plane i bounds line on the right. */
                    tRight = Math.Min(tRight, t);
                }

                if (tLeft > tRight)
                {
                    return false;
                }
            }

            if (directionOpt)
            {
                /* Optimize direction. */
                if (Vector3.Dot(optVelocity, line.direction) > 0.0f)
                {
                    /* Take right extreme. */
                    result = line.point + tRight * line.direction;
                }
                else
                {
                    /* Take left extreme. */
                    result = line.point + tLeft * line.direction;
                }
            }
            else
            {
                /* Optimize closest point. */
                float t = Vector3.Dot(line.direction, (optVelocity - line.point));

                if (t < tLeft)
                {
                    result = line.point + tLeft * line.direction;
                }
                else if (t > tRight)
                {
                    result = line.point + tRight * line.direction;
                }
                else
                {
                    result = line.point + t * line.direction;
                }
            }

            return true;
        }

        bool linearProgram2(IList<Plane> planes, int planeNo, float radius, Vector3 optVelocity, bool directionOpt, ref Vector3 result)
        {
            float planeDist = Vector3.Dot(planes[planeNo].point, planes[planeNo].normal);
            float planeDistSq = RVOMath.sqr(planeDist);
            float radiusSq = RVOMath.sqr(radius);

            if (planeDistSq > radiusSq) {
                /* Max speed sphere fully invalidates plane planeNo. */
                return false;
            }

            float planeRadiusSq = radiusSq - planeDistSq;

            Vector3 planeCenter = planeDist * planes[planeNo].normal;

            if (directionOpt) 
            {
                /* Project direction optVelocity on plane planeNo. */
                Vector3 planeOptVelocity = optVelocity - Vector3.Dot(optVelocity, planes[planeNo].normal) * planes[planeNo].normal;
                float planeOptVelocityLengthSq = RVOMath.absSq(planeOptVelocity);

                if (planeOptVelocityLengthSq <= RVOMath.RVOEPSILON) 
                {
                    result = planeCenter;
                }
                else 
                {
                    result = planeCenter + RVOMath.sqrt(planeRadiusSq / planeOptVelocityLengthSq) * planeOptVelocity;
                }
            }
            else 
            {
                /* Project point optVelocity on plane planeNo. */
                result = optVelocity + Vector3.Dot((planes[planeNo].point - optVelocity), planes[planeNo].normal) * planes[planeNo].normal;

                /* If outside planeCircle, project on planeCircle. */
                if (RVOMath.absSq(result) > radiusSq) 
                {
                    Vector3 planeResult = result - planeCenter;
                    float planeResultLengthSq = RVOMath.absSq(planeResult);
                    result = planeCenter + RVOMath.sqrt(planeRadiusSq / planeResultLengthSq) * planeResult;
                }
            }

            for (int i = 0; i < planeNo; ++i) 
            {
                if (Vector3.Dot(planes[i].normal, (planes[i].point - result)) > 0.0f) 
                {
                    /* Result does not satisfy constraint i. Compute new optimal result. */
                    /* Compute intersection line of plane i and plane planeNo. */
                    Vector3 crossProduct = Vector3.Cross(planes[i].normal, planes[planeNo].normal);

                    if (RVOMath.absSq(crossProduct) <= RVOMath.RVOEPSILON) 
                    {
                        /* Planes planeNo and i are (almost) parallel, and plane i fully invalidates plane planeNo. */
                        return false;
                    }

                    Line line;
                    line.direction = RVOMath.normalize(crossProduct);
                    Vector3 lineNormal = Vector3.Cross(line.direction, planes[planeNo].normal);
                    line.point = planes[planeNo].point + 
                        (Vector3.Dot((planes[i].point - planes[planeNo].point), planes[i].normal) / Vector3.Dot(lineNormal, planes[i].normal)) 
                        * lineNormal;

                    if (!linearProgram1(planes, i, line, radius, optVelocity, directionOpt, ref result)) 
                    {
                        return false;
                    }
                }
            }

            return true;
        }

        /**
         * <summary>Solves a two-dimensional linear program subject to linear
         * constraints defined by lines and a circular constraint.</summary>
         *
         * <returns>The number of the line it fails on, and the number of lines
         * if successful.</returns>
         *
         * <param name="lines">Lines defining the linear constraints.</param>
         * <param name="radius">The radius of the circular constraint.</param>
         * <param name="optVelocity">The optimization velocity.</param>
         * <param name="directionOpt">True if the direction should be optimized.
         * </param>
         * <param name="result">A reference to the result of the linear program.
         * </param>
         */
        private int linearProgram3(IList<Plane> planes, float radius, Vector3 optVelocity, bool directionOpt, ref Vector3 result)
        {
            if (directionOpt)
            {
                /*
                 * Optimize direction. Note that the optimization velocity is of
                 * unit length in this case.
                 */
                result = optVelocity * radius;
            }
            else if (RVOMath.absSq(optVelocity) > RVOMath.sqr(radius))
            {
                /* Optimize closest point and outside circle. */
                result = RVOMath.normalize(optVelocity) * radius;
            }
            else
            {
                /* Optimize closest point and inside circle. */
                result = optVelocity;
            }

            for (int i = 0; i < planes.Count; ++i)
            {
                if (Vector3.Dot(planes[i].normal, planes[i].point - result) > 0.0f)
                {
                    /* Result does not satisfy constraint i. Compute new optimal result. */
                    Vector3 tempResult = result;
                    if (!linearProgram2(planes, i, radius, optVelocity, directionOpt, ref result))
                    {
                        result = tempResult;

                        return i;
                    }
                }
            }

            return planes.Count;
        }

        /**
         * <summary>Solves a two-dimensional linear program subject to linear
         * constraints defined by lines and a circular constraint.</summary>
         *
         * <param name="lines">Lines defining the linear constraints.</param>
         * <param name="numObstLines">Count of obstacle lines.</param>
         * <param name="beginLine">The line on which the 2-d linear program
         * failed.</param>
         * <param name="radius">The radius of the circular constraint.</param>
         * <param name="result">A reference to the result of the linear program.
         * </param>
         */
        private void linearProgram4(IList<Plane> planes, int beginPlane, float radius, ref Vector3 result)
        {
            float distance = 0.0f;

            for (int i = beginPlane; i < planes.Count; ++i)
            {
                if (Vector3.Dot(planes[i].normal, planes[i].point - result) > distance)
                {
                    /* Result does not satisfy constraint of line i. */
                    IList<Plane> projPlanes = new List<Plane>();

                    for (int j = 0; j < i; ++j)
                    {
                        Plane plane;

                        Vector3 crossProduct = Vector3.Cross(planes[j].normal, planes[i].normal);

                        if (RVOMath.absSq(crossProduct) <= RVOMath.RVOEPSILON)
                        {
                            /* Line i and line j are parallel. */
                            if (Vector3.Dot(planes[i].normal, planes[j].normal) > 0.0f)
                            {
                                /* Line i and line j point in the same direction. */
                                continue;
                            }
                            else
                            {
                                /* Line i and line j point in opposite direction. */
                                plane.point = 0.5f * (planes[i].point + planes[j].point);
                            }
                        }
                        else
                        {
                            Vector3 lineNormal = Vector3.Cross(crossProduct, planes[i].normal);
                            plane.point = planes[i].point + 
                                (Vector3.Dot((planes[j].point - planes[i].point), planes[j].normal) / Vector3.Dot(lineNormal, planes[j].normal))
                                * lineNormal;
                        }

                        plane.normal = RVOMath.normalize(planes[j].normal - planes[i].normal);
                        projPlanes.Add(plane);
                    }

                    Vector3 tempResult = result;

                    if (linearProgram3(projPlanes, radius, planes[i].normal, true, ref result) < projPlanes.Count)
                    {
                        /*
                         * This should in principle not happen. The result is by
                         * definition already in the feasible region of this
                         * linear program. If it fails, it is due to small
                         * floating point error, and the current result is kept.
                         */
                        result = tempResult;
                    }

                    distance = Vector3.Dot(planes[i].normal, (planes[i].point - result));
                }
            }
        }
    }
}
