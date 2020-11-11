using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;

public class Agent : MonoBehaviour
{
    private struct WallCollisionData
    {
        public Vector3 collisionPoint;
        public Vector3 sideNormal;
    }

    public float radius;
    public float mass;
    public float perceptionRadius;

    public string agentTag = "Agent";
    public string wallTag = "Wall";

    private List<Vector3> path;
    private NavMeshAgent nma;
    private Rigidbody rb;

    private HashSet<GameObject> perceivedNeighbors = new HashSet<GameObject>();
    private HashSet<GameObject> perceivedWalls = new HashSet<GameObject>();
    private Dictionary<int, WallCollisionData> agentWallCollisionData = new Dictionary<int, WallCollisionData>();

    public bool visualize = true;
    public bool visualizeForces = false;
    public LayerMask deadlockLayerMask;

    public bool wallHugger = false;
    public bool noGoal = false;
    public bool EVADER = false;
    public bool CHASER = false;

    public bool FOLLOWER = false;
    public bool LEADER = false;
    public Agent leaderAgent;

    public Vector3 goalForce = Vector3.zero;

    public LayerMask chaserLayer;
    public LayerMask evaderLayer;

    private Renderer agentRenderer;

    public bool cohessive = false;
    public float cohessiveness = 0.7f;

    void Start()
    {
        path = new List<Vector3>();
        nma = GetComponent<NavMeshAgent>();
        rb = GetComponent<Rigidbody>();
        agentRenderer = GetComponent<Renderer>();
        if (CHASER || EVADER || LEADER || FOLLOWER)
        {
            agentRenderer.material.color = (CHASER || LEADER) ? Color.red : Color.green;
        }

        if (EVADER || CHASER)
        {
            perceptionRadius = 100.0f;
        }

        gameObject.transform.localScale = new Vector3(2 * radius, 1, 2 * radius);
        nma.radius = radius;
        rb.mass = mass;
        GetComponent<SphereCollider>().radius = perceptionRadius / 2;
    }

    private void Update()
    {
        if (path.Count > 1 && Vector3.Distance(transform.position, path[0]) < 1.1f)
        {
            path.RemoveAt(0);
        }
        else if (path.Count == 1 && Vector3.Distance(transform.position, path[0]) < 2f)
        {
            path.RemoveAt(0);

            if (path.Count == 0)
            {
                gameObject.SetActive(false);
                AgentManager.RemoveAgent(gameObject);
            }
        }

        #region Visualization

        if (visualize)
        {
            if (path.Count > 0)
            {
                Debug.DrawLine(transform.position, path[0], Color.green);
            }
            for (int i = 0; i < path.Count - 1; i++)
            {
                Debug.DrawLine(path[i], path[i + 1], Color.yellow);
            }
        }

        if (visualize)
        {
            foreach (var neighbor in perceivedNeighbors)
            {
                Debug.DrawLine(transform.position, neighbor.transform.position, Color.yellow);
            }
        }

        #endregion
    }

    #region Public Functions

    public void ComputePath(Vector3 destination)
    {
        nma.enabled = true;
        var nmPath = new NavMeshPath();
        nma.CalculatePath(destination, nmPath);
        path = nmPath.corners.Skip(1).ToList();
        //path = new List<Vector3>() { destination };
        //nma.SetDestination(destination);
        nma.enabled = false;
    }

    public Vector3 GetVelocity()
    {
        return rb.velocity;
    }

    #endregion

    #region Incomplete Functions

    private Vector3 ComputeForce()
    {
        goalForce = Vector3.zero;
        Vector3 agentForce = CalculateAgentForce();
        if (cohessive)
        {
            agentForce *= -cohessiveness;
        }
        Vector3 wallForce = Vector3.zero;

        Vector3 force = Vector3.zero;
        if (wallHugger)
        {
            wallForce = CalculateWallHuggingForce();
        }
        else
        {
            wallForce = CalculateWallForce();
        }

        if (EVADER || CHASER)
        {
            goalForce = CalculateActionSceneForce(EVADER);
            goalForce = CalculateEscapeTrappedForce(goalForce);
        }
        else if(FOLLOWER)
        {
            goalForce = CalculateGoalForce(leaderAgent.transform.position);
            goalForce = ShowFollowerTheRightWay(goalForce);
            //agentForce *= -0.25f;
            agentForce *= 0.1f;
        }
        else if(!noGoal)
        {
            goalForce = CalculateGoalForce(path[0]);
        }

        if (visualizeForces)
        {
            Debug.DrawRay(transform.position, goalForce, Color.red);
            //Debug.Log(perceivedNeighbors.Count);
            Debug.DrawRay(transform.position, agentForce, Color.cyan);
            Debug.DrawRay(transform.position, wallForce, Color.yellow);
        }

        force = goalForce + agentForce + wallForce;

        if (force != Vector3.zero)
        {
            Debug.DrawRay(transform.position, force, Color.green);
            return force.normalized * Mathf.Min(force.magnitude, Parameters.maxSpeed);
        }
        else
        {
            return Vector3.zero;
        }
    }

    private Vector3 CalculateGoalForce(Vector3 position)
    {
        Vector3 force = Vector3.zero;

        if(path.Count > 0 || FOLLOWER)
        {
            Vector3 goalAtSameLvl = new Vector3(position.x, transform.position.y, position.z);
            force = (goalAtSameLvl - transform.position).normalized * nma.speed;
            force -= GetVelocity();
            force /= 1.0f;
        }

        return force.normalized;
    }

    private Vector3 CalculateAgentForce()
    {
        Vector3 agentForce = Vector3.zero;
        foreach (GameObject otherAgent in perceivedNeighbors)
        {
            if (otherAgent.activeSelf)
            {
                //Debug.DrawLine(transform.position, otherAgent.transform.position, Color.blue);

                Vector3 thisAgentSocialForce = Vector3.zero;

                float distBtwAgents = Vector3.Distance(otherAgent.transform.position, transform.position);
                float sumOfRadius = perceptionRadius * 2;
                float dif = sumOfRadius - distBtwAgents;

                float psychologicalForce = Mathf.Exp(dif) / 10000.0f;

                Vector3 dirToOtherAgent = otherAgent.transform.position - transform.position;

                float agentRadiusSum = nma.radius * 2;
                float g = 0.0f;
                if (distBtwAgents <= agentRadiusSum)
                {
                    g = 1.0f;
                }

                if (g != 0.0f)
                {
                    float nonPenetrationForce = g * Mathf.Abs(dif);

                    Vector3 tangentialDir = Vector3.Cross(dirToOtherAgent, Vector3.up);
                    if (Vector3.Dot(tangentialDir, GetVelocity()) < 0)
                    {
                        tangentialDir *= -1;
                    }

                    float tangentialVelA = g * dif * Vector3.Dot(GetVelocity(), tangentialDir);
                    float tangentialVelB = g * dif * Vector3.Dot(otherAgent.GetComponent<Agent>().GetVelocity(), tangentialDir);

                    Vector3 tangentialPoint = transform.position + dirToOtherAgent.normalized * nma.radius;

                    Vector3 slidingForce = tangentialDir * (tangentialVelA - tangentialVelB);
                    //Debug.DrawRay(tangentialPoint, tangentialDir * agentRadiusSum, Color.red);

                    thisAgentSocialForce = -1 * dirToOtherAgent * (psychologicalForce + nonPenetrationForce) + slidingForce;
                }
                else
                {
                    thisAgentSocialForce = -1 * dirToOtherAgent * psychologicalForce;
                }

                agentForce += thisAgentSocialForce;

            }
        }
        return agentForce.normalized;
    }

    private Vector3 CalculateWallForce()
    {
        Vector3 wallForce = Vector3.zero;

        foreach(GameObject wall in perceivedWalls)
        {
            Vector3 thisAgentSocialForce = Vector3.zero;

            int wallID = wall.GetInstanceID();

            float distBtwObjects = Vector3.Distance(wall.transform.position, transform.position);

            float agentRadiusSum = nma.radius;
            //Debug.Log(wall.transform.localScale.magnitude);
            float dif = agentRadiusSum - distBtwObjects;

            float psychologicalForce = Mathf.Exp(dif) / 1000.0f;

            Vector3 dirTowardsWall = wall.transform.position - transform.position;

            if (agentWallCollisionData.ContainsKey(wallID))
            {
                dirTowardsWall = agentWallCollisionData[wallID].collisionPoint - transform.position;

                float nonPenetrationForce = Mathf.Abs(dif);

                Vector3 tangentialDir = Vector3.Cross(agentWallCollisionData[wallID].sideNormal, Vector3.up);
                if(Vector3.Dot(tangentialDir, GetVelocity()) < 0)
                {
                    tangentialDir *= -1;
                }

                float tangentialVelA = dif * Vector3.Dot(GetVelocity(), tangentialDir);

                Vector3 tangentialPoint = agentWallCollisionData[wallID].collisionPoint;

                Vector3 slidingForce = tangentialDir * (tangentialVelA);
                //Debug.DrawRay(tangentialPoint, tangentialDir, Color.red);
                //Debug.DrawRay(tangentialPoint, agentWallCollisionData[wallID].sideNormal, Color.cyan);

                thisAgentSocialForce = -1 * dirTowardsWall * psychologicalForce + agentWallCollisionData[wallID].sideNormal * (nonPenetrationForce) - slidingForce;
            }
            else
            {
                thisAgentSocialForce = -1 * dirTowardsWall * psychologicalForce;
            }

            wallForce += thisAgentSocialForce;
        }

        return wallForce.normalized;
    }

    private Vector3 CalculateActionSceneForce(bool evaderAgent)
    {
        Vector3 closestPosition = transform.position;
        float closestDist = -1.0f;
        Color drawColour = CHASER ? Color.blue : Color.yellow;

        foreach (GameObject otherAgent in perceivedNeighbors)
        {
            float curDistance = Vector3.Distance(transform.position, otherAgent.transform.position);
            if (closestDist == -1.0f || closestDist > curDistance)
            {
                closestDist = curDistance;
                closestPosition = otherAgent.transform.position;
            }
        }

        //Debug.DrawLine(closestPosition + Vector3.up, transform.position + Vector3.up, drawColour);

        Vector3 dirToOtherAgent = closestPosition - transform.position;
        float normalSize = Mathf.Abs(dirToOtherAgent.magnitude) / (perceptionRadius * 0.5f);
        normalSize = 1.0f - normalSize;

        dirToOtherAgent = dirToOtherAgent.normalized;
        dirToOtherAgent *= normalSize;

        float multiplier = evaderAgent ? -1.0f : 1.0f;

        return multiplier * dirToOtherAgent;
    }

    private Vector3 ShowFollowerTheRightWay(Vector3 curGoalForce)
    {
        Vector3 rightForce = curGoalForce;

        Vector3 leaderGoalForce = leaderAgent.goalForce;
        float mPerpLF = -1 / Mathf.Atan2(leaderGoalForce.z, leaderGoalForce.x);
        float val = goalForce.z - mPerpLF * goalForce.x;
        Debug.DrawRay(transform.position + Vector3.up, leaderGoalForce * 2.0f, Color.blue);
        Debug.DrawRay(transform.position + Vector3.up, goalForce * 2.0f, Color.green);
        if (Vector3.Dot(leaderGoalForce, goalForce) <= 0)
        {
            rightForce = curGoalForce * -1;
            rightForce += Vector3.Cross(Vector3.up, rightForce * -1);
            rightForce = rightForce.normalized;
            //rightForce += Vector3.Cross(Vector3.up, rightForce);
            Debug.DrawRay(transform.position, rightForce, Color.green);
        }

        //rightForce = rightForce.normalized;

        return rightForce;
    }

    private Vector3 CalculateEscapeTrappedForce(Vector3 curGoalForce)
    {
        Vector3 goalForce = curGoalForce;

        LayerMask checkLayer = CHASER ? evaderLayer : chaserLayer;

        if(Physics.Raycast(transform.position, curGoalForce, curGoalForce.magnitude, checkLayer))
        {
            goalForce = Vector3.Cross(Vector3.up, curGoalForce);
            if (Physics.Raycast(transform.position, goalForce, goalForce.magnitude, checkLayer))
            {
                goalForce *= -1;
                if (Physics.Raycast(transform.position, goalForce, goalForce.magnitude, checkLayer))
                {
                    goalForce = curGoalForce * -1;
                }
            }
        }

        return goalForce;
    }

    private Vector3 CalculateWallHuggingForce()
    {
        Vector3 wallForce = Vector3.zero;

        foreach (GameObject wall in perceivedWalls)
        {
            Vector3 thisAgentSocialForce = Vector3.zero;

            int wallID = wall.GetInstanceID();

            float distBtwObjects = Vector3.Distance(wall.transform.position, transform.position);

            float agentRadiusSum = nma.radius;
            //Debug.Log(wall.transform.localScale.magnitude);
            float dif = agentRadiusSum - distBtwObjects;

            float psychologicalForce = Mathf.Exp(dif) / 1000.0f;

            Vector3 dirTowardsWall = (wall.transform.position - transform.position).normalized;
            //Debug.DrawLine(transform.position, transform.position + dirTowardsWall, Color.green);
            //Debug.DrawLine(Vector3.zero, transform.position + dirTowardsWall, Color.green);

            if (agentWallCollisionData.ContainsKey(wallID))
            {
                dirTowardsWall = agentWallCollisionData[wallID].collisionPoint - transform.position;

                float nonPenetrationForce = Mathf.Abs(dif);

                Vector3 tangentialDir = Vector3.Cross(agentWallCollisionData[wallID].sideNormal, Vector3.up);
                if (Vector3.Dot(tangentialDir, GetVelocity()) < 0)
                {
                    tangentialDir *= -1;
                }

                float tangentialVelA = dif * Vector3.Dot(GetVelocity(), tangentialDir);

                Vector3 tangentialPoint = agentWallCollisionData[wallID].collisionPoint;

                Vector3 slidingForce = tangentialDir * (tangentialVelA);
                Debug.DrawRay(tangentialPoint, tangentialDir, Color.red);
                Debug.DrawRay(tangentialPoint, agentWallCollisionData[wallID].sideNormal, Color.cyan);

                thisAgentSocialForce = agentWallCollisionData[wallID].sideNormal * (psychologicalForce + nonPenetrationForce) - slidingForce;
            }
            else
            {
                thisAgentSocialForce = dirTowardsWall * psychologicalForce;
            }

            wallForce += thisAgentSocialForce;
        }

        return wallForce.normalized;
    }

    public void ApplyForce()
    {
        Vector3 force = ComputeForce();
        force.y = 0;

        rb.AddForce(force * 10, ForceMode.Force);
    }

    public void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag(agentTag))
        {
            if(EVADER || CHASER)
            {
                Agent otherAgentComp = other.GetComponent<Agent>();
                if (otherAgentComp.CHASER != CHASER)
                {
                    //Debug.Log("Got OTHER");
                    if (!perceivedNeighbors.Contains(other.gameObject))
                    {
                        perceivedNeighbors.Add(other.gameObject);
                    }
                }
            }
            else
            {
                if (!perceivedNeighbors.Contains(other.gameObject))
                {
                    perceivedNeighbors.Add(other.gameObject);
                }
            }

            //Debug.Log(other.gameObject == gameObject);
        }
        else if (other.CompareTag(wallTag))
        {
            if (!perceivedWalls.Contains(other.gameObject))
            {
                perceivedWalls.Add(other.gameObject);
            }
        }

    }

    public void OnTriggerExit(Collider other)
    {
        if (other.CompareTag(agentTag))
        {
            perceivedNeighbors.Remove(other.gameObject);
        }
        else if (other.CompareTag(wallTag))
        {
            perceivedWalls.Remove(other.gameObject);
        }
    }

    public void OnCollisionEnter(Collision collision)
    {
        if (collision.collider.CompareTag(wallTag))
        {
            if (perceivedWalls.Contains(collision.collider.gameObject))
            {
                WallCollisionData thisCollisionData = new WallCollisionData()
                {
                    collisionPoint = collision.GetContact(0).point,
                    sideNormal = collision.GetContact(0).normal
                };
                agentWallCollisionData.Add(collision.collider.gameObject.GetInstanceID(), thisCollisionData);
            }
        }
    }

    public void OnCollisionExit(Collision collision)
    {
        if (collision.collider.CompareTag(wallTag))
        {
            agentWallCollisionData.Remove(collision.collider.gameObject.GetInstanceID());
        }
    }

    #endregion
}
