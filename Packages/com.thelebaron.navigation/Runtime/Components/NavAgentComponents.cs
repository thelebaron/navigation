using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

namespace NavJob.Components
{

    public enum AgentStatus
    {
        Idle = 0,
        PathQueued = 1,
        Moving = 2,
        Paused = 4
    }

    [System.Serializable]
    public struct NavAgent : IComponentData
    {
        public float       stoppingDistance;
        public float       moveSpeed;
        public float       acceleration;
        public float       rotationSpeed;
        public int         areaMask;
        public float3      destination;
        public float       currentMoveSpeed;
        public int         queryVersion;
        public AgentStatus status;
        public float3      position;
        public float3      nextPosition;
        public quaternion  rotation;
        public float       remainingDistance;
        public float3      currentWaypoint;
        public int         nextWaypointIndex;
        public int         totalWaypoints;

        public NavAgent (
            float3 position,
            quaternion rotation,
            float stoppingDistance = 1f,
            float moveSpeed = 4f,
            float acceleration = 1f,
            float rotationSpeed = 10f,
            int areaMask = -1)
        {
            this.stoppingDistance = stoppingDistance;
            this.moveSpeed = moveSpeed;
            this.acceleration = acceleration;
            this.rotationSpeed = rotationSpeed;
            this.areaMask = areaMask;
            destination = float3.zero;
            currentMoveSpeed = 0;
            queryVersion = 0;
            status = AgentStatus.Idle;
            this.position = position;
            this.rotation = rotation;
            nextPosition = new float3 (Mathf.Infinity, Mathf.Infinity, Mathf.Infinity);
            remainingDistance = 0;
            currentWaypoint = Vector3.zero;
            nextWaypointIndex = 0;
            totalWaypoints = 0;
        }
    }

    [System.Serializable]
    public struct NavAgentAvoidance : IComponentData
    {
        public float  radius;
        public float3 partition;

        public NavAgentAvoidance (float radius = 1f)
        {
            this.radius    = radius;
            this.partition = new float3 (0);
        }
    }
    
    public struct SyncPositionFromNavAgent : IComponentData { }
    public struct SyncPositionToNavAgent : IComponentData { }
    public struct SyncRotationFromNavAgent : IComponentData { }
    public struct SyncRotationToNavAgent : IComponentData { }
}