#region

using System.Collections.Concurrent;
using System.Collections.Generic;
using UnityEngine;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using NavJob.Components;
using thelebaron.mathematics;

#endregion

namespace NavJob.Systems
{

    class SetDestinationBarrier : EntityCommandBufferSystem { }
    class PathSuccessBarrier : EntityCommandBufferSystem { }
    class PathErrorBarrier : EntityCommandBufferSystem { }

    public class NavAgentSystem : SystemBase
    {

        private struct AgentData
        {
            public int index;
            public Entity entity;
            public NavAgent agent;
        }

        private          NativeQueue<AgentData>                           _needsWaypoint;
        private readonly ConcurrentDictionary<int, FixedList4096<float3>> _waypoints = new ConcurrentDictionary<int, FixedList4096<float3>>();
        private          NativeHashMap<int, AgentData>                    _pathFindingData;

        [BurstCompile]
        private struct DetectNextWaypointJob : IJobParallelFor
        {
            public int navMeshQuerySystemVersion;

            [ReadOnly]
            public NativeArray<Entity> Entities;
            [NativeDisableParallelForRestriction]
            public ComponentDataFromEntity<NavAgent> Agents;
            public NativeQueue<AgentData>.ParallelWriter NeedsWaypoint;

            public void Execute(int index)
            {
                var entity = Entities[index];
                var agent = Agents[entity];
                if (agent.remainingDistance - agent.stoppingDistance > 0 || agent.status != AgentStatus.Moving)
                {
                    return;
                }

                if (agent.nextWaypointIndex != agent.totalWaypoints)
                {
                    NeedsWaypoint.Enqueue(new AgentData { agent = agent, entity = entity, index = index });
                }
                else if (navMeshQuerySystemVersion != agent.queryVersion || agent.nextWaypointIndex == agent.totalWaypoints)
                {
                    agent.totalWaypoints = 0;
                    agent.currentWaypoint = 0;
                    agent.status = AgentStatus.Idle;
                    Agents[entity] = agent;
                }
            }
        }

        private struct SetNextWaypointJob : IJob
        {
            public ComponentDataFromEntity<NavAgent> Agents;
            public NativeQueue<AgentData> NeedsWaypoint;
            public void Execute()
            {
                // TODO: Don't like how this one converted...
                while (NeedsWaypoint.TryDequeue(out AgentData item))
                {
                    var entity = item.entity;
                    if (NavAgentSystem.Instance._waypoints.TryGetValue(entity.Index, out FixedList4096<float3> currentWaypoints))
                    {
                        var agent = item.agent;
                        agent.currentWaypoint = currentWaypoints[agent.nextWaypointIndex];
                        agent.remainingDistance = math.distance(agent.position, agent.currentWaypoint);
                        agent.nextWaypointIndex++;
                        Agents[entity] = agent;
                    }
                }
            }
        }

        [BurstCompile]
        private struct MovementJob : IJobParallelFor
        {
            public float DeltaTime;
            [ReadOnly]
            [DeallocateOnJobCompletion]
            public NativeArray<Entity> Entities;
            [NativeDisableParallelForRestriction]
            public ComponentDataFromEntity<NavAgent> Agents;

            public void Execute(int index)
            {
                var entity = Entities[index];

                var agent = Agents[entity];
                if (agent.status != AgentStatus.Moving)
                {
                    return;
                }

                if (agent.remainingDistance > 0)
                {
                    agent.currentMoveSpeed = math.lerp(agent.currentMoveSpeed, agent.moveSpeed, DeltaTime * agent.acceleration);
                    // todo: deceleration
                    if (agent.nextPosition.x != math.INFINITY)
                    {
                        agent.position = agent.nextPosition;
                    }
                    var heading = (agent.currentWaypoint - agent.position);
                    agent.remainingDistance = math.length(heading);
                    if (agent.remainingDistance > 0.001f)
                    {
                        //var targetRotation                  = Quaternion.LookRotation(heading, maths.up).eulerAngles;
                        var targetRotation                  = maths.eulerXYZ(quaternion.LookRotationSafe(heading, maths.up));
                        targetRotation.x = targetRotation.z = 0;
                        if (agent.remainingDistance < 1)
                        {
                            agent.rotation = quaternion.Euler(targetRotation, math.RotationOrder.XYZ);
                        }
                        else
                        {
                            var rot = quaternion.Euler(targetRotation, math.RotationOrder.XYZ);
                            //rot = maths.nanSafeQuaternion(rot);
                            //rot = math.normalizesafe(rot);
                            //agent.rotation = math.slerp(agent.rotation, rot, DeltaTime * agent.rotationSpeed); //math.slerp fails: Quaternion To Matrix conversion failed because input Quaternion is invalid
                            agent.rotation = Quaternion.Slerp(agent.rotation, rot, DeltaTime * agent.rotationSpeed);
                        }
                    }
                    var forward = math.forward(agent.rotation) * agent.currentMoveSpeed * DeltaTime;
                    agent.nextPosition = agent.position + forward;
                    Agents[entity] = agent;
                }
                else if (agent.nextWaypointIndex == agent.totalWaypoints)
                {
                    agent.nextPosition = new float3 { x = math.INFINITY, y = math.INFINITY, z = math.INFINITY };
                    agent.status       = AgentStatus.Idle;
                    Agents[entity]     = agent;
                }
            }
        }

        private EntityQuery _agentQuery;
        private NavMeshQuerySystem _querySystem;
        private SetDestinationBarrier _setDestinationBarrier;
        private PathSuccessBarrier _pathSuccessBarrier;
        private PathErrorBarrier _pathErrorBarrier;

        protected override void OnUpdate()
        {
            var entityCnt = _agentQuery.CalculateEntityCount();
            var entities = _agentQuery.ToEntityArray(Allocator.TempJob);

            var dt = Time.DeltaTime;
            Dependency = new DetectNextWaypointJob
            {
                Entities = entities,
                Agents = GetComponentDataFromEntity<NavAgent>(),
                NeedsWaypoint = _needsWaypoint.AsParallelWriter(),
                navMeshQuerySystemVersion = _querySystem.Version
            }.Schedule(entityCnt, 64, Dependency);

            Dependency = new SetNextWaypointJob
            {
                Agents = GetComponentDataFromEntity<NavAgent>(),
                NeedsWaypoint = _needsWaypoint
            }.Schedule(Dependency);

            Dependency = new MovementJob
            {
                DeltaTime = dt,
                Entities = entities,
                Agents = GetComponentDataFromEntity<NavAgent>()
            }.Schedule(entityCnt, 64, Dependency);
        }

        /// <summary>
        /// Used to set an agent destination and start the pathfinding process
        /// </summary>
        /// <param name="entity"></param>
        /// <param name="agent"></param>
        /// <param name="destination"></param>
        /// <param name="areas"></param>
        public void SetDestination(Entity entity, NavAgent agent, float3 destination, int areas = -1)
        {
            if (_pathFindingData.TryAdd(entity.Index, new AgentData { index = entity.Index, entity = entity, agent = agent }))
            {
                var command = _setDestinationBarrier.CreateCommandBuffer();
                agent.status = AgentStatus.PathQueued;
                agent.destination = destination;
                agent.queryVersion = _querySystem.Version;
                command.SetComponent<NavAgent>(entity, agent);
                _querySystem.RequestPath(entity.Index, agent.position, agent.destination, areas);
            }
        }

        /// <summary>
        /// Static counterpart of SetDestination
        /// </summary>
        /// <param name="entity"></param>
        /// <param name="agent"></param>
        /// <param name="destination"></param>
        public static void SetDestinationStatic(Entity entity, NavAgent agent, float3 destination, int areas = -1)
        {
            Instance.SetDestination(entity, agent, destination, areas);
        }

        protected static NavAgentSystem Instance;

        protected override void OnCreate()
        {
            Instance = this;

            _querySystem = World.GetOrCreateSystem<NavMeshQuerySystem>();
            _setDestinationBarrier = World.GetOrCreateSystem<SetDestinationBarrier>();
            _pathSuccessBarrier = World.GetOrCreateSystem<PathSuccessBarrier>();
            _pathErrorBarrier = World.GetOrCreateSystem<PathErrorBarrier>();

            _querySystem.RegisterPathResolvedCallback(OnPathSuccess);
            _querySystem.RegisterPathFailedCallback(OnPathError);

            var agentQueryDesc = new EntityQueryDesc
            {
                All = new ComponentType[] { typeof(NavAgent) }
            };
            _agentQuery = GetEntityQuery(agentQueryDesc);

            _needsWaypoint = new NativeQueue<AgentData>(Allocator.Persistent);
            _pathFindingData = new NativeHashMap<int, AgentData>(0, Allocator.Persistent);
        }

        protected override void OnDestroy()
        {
            _needsWaypoint.Dispose();
            _pathFindingData.Dispose();
        }

        private void SetWaypoint(Entity entity, NavAgent agent, FixedList4096<float3> newWaypoints)
        {
            _waypoints[entity.Index] = newWaypoints;
            var command = _pathSuccessBarrier.CreateCommandBuffer();
            agent.status = AgentStatus.Moving;
            agent.nextWaypointIndex = 1;
            agent.totalWaypoints = newWaypoints.Length;
            agent.currentWaypoint = newWaypoints[0];
            agent.remainingDistance = math.distance(agent.position, agent.currentWaypoint);
            command.SetComponent(entity, agent);
        }

        // make public for refactoring
        public void OnPathSuccess(int index, FixedList4096<float3> waypoints)
        {
            if (_pathFindingData.TryGetValue(index, out AgentData entry))
            {
                SetWaypoint(entry.entity, entry.agent, waypoints);
                _pathFindingData.Remove(index);
            }
        }

        private void OnPathError(int index, PathfindingFailedReason reason)
        {
            if (_pathFindingData.TryGetValue(index, out AgentData entry))
            {
                entry.agent.status = AgentStatus.Idle;
                var command = _pathErrorBarrier.CreateCommandBuffer();
                command.SetComponent<NavAgent>(entry.entity, entry.agent);
                _pathFindingData.Remove(index);
            }
        }
    }
}