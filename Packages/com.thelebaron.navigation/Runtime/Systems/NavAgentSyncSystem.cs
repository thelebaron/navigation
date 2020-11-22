using UnityEngine;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Transforms;
using NavJob.Components;
using Unity.Mathematics;

// ReSharper disable UnusedType.Global

namespace NavJob.Systems
{
    [UpdateAfter(typeof(NavAgentSystem))]
    public class NavigationTransformSystem : SystemBase
    {
        protected override void OnUpdate()
        {
            
        }
    }
    
    
    /// <summary>
    /// Applies the damage to the health component. Todo: merge damage and apply in one go? so can be gibbed?
    /// </summary>
    [BurstCompile]
    public struct ChangeTransforms : IJobChunk
    {
        public ComponentTypeHandle<Translation>              TranslationTypeHandle;
        public ComponentTypeHandle<Rotation>                 RotationTypeHandle;
        public ComponentTypeHandle<LocalToWorld>             LocalToWorldTypeHandle;
        public ComponentTypeHandle<NavAgent>                 NavAgentTypeHandle;
        public ComponentTypeHandle<SyncPositionToNavAgent>   SyncPositionToNavAgentTypeHandle;
        public ComponentTypeHandle<SyncPositionFromNavAgent> SyncPositionFromNavAgentTypeHandle;
        public ComponentTypeHandle<SyncRotationToNavAgent>   SyncRotationToNavAgentTypeHandle;
        public ComponentTypeHandle<SyncRotationFromNavAgent> SyncRotationFromNavAgentTypeHandle;
        public uint                                          LastSystemVersion;
        
        public void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
        {
            var changed = chunk.DidOrderChange(LastSystemVersion) || chunk.DidChange(NavAgentTypeHandle, LastSystemVersion);
            if (!changed)
                return;
            
            
            var chunkAgents         = chunk.GetNativeArray(NavAgentTypeHandle);
            var chunkTranslations   = chunk.GetNativeArray(TranslationTypeHandle);
            var chunkRotations      = chunk.GetNativeArray(RotationTypeHandle);
            var chunkLocalToWorlds  = chunk.GetNativeArray(LocalToWorldTypeHandle);
            var hasTranslation      = chunk.Has(TranslationTypeHandle);
            var hasRotation         = chunk.Has(RotationTypeHandle);
            var hasLocalToWorld     = chunk.Has(LocalToWorldTypeHandle);
            var hasSyncPositionTo   = chunk.Has(SyncPositionToNavAgentTypeHandle);
            var hasSyncPositionFrom = chunk.Has(SyncPositionFromNavAgentTypeHandle);
            var hasSyncRotationTo   = chunk.Has(SyncRotationToNavAgentTypeHandle);
            var hasSyncRotationFrom = chunk.Has(SyncRotationFromNavAgentTypeHandle);
            var count               = chunk.Count;

            for (var i = 0; i < count; i++)
            {
                var localToWorld  = chunkLocalToWorlds[i];
                localToWorld.Value    = float4x4.TRS(chunkAgents[i].position, chunkAgents[i].rotation, Vector3.one);
                chunkLocalToWorlds[i] = localToWorld;
            }
            
        }
    }
    
    
    
    
    
    /// <summary>
    /// Syncs the transform matrix from the nav agent to a LocalToWorld component
    /// </summary>
    [UpdateAfter(typeof(NavAgentSystem))]
    //[DisableAutoCreation]
    public class NavAgentToTransfomMatrixSyncSystem : JobComponentSystem
    {

        [BurstCompile]
        [ExcludeComponent(typeof(Translation), typeof(Rotation))]
        private struct NavAgentToTransfomMatrixSyncSystemJob : IJobForEach<NavAgent, LocalToWorld>
        {
            public void Execute([ReadOnly] ref NavAgent agent, ref LocalToWorld localToWorld)
            {
                localToWorld.Value = Matrix4x4.TRS(agent.position, agent.rotation, Vector3.one);
            }
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            return new NavAgentToTransfomMatrixSyncSystemJob().Schedule(this, inputDeps);
        }
    }
    
    
    

    /// <summary>
    /// Sets the NavAgent position to the Position component
    /// </summary>
    [UpdateBefore(typeof(NavAgentSystem))]
    //[DisableAutoCreation]
    public class NavAgentFromPositionSyncSystem : JobComponentSystem
    {
        [BurstCompile]
        [RequireComponentTag(typeof(SyncPositionToNavAgent))]
        private struct NavAgentFromPositionSyncSystemJob : IJobForEach<NavAgent, Translation>
        {
            public void Execute(ref NavAgent NavAgent, [ReadOnly] ref Translation Position)
            {
                NavAgent.position = Position.Value;
            }
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            return new NavAgentFromPositionSyncSystemJob().Schedule(this, inputDeps);
        }
    }

    /// <summary>
    /// Sets the Position component to the NavAgent position
    /// </summary>
    [UpdateAfter(typeof(NavAgentSystem))]
    //[DisableAutoCreation]
    public class NavAgentToPositionSyncSystem : JobComponentSystem
    {
        [BurstCompile]
        [RequireComponentTag(typeof(SyncPositionFromNavAgent))]
        private struct NavAgentToPositionSyncSystemJob : IJobForEach<NavAgent, Translation>
        {
            public void Execute([ReadOnly] ref NavAgent NavAgent, ref Translation Position)
            {
                Position.Value = NavAgent.position;
            }
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            return new NavAgentToPositionSyncSystemJob().Schedule(this, inputDeps);
        }
    }

    /// <summary>
    /// Sets the NavAgent rotation to the Rotation component
    /// </summary>
    [UpdateBefore(typeof(NavAgentSystem))]
    //[DisableAutoCreation]
    public class NavAgentFromRotationSyncSystem : JobComponentSystem
    {
        [BurstCompile]
        [RequireComponentTag(typeof(SyncRotationToNavAgent))]
        private struct NavAgentFromRotationSyncSystemJob : IJobForEach<NavAgent, Rotation>
        {
            public void Execute(ref NavAgent NavAgent, [ReadOnly] ref Rotation Rotation)
            {
                NavAgent.rotation = Rotation.Value;
            }
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            return new NavAgentFromRotationSyncSystemJob().Schedule(this, inputDeps);
        }
    }

    /// <summary>
    /// Sets the Rotation component to the NavAgent rotation
    /// </summary>
    [UpdateAfter(typeof(NavAgentSystem))]
    //[DisableAutoCreation]
    public class NavAgentToRotationSyncSystem : JobComponentSystem
    {
        [BurstCompile]
        [RequireComponentTag(typeof(SyncRotationFromNavAgent))]
        private struct NavAgentToRotationSyncSystemJob : IJobForEach<NavAgent, Rotation>
        {
            public void Execute([ReadOnly] ref NavAgent NavAgent, ref Rotation Rotation)
            {
                Rotation.Value = NavAgent.rotation;
            }
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            return new NavAgentToRotationSyncSystemJob().Schedule(this, inputDeps);
        }
    }
}