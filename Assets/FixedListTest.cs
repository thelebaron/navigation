using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

public class FixedListTest : MonoBehaviour
{
    public int count = 100;
    private NativeList<FixedList4096<float3>> list = new NativeList<FixedList4096<float3>>(Allocator.Persistent);
    
    void Start()
    {
        
    }

    
    void Update()
    {
        for (int i = 0; i < count; i++)
        {
            list.Add(new FixedList4096<float3>());
        }
    }
}
