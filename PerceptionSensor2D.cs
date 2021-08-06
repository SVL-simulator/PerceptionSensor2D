/**
 * Copyright (c) 2019-2021 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */

namespace Simulator.Sensors
{
    using System;
    using System.Collections.Generic;
    using UnityEngine;
    using Bridge;
    using Bridge.Data;
    using Utilities;
    using UI;
    using Components;
    using UnityEngine.Experimental.Rendering;
    using UnityEngine.Rendering;
    using UnityEngine.Rendering.HighDefinition;

    [SensorType("2D Perception", new[] {typeof(Detected2DObjectData)})]
    public class PerceptionSensor2D : FrequencySensorBase
    {
        private static class Properties
        {
            public static readonly int TexSize = Shader.PropertyToID("_TexSize");
            public static readonly int Input = Shader.PropertyToID("_Input");
            public static readonly int MinX = Shader.PropertyToID("_MinX");
            public static readonly int MaxX = Shader.PropertyToID("_MaxX");
            public static readonly int MinY = Shader.PropertyToID("_MinY");
            public static readonly int MaxY = Shader.PropertyToID("_MaxY");
        }
        
        [SensorParameter]
        [Range(1, 1920)]
        public int Width = 1920;

        [SensorParameter]
        [Range(1, 1080)]
        public int Height = 1080;

        [SensorParameter]
        [Range(1.0f, 90.0f)]
        public float FieldOfView = 60.0f;

        [SensorParameter]
        [Range(0.01f, 1000.0f)]
        public float MinDistance = 0.1f;

        [SensorParameter]
        [Range(0.01f, 2000.0f)]
        public float MaxDistance = 2000.0f;

        [SensorParameter]
        [Range(0.01f, 2000f)]
        public float DetectionRange = 100f;

        [Range(0, 255)]
        public int testId = 32;

        private uint seqId;

        private SensorRenderTarget renderTarget;
        private RenderTexture activeRT;

        private List<Detected2DObject> DetectedObjects = new List<Detected2DObject>();

        AAWireBox AAWireBoxes;

        private BridgeInstance Bridge;
        private Publisher<Detected2DObjectData> Publish;

        public Camera Camera;
        public Camera SegmentationCamera;

        public ComputeShader cs;

        private ComputeBuffer minX;
        private ComputeBuffer maxX;
        private ComputeBuffer minY;
        private ComputeBuffer maxY;

        private uint[] minXarr = new uint[256];
        private uint[] maxXarr = new uint[256];
        private uint[] minYarr = new uint[256];
        private uint[] maxYarr = new uint[256];

        [AnalysisMeasurement(MeasurementType.Count)]
        public int MaxTracked = -1;

        public override SensorDistributionType DistributionType => SensorDistributionType.MainOrClient;
        public override float PerformanceLoad => 0.2f;

        private ShaderTagId passId;

        protected override bool UseFixedUpdate => false;

        private IAgentController Controller;

        protected override void Initialize()
        {
            Controller = GetComponentInParent<IAgentController>();
            
            activeRT = new RenderTexture(Width, Height, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.Linear)
            {
                dimension = TextureDimension.Tex2D,
                antiAliasing = 1,
                useMipMap = false,
                useDynamicScale = false,
                wrapMode = TextureWrapMode.Clamp,
                filterMode = FilterMode.Bilinear
            };

            activeRT.Create();

            Camera = GetComponentInChildren<Camera>();
            Camera.targetTexture = activeRT;
            Camera.fieldOfView = FieldOfView;
            Camera.nearClipPlane = MinDistance;
            Camera.farClipPlane = MaxDistance;
            Camera.gameObject.GetComponent<HDAdditionalCameraData>().hasPersistentHistory = true;

            passId = new ShaderTagId("SimulatorSegmentationPass");
            SegmentationCamera.fieldOfView = FieldOfView;
            SegmentationCamera.nearClipPlane = MinDistance;
            SegmentationCamera.farClipPlane = DetectionRange;
            SegmentationCamera.gameObject.GetComponent<HDAdditionalCameraData>().customRender += OnSegmentationRender;
            SegmentationCamera.gameObject.GetComponent<HDAdditionalCameraData>().hasPersistentHistory = true;
            renderTarget = SensorRenderTarget.Create2D(Width, Height, GraphicsFormat.R8G8B8A8_UNorm);
            SegmentationCamera.targetTexture = renderTarget;

            minX = new ComputeBuffer(256, sizeof(uint));
            maxX = new ComputeBuffer(256, sizeof(uint));
            minY = new ComputeBuffer(256, sizeof(uint));
            maxY = new ComputeBuffer(256, sizeof(uint));

            AAWireBoxes = gameObject.AddComponent<AAWireBox>();
            AAWireBoxes.Camera = Camera;
            AAWireBoxes.IgnoreUpdates = true;
        }

        protected override void Deinitialize()
        {
            if (activeRT != null)
                activeRT.Release();

            renderTarget?.Release();
            minX?.Release();
            maxX?.Release();
            minY?.Release();
            maxY?.Release();
        }

        public override void OnBridgeSetup(BridgeInstance bridge)
        {
            Bridge = bridge;
            Publish = Bridge.AddPublisher<Detected2DObjectData>(Topic);
        }

        private void OnSegmentationRender(ScriptableRenderContext context, HDCamera hdCamera)
        {
            var cmd = CommandBufferPool.Get();
            SensorPassRenderer.Render(context, cmd, hdCamera, renderTarget, passId, SimulatorManager.Instance.SkySegmentationColor);

            var clearKernel = cs.FindKernel("Clear");
            cmd.SetComputeVectorParam(cs, Properties.TexSize, new Vector4(Width, Height, 1f / Width, 1f / Height));
            cmd.SetComputeBufferParam(cs, clearKernel, Properties.MinX, minX);
            cmd.SetComputeBufferParam(cs, clearKernel, Properties.MaxX, maxX);
            cmd.SetComputeBufferParam(cs, clearKernel, Properties.MinY, minY);
            cmd.SetComputeBufferParam(cs, clearKernel, Properties.MaxY, maxY);
            cmd.DispatchCompute(cs, clearKernel, 4, 1, 1);

            var detectKernel = cs.FindKernel("Detect");
            cmd.SetComputeTextureParam(cs, detectKernel, Properties.Input, renderTarget.ColorHandle);
            cmd.SetComputeVectorParam(cs, Properties.TexSize, new Vector4(Width, Height, 1f / Width, 1f / Height));
            cmd.SetComputeBufferParam(cs, detectKernel, Properties.MinX, minX);
            cmd.SetComputeBufferParam(cs, detectKernel, Properties.MaxX, maxX);
            cmd.SetComputeBufferParam(cs, detectKernel, Properties.MinY, minY);
            cmd.SetComputeBufferParam(cs, detectKernel, Properties.MaxY, maxY);
            cmd.DispatchCompute(cs, detectKernel, HDRPUtilities.GetGroupSize(Width, 8), HDRPUtilities.GetGroupSize(Height, 8), 1);

            context.ExecuteCommandBuffer(cmd);
            cmd.Clear();
            CommandBufferPool.Release(cmd);
        }

        protected override void SensorUpdate()
        {
            Camera.Render();
            SegmentationCamera.Render();

            minX.GetData(minXarr);
            maxX.GetData(maxXarr);
            minY.GetData(minYarr);
            maxY.GetData(maxYarr);

            DetectedObjects.Clear();

            // 0 is reserved for clear color, 255 for non-agent segmentation
            for (var i = 1; i < 255; ++i)
            {
                if (minXarr[i] == Width - 1 || maxXarr[i] == 0)
                    continue;

                var detected = GetDetectedObject(i);
                if (detected != null)
                    DetectedObjects.Add(detected);
            }

            MaxTracked = Math.Max(MaxTracked, DetectedObjects.Count);
            if (Bridge != null && Bridge.Status == Status.Connected)
            {
                Publish(new Detected2DObjectData()
                {
                    Frame = Frame,
                    Sequence = seqId++,
                    Time = SimulatorManager.Instance.CurrentTime,
                    Data = DetectedObjects.ToArray(),
                });
            }

            foreach (var obj in DetectedObjects)
            {
                var min = obj.Position - obj.Scale / 2;
                var max = obj.Position + obj.Scale / 2;

                var color = string.Equals(obj.Label, "Pedestrian") ? Color.yellow : Color.green;
                AAWireBoxes.DrawBox(min, max, color);
            }

            var cmd = CommandBufferPool.Get();
            Camera.Render();
            AAWireBoxes.Draw(cmd);
            AAWireBoxes.Clear();
            HDRPUtilities.ExecuteAndClearCommandBuffer(cmd);
            CommandBufferPool.Release(cmd);
        }

        private Detected2DObject GetDetectedObject(int segId)
        {
            if (!SimulatorManager.Instance.SegmentationIdMapping.TryGetEntityGameObject(segId, out var go, out var type))
            {
                Debug.LogError($"Entity with ID {segId} is not registered.");
                return null;
            }

            uint id;
            string label;
            float linear_vel;
            float angular_vel;

            switch (type)
            {
                case SegmentationIdMapping.SegmentationEntityType.Agent:
                {
                    var controller = go.GetComponent<IAgentController>();
                    var dynamics = go.GetComponent<IVehicleDynamics>();
                    id = controller.GTID;
                    label = "Sedan";
                    // TODO: verify forward vector
                    linear_vel = Vector3.Dot(dynamics.Velocity, go.transform.forward);
                    angular_vel = -dynamics.AngularVelocity.y;
                }
                    break;
                case SegmentationIdMapping.SegmentationEntityType.NPC:
                {
                    var npcC = go.GetComponent<NPCController>();
                    id = npcC.GTID;
                    label = npcC.NPCLabel;
                    linear_vel = Vector3.Dot(npcC.GetVelocity(), go.transform.forward);
                    angular_vel = -npcC.GetAngularVelocity().y;
                }
                    break;
                case SegmentationIdMapping.SegmentationEntityType.Pedestrian:
                {
                    var pedC = go.GetComponent<PedestrianController>();
                    id = pedC.GTID;
                    label = "Pedestrian";
                    linear_vel = Vector3.Dot(pedC.CurrentVelocity, go.transform.forward);
                    angular_vel = -pedC.CurrentAngularVelocity.y;
                }
                    break;
                default:
                {
                    Debug.LogError($"Invalid entity type: {type.ToString()}");
                    return null;
                }
            }

            if (id == Controller.GTID)
                return null;

            var posPix = new Vector2(((float) minXarr[segId] + maxXarr[segId]) / 2, ((float) minYarr[segId] + maxYarr[segId]) / 2);
            posPix.y = Height - posPix.y;
            var sizePix = new Vector2(maxXarr[segId] - minXarr[segId], maxYarr[segId] - minYarr[segId]);

            return new Detected2DObject()
            {
                Id = id,
                Label = label,
                Score = 1.0f,
                Position = new Vector2(posPix.x, posPix.y),
                Scale = new Vector2(sizePix.x, sizePix.y),
                LinearVelocity = new Vector3(linear_vel, 0, 0),
                AngularVelocity = new Vector3(0, 0, angular_vel),
            };
        }

        public override void OnVisualize(Visualizer visualizer)
        {
            visualizer.UpdateRenderTexture(activeRT, Camera.aspect);
        }

        public override void OnVisualizeToggle(bool state)
        {
            //
        }
    }
}