using OWML.Common;
using OWML.ModHelper;
using System;
using UnityEngine;

namespace StrangerSpinGravity
{
    public class StrangerSpinGravity : ModBehaviour
    {

        
        public static float SpinSpeed = -0.33f;
        public static float GravFactor = 0.001f;
        public static bool DepartureComp = true;
        public static bool FreeRotate = false;
        public static OWRigidbody ringworld = null;

        private void Start()
        {
            ModHelper.Console.WriteLine($"Loaded StrangerSpinGravity");
            ModHelper.HarmonyHelper.AddPostfix<RingWorldForceVolume>("CalculateForceAccelerationAtPoint", typeof(StrangerSpinGravity), nameof(StrangerSpinGravity.PointForce));
            ModHelper.HarmonyHelper.AddPostfix<CylindricalForceVolume>("CalculateForceAccelerationAtPoint", typeof(StrangerSpinGravity), nameof(StrangerSpinGravity.PointForce));
            ModHelper.HarmonyHelper.AddPostfix<DirectionalForceVolume>("CalculateForceAccelerationAtPoint", typeof(StrangerSpinGravity), nameof(StrangerSpinGravity.PointForce));
            ModHelper.HarmonyHelper.AddPrefix<RingWorldController>("FixedUpdate", typeof(StrangerSpinGravity), nameof(StrangerSpinGravity.ControllerUpdate));
            ModHelper.HarmonyHelper.AddPrefix<RingRiverFluidVolume>("GetBuoyancy", typeof(StrangerSpinGravity), nameof(StrangerSpinGravity.InertialBuoyancy));
            ModHelper.HarmonyHelper.AddPrefix<RingWaveFluidVolume>("GetBuoyancy", typeof(StrangerSpinGravity), nameof(StrangerSpinGravity.InertialBuoyancyWave));
            ModHelper.HarmonyHelper.AddPrefix<ShipThrusterModel>("FixedUpdate", typeof(StrangerSpinGravity), nameof(StrangerSpinGravity.AngleUpdate));
            ModHelper.HarmonyHelper.AddPrefix<JetpackThrusterModel>("FixedUpdate", typeof(StrangerSpinGravity), nameof(StrangerSpinGravity.AngleUpdate));
        }

        // Set angular velocity drag for ship and player
        public static bool AngleUpdate(ThrusterModel __instance)
        {
            __instance.GetAttachedOWRigidbody().GetRigidbody().angularDrag = FreeRotate ? 0.0f : 0.96f;
            return true;
        }

        // Reduce force applied by ringworld grav fields
        public static void PointForce(ref Vector3 __result, ForceVolume __instance)
        {
            if (ringworld != null && __instance._attachedBody == ringworld)
            {
                __result *= GravFactor;
            }
        }

        // Add inertia to river buoyancy calculation
        public static bool InertialBuoyancy(ref Vector3 __result, RingRiverFluidVolume __instance, FluidDetector detector, float fractionSubmerged)
        {
            if (detector.GetAttachedOWRigidbody().GetAttachedForceDetector() != null)
            {
                Vector3 a = detector.GetAttachedOWRigidbody().GetAttachedForceDetector().GetForceAcceleration() - __instance._attachedBody.GetAttachedForceDetector().GetForceAcceleration();
                // Add centripetal acceleration relative to ring into buoyancy acceleration
                a -= __instance._attachedBody.GetPointCentripetalAcceleration(detector.GetAttachedOWRigidbody().GetWorldCenterOfMass())*0.9f;
                Vector3 a2 = Vector3.ProjectOnPlane(detector.transform.position - __instance.transform.position, __instance.transform.up);
                __result = Vector3.Project(-a, -a2) * fractionSubmerged * __instance._buoyancyDensity / detector.GetBuoyancyData().density;
            }
            else
            {
                __result = Vector3.zero;
            }
            return false;
        }

        // Add inertia to wave buoyancy calculation
        public static bool InertialBuoyancyWave(ref Vector3 __result, RingWaveFluidVolume __instance, FluidDetector detector, float fractionSubmerged)
        {
            if (detector.GetAttachedOWRigidbody().GetAttachedForceDetector() != null)
            {
                Vector3 onNormal = Vector3.ProjectOnPlane(__instance._riverFluid.transform.position - detector.transform.position, __instance._riverFluid.transform.up);
                Vector3 forcesum = detector.GetAttachedOWRigidbody().GetAttachedForceDetector().GetForceAcceleration() - __instance._attachedBody.GetAttachedForceDetector().GetForceAcceleration();
                // Add centripetal acceleration relative to ring into buoyancy acceleration
                forcesum -= __instance._attachedBody.GetPointCentripetalAcceleration(detector.GetAttachedOWRigidbody().GetWorldCenterOfMass()) * 0.9f;
                __result = Vector3.Project(-forcesum, onNormal) * fractionSubmerged * __instance._buoyancyDensity / detector.GetBuoyancyData().density;
            }
            else
            {
                __result = Vector3.zero;
            }
            return false;
        }

        // Slowly change ring spin speed towards target to avoid smashing
        // Set departure compensation factor
        public static bool ControllerUpdate(RingWorldController __instance)
        {
            ringworld = __instance.GetRingWorldBody();

            Vector3 target = __instance.GetComponent<InitialMotion>()._rotationAxis.normalized* SpinSpeed;
            Vector3 current = __instance.GetRingWorldBody().GetAngularVelocity();
            Vector3 change = target - current;
            Vector3 slowChange = change.normalized * 0.001f;
            if (change.magnitude > slowChange.magnitude)
            {
                change = slowChange;
            }

            __instance._interiorAccelFactor = DepartureComp ? 0.5f : 0.0f;
            __instance.GetRingWorldBody().SetAngularVelocity(current + change);
            return true;
        }
        public override void Configure(IModConfig config)
        {
            // This puts the stranger ground level at ~1.3g as usual.
            SpinSpeed = (float)(Math.Sqrt(config.GetSettingsValue<double>("Stranger Spin Gravity"))* -0.2046572d);
            // Gravity field should be very small, but must still exist to cause player alignment.
            GravFactor = Math.Max(0.001f, config.GetSettingsValue<float>("Stranger Faked Gravity") /1.25f);
            // Accelerate the player along with the stranger when departing.  Unnoticable, acceleration is ~0.02g
            DepartureComp = config.GetSettingsValue<bool>("Stranger Acceleration Compenstaion");
            // Remove angular drag to increase ability to make clean landings in hanger
            FreeRotate = !config.GetSettingsValue<bool>("Player Ship Angular Drag");

        }
    }
}