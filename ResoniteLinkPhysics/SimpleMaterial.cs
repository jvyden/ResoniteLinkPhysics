using BepuPhysics.Constraints;

namespace ResoniteLinkPhysics;

public struct SimpleMaterial
{
    public SpringSettings SpringSettings;
    public float FrictionCoefficient;
    public float MaximumRecoveryVelocity;
}