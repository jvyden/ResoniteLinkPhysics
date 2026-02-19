using System.Numerics;
using System.Runtime.CompilerServices;
using BepuPhysics;
using ResoniteLink;

namespace ResoniteLinkPhysics;

public class Ball
{
    public string SlotId;
    public BodyHandle BodyHandle;

    public DataModelOperation UpdatePosition(Simulation sim)
    {
        RigidPose pose = sim.Bodies[this.BodyHandle].Pose;

        Slot slot = new()
        {
            ID = this.SlotId,
            Position = new Field_float3 {Value = Unsafe.BitCast<Vector3, float3>(pose.Position)},
            Rotation = new Field_floatQ {Value = Unsafe.BitCast<Quaternion, floatQ>(pose.Orientation)}
        };

        return new UpdateSlot
        {
            Data = slot,
        };
    }
}