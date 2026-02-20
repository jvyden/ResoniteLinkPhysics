using System.Numerics;
using System.Runtime.CompilerServices;
using BepuPhysics;
using ResoniteLink;

namespace ResoniteLinkPhysics;

public class Ball
{
    public string SlotId;
    public BodyHandle BodyHandle;
    
    public Quaternion LastOrientation;
    public Vector3 LastPosition;

    public DataModelOperation? UpdatePosition(Simulation sim)
    {
        BodyReference body = sim.Bodies[this.BodyHandle];

        if (!body.Awake)
            return null;

        RigidPose pose = body.Pose;

        if (pose.Orientation == LastOrientation && pose.Position == LastPosition)
            return null;

        this.LastOrientation = pose.Orientation;
        this.LastPosition = pose.Position;

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

    public void Jump(Simulation sim, float strength)
    {
        BodyReference body = sim.Bodies[this.BodyHandle];
        Vector3 impulse = new()
        {
            X = Util.Rand(-strength, strength),
            Y = Util.Rand(-strength, strength),
            Z = Util.Rand(-strength, strength),
        };
        body.ApplyLinearImpulse(impulse);
    }
}