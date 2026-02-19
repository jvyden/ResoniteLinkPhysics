using System.Collections.Concurrent;
using System.Diagnostics;
using System.Drawing;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;
using ResoniteLink;
using static ResoniteLinkPhysics.Util;

namespace ResoniteLinkPhysics;
public static class Program
{
    private static bool _running = true;
    private static CollidableProperty<SimpleMaterial> _property;
    private static Simulation _sim;
    private static LinkInterface _link;
    
    // The prefix prevents multiple REPL sessions from colliding with each other's ID's
    private static string _prefix;
    private static int _idPool;
    private static string AllocateId() => $"{nameof(ResoniteLinkPhysics)}_{_prefix}_{_idPool++:X}";

    private static readonly ConcurrentQueue<Ball> _balls = [];
    private static readonly List<Material> _materials = [];
    
    public static async Task Main()
    {
        Console.TreatControlCAsInput = false;
        Console.CancelKeyPress += (_, eventArgs) =>
        {
            _running = false;
            eventArgs.Cancel = true;
        };
        
        // Console.Write("Port: ");
        // int port = int.Parse(Console.ReadLine()!.TrimEnd());
        const int port = 23608;
        
        using LinkInterface link = new();
        await link.Connect(new Uri($"ws://localhost:{port}"), CancellationToken.None);
        _link = link;

        SessionData? session = await link.GetSessionData();
        if (session == null)
            throw new Exception("Session not found");

        _prefix = session.UniqueSessionId;
        
        Console.WriteLine("Connected!");
        
        using CollidableProperty<SimpleMaterial> collidableMaterials = new();
        _property = collidableMaterials;
        
        using BufferPool bufferPool = new();
        using Simulation simulation = Simulation.Create(bufferPool, new NarrowPhaseCallbacks {CollidableMaterials = collidableMaterials}, new PoseIntegratorCallbacks(new Vector3(0, -10, 0), 0.3f, 0.5f), new SolveDescription(8, 1));
        _sim = simulation;

        using ThreadDispatcher dispatcher = new(Environment.ProcessorCount);

        List<DataModelOperation> initOps = [];
        
        const int totalMaterials = 4;
        for (int i = 0; i < totalMaterials; i++)
        {
            initOps.AddRange(AddMaterial(false));
        }
        
        const int totalBalls = 10000;
        for (int i = 0; i < totalBalls; i++)
        {
            const float range = 15f;
            const float maxSize = 0.6f;
            const float heightOffset = 30f;
            float radius = Rand(0.3f, maxSize);
            initOps.AddRange(AddBall(new Vector3(Rand(-range, range), Rand(maxSize + heightOffset, range + heightOffset), Rand(-range, range)), radius));
        }

        await DiscoverCollidersAsync();

        try
        {
            Console.WriteLine("Submitting init batch...");

            const int batchSize = 50;
            List<DataModelOperation> batchedOps = new(batchSize);
            int i = 0;
            while (i < initOps.Count)
            {
                int j = 0;
                while (j < batchSize && i < initOps.Count)
                {
                    batchedOps.Add(initOps[i]);
                    i++;
                    j++;
                }
                
                BatchResponse? responses = await _link.RunDataModelOperationBatch(batchedOps);
                foreach (Response response in responses.Responses)
                {
                    if (!response.Success)
                        throw new Exception(response.ErrorInfo);
                }
                batchedOps.Clear();
            }

            const float targetTickrate = 60.0f;
            
            Console.WriteLine("Simulation running...");
            List<DataModelOperation> ops = [];

            Stopwatch sw = Stopwatch.StartNew();
            long lastTick = sw.ElapsedMilliseconds;
            while (_running)
            {
                long now = sw.ElapsedMilliseconds;
                float elapsed = Math.Max(0.01f, (now - lastTick) / 1000f);
                lastTick = now;
                ops.Clear();
                simulation.Timestep(elapsed, dispatcher);
                foreach (Ball ball in _balls)
                {
                    DataModelOperation? op = ball.UpdatePosition(simulation);
                    if(op != null)
                        ops.Add(op);
                }

                await _link.RunDataModelOperationBatch(ops);
                
                Thread.Sleep((int)((1000.0f / targetTickrate) - elapsed));
            }
        }
        finally
        {
            Console.WriteLine("Exiting");
            await RemoveAllSlots();
        }
    }

    private static IEnumerable<DataModelOperation> AddMaterial(bool pbs)
    {
        string slotId = AllocateId();
        string materialId = AllocateId();
        yield return new AddSlot
        {
            Data = new Slot
            {
                ID = slotId,
                Name = new Field_string {Value = "Shared Material"}
            }
        };
        
        Color color = ColorFromHSV(Random.Shared.NextSingle() * 255, 1, 1);
        if (pbs)
        {
            yield return new AddComponent
            {
                ContainerSlotId = slotId,
                Data = new Component
                {
                    ID = materialId,
                    ComponentType = "[FrooxEngine]FrooxEngine.PBS_Metallic",
                    Members = new Dictionary<string, Member>
                    {
                        {"AlbedoColor", new Field_colorX {Value = new colorX
                            {
                                r = color.R / 255.0f,
                                g = color.G / 255.0f,
                                b = color.B / 255.0f,
                                a = 1.0f,
                                Profile = "sRGB",
                            } }
                        },
                        {"Metallic", new Field_float {Value = Random.Shared.NextSingle()}},
                        {"Smoothness", new Field_float {Value = Random.Shared.NextSingle()}}
                    }
                }
            };
        }
        else
        {
            yield return new AddComponent
            {
                ContainerSlotId = slotId,
                Data = new Component
                {
                    ID = materialId,
                    ComponentType = "[FrooxEngine]FrooxEngine.UnlitMaterial",
                    Members = new Dictionary<string, Member>
                    {
                        {"TintColor", new Field_colorX {Value = new colorX
                            {
                                r = color.R / 255.0f,
                                g = color.G / 255.0f,
                                b = color.B / 255.0f,
                                a = 1.0f,
                                Profile = "sRGB",
                            } }
                        },
                    }
                }
            };
        }
        
        _materials.Add(new Material
        {
            SlotId = slotId,
            MaterialId = materialId,
        });
    }

    private static async Task RemoveAllSlots()
    {
        List<DataModelOperation> ops = [];
        
        foreach (Material material in _materials)
        {
            Debug.Assert(material != null);
            ops.Add(new RemoveSlot
            {
                SlotID = material.SlotId
            });
        }
        
        while (_balls.TryDequeue(out Ball? ball))
        {
            Debug.Assert(ball != null);
            ops.Add(new RemoveSlot
            {
                SlotID = ball.SlotId
            });
        }

        await _link.RunDataModelOperationBatch(ops);
    }

    public static IEnumerable<DataModelOperation> AddBall(Vector3 position, float radius = 1.0f)
    {
        Sphere sphere = new(radius);
        float mass = radius * 0.5f;
        BodyInertia inertia = sphere.ComputeInertia(mass);

        TypedIndex shape = _sim.Shapes.Add(sphere);
        BodyDescription desc = BodyDescription.CreateDynamic(position, inertia, shape, new BodyActivityDescription(0.01f));
        BodyHandle bodyHandle = _sim.Bodies.Add(desc);

        const int randRange = 200;
        float rand = Random.Shared.Next(0, randRange);
        _property.Allocate(bodyHandle) = new SimpleMaterial
        {
            FrictionCoefficient = 1,
            MaximumRecoveryVelocity = float.MaxValue,
            SpringSettings = new SpringSettings(5 + 0.25f * Random.Shared.Next(0, randRange), rand * rand / 10000f), 
        };

        string id = AllocateId();
        yield return new AddSlot
        {
            Data = new Slot
            {
                ID = id,
                Name = new Field_string
                {
                    Value = "Ball",
                },
                Position = new Field_float3
                {
                    Value = Unsafe.BitCast<Vector3, float3>(position),
                },
            }
        };

        string? materialId = null;
        if (_materials.Count > 0)
            materialId = _materials[Random.Shared.Next(0, _materials.Count)].MaterialId;

        string meshId = AllocateId();
        yield return new AddComponent
        {
            ContainerSlotId = id,
            Data = new Component
            {
                ID = meshId,
                ComponentType = "[FrooxEngine]FrooxEngine.SphereMesh",
                Members = new Dictionary<string, Member>
                {
                    {"Radius", new Field_float {Value = radius}},
                    {"Segments", new Field_int {Value = 12}},
                    {"Rings", new Field_int {Value = 6}},
                }
            }
        };
        
        yield return new AddComponent
        {
            ContainerSlotId = id,
            Data = new Component
            {
                ID = AllocateId(),
                ComponentType = "[FrooxEngine]FrooxEngine.SphereCollider",
                Members = new Dictionary<string, Member>
                {
                    {"Radius", new Field_float {Value = radius}},
                    {"Mass", new Field_float {Value = mass}},
                    {"CharacterCollider", new Field_bool {Value = true}},
                    {"Type", new Field_Enum {Value = "Active"}}
                }
            }
        };

        yield return new AddComponent
        {
            ContainerSlotId = id,
            Data = new Component
            {
                ID = AllocateId(),
                ComponentType = "[FrooxEngine]FrooxEngine.MeshRenderer",
                Members = new Dictionary<string, Member>
                {
                    {"Mesh", new Reference {TargetID = meshId}},
                    {"Materials",
                        new SyncList
                        {
                            Elements =
                            [
                                new Reference { TargetID = materialId }
                            ]
                        }
                    }
                }
            }
        };
        
        _balls.Enqueue(new Ball
        {
            SlotId = id,
            BodyHandle = bodyHandle,
        });
    }

    private static void AddBox(Vector3 position, Vector3 size, Quaternion rotation)
    {
        Box box = new(size.X, size.Y, size.Z);

        TypedIndex shape = _sim.Shapes.Add(box);
        StaticDescription desc = new(new RigidPose(position, rotation), shape);
        _sim.Statics.Add(desc);
    }
    
    private static async Task DiscoverCollidersAsync()
    {
        SlotData? data = await _link.GetSlotData(new GetSlot
        {
            SlotID = "Root",
            Depth = 2,
            IncludeComponentData = true,
        });

        if (data == null)
            return;

        await DiscoverCollidersFromSlotAsync(data.Data, Vector3.Zero, Vector3.One, Quaternion.Identity);
    }

    private static async Task DiscoverCollidersFromSlotAsync(Slot slot, Vector3 position, Vector3 scale, Quaternion rotation)
    {
        if (slot.ID != "Root" && !slot.ID.StartsWith("Reso_"))
            return;

        if (!slot.IsPersistent.Value || !slot.IsActive.Value)
            return;
        
        position *= Unsafe.BitCast<float3, Vector3>(slot.Position.Value);
        scale *= Unsafe.BitCast<float3, Vector3>(slot.Scale.Value);
        rotation *= Unsafe.BitCast<floatQ, Quaternion>(slot.Rotation.Value);
        
        if (slot.IsReferenceOnly)
        {
            SlotData? data = await _link.GetSlotData(new GetSlot
            {
                SlotID = slot.ID,
                Depth = 16,
                IncludeComponentData = true,
            });

            if (data == null)
                return;

            slot = data.Data;
        }

        if (slot.Components != null)
        {
            foreach (Component component in slot.Components)
            {
                if(component.ComponentType != "[FrooxEngine]FrooxEngine.BoxCollider")
                    continue;

                Vector3 size = Unsafe.BitCast<float3, Vector3>(((Field_float3)component.Members["Size"]).Value);
                AddBox(Unsafe.BitCast<float3, Vector3>(slot.Position.Value), scale * size, rotation);
            }
        }

        if (slot.Children != null)
        {
            foreach (Slot child in slot.Children)
            {
                await DiscoverCollidersFromSlotAsync(child, position, scale, rotation);
            }
        }
    }
}