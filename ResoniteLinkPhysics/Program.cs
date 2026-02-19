using System.Collections.Concurrent;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using BepuUtilities.Memory;
using ResoniteLink;

namespace ResoniteLinkPhysics;
public static class Program
{
    private static bool _running = true;
    private static Simulation _sim;
    private static LinkInterface _link;
    
    // The prefix prevents multiple REPL sessions from colliding with each other's ID's
    private static string _prefix;
    private static int _idPool;
    private static string AllocateId() => $"{nameof(ResoniteLinkPhysics)}_{_prefix}_{_idPool++:X}";

    private static readonly ConcurrentQueue<Ball> _balls = []; 
    
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
        const int port = 9362;
        
        using LinkInterface link = new();
        await link.Connect(new Uri($"ws://localhost:{port}"), CancellationToken.None);
        _link = link;

        SessionData? session = await link.GetSessionData();
        if (session == null)
            throw new Exception("Session not found");

        _prefix = session.UniqueSessionId;
        
        Console.WriteLine("Connected!");
        
        using BufferPool bufferPool = new();
        using Simulation simulation = Simulation.Create(bufferPool, new NarrowPhaseCallbacks(new SpringSettings(30, 1)), new PoseIntegratorCallbacks(new Vector3(0, -10, 0)), new SolveDescription(8, 1));
        _sim = simulation;

        using ThreadDispatcher dispatcher = new(Environment.ProcessorCount);

        List<DataModelOperation> initOps = [];
        
        const int totalBalls = 3;
        for (int i = 0; i < totalBalls; i++)
        {
            initOps.AddRange(AddBall(new Vector3((i * 2.5f) - (totalBalls / 2.0f), 5.0f, 0.0f), (i + 1) / (float)totalBalls));
        }

        AddBox(Vector3.Zero, new Vector3(1000, 0, 1000));

        try
        {
            Console.WriteLine("Submitting init batch...");
            BatchResponse? responses = await _link.RunDataModelOperationBatch(initOps);
            foreach (Response response in responses.Responses)
            {
                if (!response.Success)
                    throw new Exception(response.ErrorInfo);
            }

            const float tickrate = 60.0f;
            
            Console.WriteLine("Simulation running...");
            List<DataModelOperation> ops = [];
            while (_running)
            {
                ops.Clear();
                simulation.Timestep(1.0f / tickrate, dispatcher);
                foreach (Ball ball in _balls)
                {
                    ops.Add(ball.UpdatePosition(simulation));
                }

                await _link.RunDataModelOperationBatch(ops);
                
                Thread.Sleep((int)(1000.0f / tickrate));
            }
        }
        finally
        {
            Console.WriteLine("Exiting");
            await RemoveAllSlots();
        }
    }

    private static async Task RemoveAllSlots()
    {
        List<DataModelOperation> ops = [];
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
        BodyInertia inertia = sphere.ComputeInertia(1f);

        TypedIndex shape = _sim.Shapes.Add(sphere);
        BodyDescription desc = BodyDescription.CreateDynamic(position, inertia, shape, new BodyActivityDescription(0.01f));
        BodyHandle bodyHandle = _sim.Bodies.Add(desc);

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

        string materialId = AllocateId();
        yield return new AddComponent
        {
            ContainerSlotId = id,
            Data = new Component
            {
                ID = materialId,
                ComponentType = "[FrooxEngine]FrooxEngine.PBS_Metallic",
            }
        };

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
                    {"Radius", new Field_float {Value = radius}}
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
                    {"CharacterCollider", new Field_bool {Value = true}}
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

    public static void AddBox(Vector3 position, Vector3 size)
    {
        Box box = new(size.X, size.Y, size.Z);

        TypedIndex shape = _sim.Shapes.Add(box);
        StaticDescription desc = new(position, shape);
        _sim.Statics.Add(desc);
    }
}