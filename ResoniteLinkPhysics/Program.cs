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
        
        const int totalBalls = 500;
        for (int i = 0; i < totalBalls; i++)
        {
            const float range = 10f;
            const float height = 0.75f;
            float radius = Rand(0.1f, height);
            initOps.AddRange(AddBall(new Vector3(Rand(-range, range), Rand(height, range), Rand(-range, range)), radius));
        }

        // TODO: search for boxcolliders in the world
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
                    ops.Add(ball.UpdatePosition(simulation));
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
        float mass = radius * 0.5f;
        BodyInertia inertia = sphere.ComputeInertia(mass);

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

        Color color = ColorFromHSV(Random.Shared.NextSingle() * 255, 1, 1);

        // todo: create a shared material so we don't make 594838953268745 drawcalls
        // no property blocks for albedo color, but we can technically create a SolidColorTexture
        string materialId = AllocateId();
        yield return new AddComponent
        {
            ContainerSlotId = id,
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
                    {"Mass", new Field_float {Value = mass}},
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

    private static void AddBox(Vector3 position, Vector3 size)
    {
        Box box = new(size.X, size.Y, size.Z);

        TypedIndex shape = _sim.Shapes.Add(box);
        StaticDescription desc = new(position, shape);
        _sim.Statics.Add(desc);
    }
}