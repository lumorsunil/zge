const std = @import("std");
const Allocator = std.mem.Allocator;
const rl = @import("raylib");
const ecs = @import("ecs");

const cfg = @import("config.zig");

const Screen = @import("screen.zig").Screen;
const PhysicsSystem = @import("physics.zig").PhysicsSystem;
const DrawSystem = @import("draw.zig").DrawSystem;
const Camera = @import("camera.zig").Camera;

const V = @import("vector.zig").V;
const Vector = @import("vector.zig").Vector;

const CollisionEvent = @import("physics.zig").Collision;

const RigidBody = @import("physics/rigid-body-flat.zig").RigidBodyFlat;
const RigidBodyStaticParams = @import("physics/rigid-body-static.zig").RigidBodyStaticParams;
const AABB = @import("physics/shape.zig").AABB;
const Rectangle = @import("physics/shape.zig").Rectangle;
const Circle = @import("physics/shape.zig").Circle;
const Densities = @import("physics/rigid-body-static.zig").Densities;

var screen = Screen.init(cfg.size);
var camera = Camera.init();

pub const DebugScene = struct {
    allocator: Allocator,
    reg: *ecs.Registry,
    rand: std.Random.DefaultPrng,
    player: ecs.Entity,

    screen: *const Screen,
    camera: *Camera,
    physicsSystem: PhysicsSystem = undefined,
    drawSystem: DrawSystem = undefined,

    const boundary: AABB = .{
        .tl = -cfg.sizeHalf,
        .br = cfg.sizeHalf,
        .isMinimal = true,
    };

    pub fn init(allocator: Allocator, reg: *ecs.Registry) DebugScene {
        return DebugScene{
            .allocator = allocator,
            .reg = reg,
            .rand = std.Random.DefaultPrng.init(0),
            .player = 0,

            .screen = &screen,
            .camera = &camera,
            .physicsSystem = PhysicsSystem.init(allocator, reg, boundary),
            .drawSystem = DrawSystem.init(allocator, reg, &screen, &camera),
        };
    }

    pub fn bind(self: *DebugScene) void {
        self.drawSystem.bind();
    }

    pub fn deinit(self: *DebugScene) void {
        self.physicsSystem.deinit();
        self.drawSystem.deinit();
    }

    pub fn randomPos(self: *DebugScene) Vector {
        return V.random(&self.rand) * cfg.size - cfg.size / V.scalar(2);
    }

    const maxSize = V.all(maxRadius * 2);
    pub fn randomSize(self: *DebugScene) Vector {
        return V.init(
            self.randomRadius() * 2,
            self.randomRadius() * 2,
        );
    }

    const maxRadius = 0.1 + 1;
    pub fn randomRadius(self: *DebugScene) f32 {
        return 0.1 + 1 * self.rand.random().float(f32);
    }

    pub fn addRandomCircle(self: *DebugScene) void {
        const e = self.reg.create();

        const result = RigidBodyStaticParams.init(
            .{ .circle = Circle.init(self.randomRadius()) },
            Densities.Element.Osmium,
            0.2,
            false,
        );

        switch (result) {
            .success => |static| {
                _ = self.physicsSystem.addRigidBody(e, self.randomPos(), static);
            },
            .err => |err| std.log.err("{s}", .{err}),
        }
    }

    pub fn addRandomRectangle(self: *DebugScene) void {
        self.addRectangle(self.randomPos(), self.randomSize(), self.rand.random().boolean());
    }

    pub fn addPlayer(self: *DebugScene, position: Vector, size: Vector) void {
        const e = self.reg.create();

        self.player = e;

        const result = RigidBodyStaticParams.init(
            .{ .rectangle = Rectangle.init(V.zero, size) },
            Densities.Water,
            1,
            false,
        );

        switch (result) {
            .success => |static| {
                _ = self.physicsSystem.addRigidBody(e, .{ .pos = position }, static);
            },
            .err => |err| std.log.err("{s}", .{err}),
        }
    }

    pub fn addRectangle(
        self: *DebugScene,
        position: Vector,
        size: Vector,
        isStatic: bool,
    ) void {
        const e = self.reg.create();

        const result = RigidBodyStaticParams.init(
            .{ .rectangle = Rectangle.init(V.zero, size) },
            Densities.Element.Osmium,
            0.2,
            isStatic,
        );

        switch (result) {
            .success => |static| {
                _ = self.physicsSystem.addRigidBody(e, .{ .pos = position }, static);
            },
            .err => |err| std.log.err("{s}", .{err}),
        }
    }

    pub fn addCircle(self: *DebugScene, position: Vector, radius: f32, isStatic: bool) void {
        const e = self.reg.create();

        const result = RigidBodyStaticParams.init(
            .{ .circle = Circle.init(radius) },
            Densities.Element.Osmium,
            0.2,
            isStatic,
        );

        switch (result) {
            .success => |static| {
                _ = self.physicsSystem.addRigidBody(e, .{ .pos = position }, static);
            },
            .err => |err| std.log.err("{s}", .{err}),
        }
    }

    const numberOfBodiesToAdd = 100;
    var bodiesAdded: usize = 0;
    var addt: f64 = 0;
    var randomPositions: [numberOfBodiesToAdd]Vector = randomPositions: {
        var rand = std.Random.DefaultPrng.init(0);
        var rps: [numberOfBodiesToAdd]Vector = undefined;

        @setEvalBranchQuota(3000000);

        for (0..rps.len) |i| {
            rps[i] = V.init(
                std.math.clamp(
                    rand.random().float(f32) * V.x(cfg.size) - V.x(cfg.size) / 2,
                    -V.x(cfg.size) / 2 + V.x(maxSize),
                    V.x(cfg.size) / 2 - V.x(maxSize),
                ),
                std.math.clamp(
                    rand.random().float(f32) * V.y(cfg.size) - V.y(cfg.size) / 2,
                    -V.y(cfg.size) / 2 + V.y(maxSize),
                    V.y(cfg.size) / 2 - V.y(maxSize),
                ),
            );
        }

        const Sorter = struct {
            pub fn lessThanFn(_: void, lhs: Vector, rhs: Vector) bool {
                return V.x(lhs) < V.x(rhs);
            }
        };

        std.mem.sort(Vector, &rps, {}, Sorter.lessThanFn);

        break :randomPositions rps;
    };

    pub fn update(self: *DebugScene, dt: f32, t: f64) void {
        _ = t; // autofix
        var force = V.zero;
        const speed = 1000;

        if (rl.isKeyDown(rl.KeyboardKey.a)) {
            force -= V.onlyX(speed);
        }
        if (rl.isKeyDown(rl.KeyboardKey.d)) {
            force += V.onlyX(speed);
        }
        if (rl.isKeyDown(rl.KeyboardKey.w)) {
            force -= V.onlyY(speed);
        }
        if (rl.isKeyDown(rl.KeyboardKey.s)) {
            force += V.onlyY(speed);
        }

        if (rl.isMouseButtonPressed(rl.MouseButton.left)) {
            for (0..1) |_| {
                self.addRectangle(
                    V.fromRl(rl.getMousePosition()) - cfg.sizeHalf,
                    self.randomSize(),
                    false,
                );
            }
        }

        if (self.reg.valid(self.player)) {
            const body = self.reg.get(RigidBody, self.player);
            body.applyForce(force);
        }

        self.addBodiesUntilLag(dt);

        self.physicsSystem.update(dt);
        //self.physicsSystem.updateDynamicSubSteps(dt, 0.0005);

        const collisions = self.physicsSystem.pollCollisions(*DebugScene, self, onCollision);
        _ = collisions;
    }

    fn addBodiesUntilLag(self: *DebugScene, dt: f32) void {
        if (dt < 0.017) {
            //if (bodiesAdded < randomPositions.len and dt < 0.017) {
            //self.addCircle(randomPositions[bodiesAdded], self.randomRadius(), false);
            const bodiesToAdd = 100;
            for (0..bodiesToAdd) |i| {
                if (bodiesAdded + i >= numberOfBodiesToAdd) {
                    self.addRectangle(self.randomPos(), self.randomSize(), false);
                } else {
                    self.addRectangle(randomPositions[bodiesAdded + i], self.randomSize(), false);
                }
            }
            bodiesAdded += bodiesToAdd;
            addt += 0.05;
        }
    }

    pub fn onCollision(self: *DebugScene, collisionEvent: CollisionEvent) void {
        _ = self;
        _ = collisionEvent;
    }

    pub fn draw(self: *DebugScene) void {
        rl.beginDrawing();
        rl.clearBackground(rl.Color.dark_blue);
        self.drawSystem.draw();
        rl.drawFPS(5, 5);
        defer rl.endDrawing();
    }
};
