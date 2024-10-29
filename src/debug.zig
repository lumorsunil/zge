const std = @import("std");
const rl = @import("raylib");
const zlm = @import("zlm");
const ecs = @import("ecs");

const cfg = @import("config.zig");

const PhysicsSystem = @import("physics.zig").PhysicsSystem;
const DrawSystem = @import("draw.zig").DrawSystem;
const Camera = @import("camera.zig").Camera;

const CollisionEvent = @import("physics.zig").CollisionEvent;

const RigidBody = @import("physics/rigid-body.zig").RigidBodyFlat;
const Densities = @import("physics/rigid-body.zig").Densities;

pub const DebugScene = struct {
    reg: *ecs.Registry,
    rand: std.Random.DefaultPrng,
    player: ecs.Entity,

    camera: Camera,
    physicsSystem: PhysicsSystem,
    drawSystem: DrawSystem,

    pub fn init(reg: *ecs.Registry) DebugScene {
        return DebugScene{
            .reg = reg,
            .rand = std.Random.DefaultPrng.init(0),
            .player = 0,

            .camera = Camera.init(),
            .physicsSystem = PhysicsSystem.init(reg),
            .drawSystem = DrawSystem.init(reg),
        };
    }

    fn randomPos(self: *DebugScene) zlm.Vec2 {
        return zlm.vec2(
            self.rand.random().float(f32) * cfg.size.x / 8 - cfg.size.x / 4,
            self.rand.random().float(f32) * cfg.size.y / 8 - cfg.size.y / 4,
        );
    }

    pub fn addRandomCircle(self: *DebugScene) void {
        const e = self.reg.create();

        if (self.player == 0) {
            self.player = e;
        }

        const result = RigidBody.init(
            .{ .circle = .{ .radius = 5 + 10 * self.rand.random().float(f32) } },
            Densities.Water,
            0,
            false,
        );

        switch (result) {
            .success => |ibody| {
                var body = ibody;
                body.d.p = self.randomPos();
                self.reg.add(e, body);
            },
            .err => |err| std.log.err("{s}", .{err}),
        }
    }

    pub fn addRandomRectangle(self: *DebugScene) void {
        const e = self.reg.create();

        if (self.player == 0) {
            self.player = e;
        }

        const result = RigidBody.init(
            .{ .rectangle = RigidBody.Shape.Rectangle.init(zlm.vec2(10 + 20 * self.rand.random().float(f32), 10 + 20 * self.rand.random().float(f32))) },
            Densities.Water,
            0,
            false,
        );

        switch (result) {
            .success => |ibody| {
                var body = ibody;
                body.d.p = self.randomPos();
                body.d.rv = 2;
                self.reg.add(e, body);
            },
            .err => |err| std.log.err("{s}", .{err}),
        }
    }

    pub fn deinit(self: DebugScene) void {
        _ = self;
    }

    pub fn update(self: *DebugScene, dt: f32) void {
        var vx: f32 = 0;
        var vy: f32 = 0;
        const speed = 50;

        if (rl.isKeyDown(rl.KeyboardKey.key_a)) {
            vx -= speed;
        }
        if (rl.isKeyDown(rl.KeyboardKey.key_d)) {
            vx += speed;
        }
        if (rl.isKeyDown(rl.KeyboardKey.key_w)) {
            vy -= speed;
        }
        if (rl.isKeyDown(rl.KeyboardKey.key_s)) {
            vy += speed;
        }

        const body = self.reg.get(RigidBody, self.player);
        body.d.v.x = vx;
        body.d.v.y = vy;

        self.physicsSystem.update(dt);

        const collisions = self.physicsSystem.pollCollisions(*DebugScene, self, onCollision);
        _ = collisions;
    }

    pub fn onCollision(self: *DebugScene, collisionEvent: CollisionEvent) void {
        _ = self;
        _ = collisionEvent;
    }

    pub fn draw(self: DebugScene) void {
        rl.beginDrawing();
        rl.clearBackground(rl.Color.dark_blue);
        self.drawSystem.draw(self.camera);
        defer rl.endDrawing();
    }
};
