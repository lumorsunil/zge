const std = @import("std");
const Allocator = std.mem.Allocator;
const rl = @import("raylib");
const zlm = @import("zlm");
const ecs = @import("ecs");

const cfg = @import("config.zig");

const PhysicsSystem = @import("physics.zig").PhysicsSystem;
const DrawSystem = @import("draw.zig").DrawSystem;
const Camera = @import("camera.zig").Camera;

const CollisionEvent = @import("physics.zig").CollisionEvent;

const RigidBody = @import("physics/rigid-body-flat.zig").RigidBodyFlat;
const RigidBodyStaticParams = @import("physics/rigid-body-static.zig").RigidBodyStaticParams;
const Rectangle = @import("physics/shape.zig").Rectangle;
const Circle = @import("physics/shape.zig").Circle;
const Densities = @import("physics/rigid-body-static.zig").Densities;

pub const DebugScene = struct {
    reg: *ecs.Registry,
    rand: std.Random.DefaultPrng,
    player: ecs.Entity,

    camera: Camera,
    physicsSystem: PhysicsSystem,
    drawSystem: DrawSystem,

    pub fn init(allocator: Allocator, reg: *ecs.Registry) !DebugScene {
        return DebugScene{
            .reg = reg,
            .rand = std.Random.DefaultPrng.init(0),
            .player = 0,

            .camera = Camera.init(),
            .physicsSystem = try PhysicsSystem.init(allocator, reg),
            .drawSystem = try DrawSystem.init(allocator, reg),
        };
    }

    pub fn deinit(self: DebugScene) void {
        self.physicsSystem.deinit();
        self.drawSystem.deinit();
    }

    pub fn randomPos(self: *DebugScene) zlm.Vec2 {
        return zlm.vec2(
            self.rand.random().float(f32) * cfg.size.x - cfg.size.x / 2,
            self.rand.random().float(f32) * cfg.size.y - cfg.size.y / 2,
        );
    }

    pub fn randomSize(self: *DebugScene) zlm.Vec2 {
        return zlm.vec2(
            self.randomRadius() * 2,
            self.randomRadius() * 2,
        );
    }

    pub fn randomRadius(self: *DebugScene) f32 {
        return 5 + 10 * self.rand.random().float(f32);
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

    pub fn addPlayer(self: *DebugScene, position: zlm.Vec2, size: zlm.Vec2) void {
        const e = self.reg.create();

        self.player = e;

        const result = RigidBodyStaticParams.init(
            .{ .rectangle = Rectangle.init(size) },
            Densities.Water,
            1,
            false,
        );

        switch (result) {
            .success => |static| {
                _ = self.physicsSystem.addRigidBody(e, position, static);
            },
            .err => |err| std.log.err("{s}", .{err}),
        }
    }

    pub fn addRectangle(self: *DebugScene, position: zlm.Vec2, size: zlm.Vec2, isStatic: bool) void {
        const e = self.reg.create();

        const result = RigidBodyStaticParams.init(
            .{ .rectangle = Rectangle.init(size) },
            Densities.Element.Osmium,
            0.2,
            isStatic,
        );

        switch (result) {
            .success => |static| {
                _ = self.physicsSystem.addRigidBody(e, position, static);
            },
            .err => |err| std.log.err("{s}", .{err}),
        }
    }

    var bodiesAdded: usize = 0;
    var addt: f64 = 0;
    var randomPositions: [300]zlm.Vec2 = randomPositions: {
        var rand = std.Random.DefaultPrng.init(0);
        var rps: [300]zlm.Vec2 = undefined;

        @setEvalBranchQuota(300000);

        for (0..rps.len) |i| {
            rps[i] = .{
                .x = rand.random().float(f32) * cfg.size.x - cfg.size.x / 2,
                .y = rand.random().float(f32) * cfg.size.y - cfg.size.y / 2,
            };
        }

        const Sorter = struct {
            pub fn lessThanFn(_: void, lhs: zlm.Vec2, rhs: zlm.Vec2) bool {
                return lhs.x < rhs.x;
            }
        };

        std.mem.sort(zlm.Vec2, &rps, {}, Sorter.lessThanFn);

        break :randomPositions rps;
    };

    pub fn update(self: *DebugScene, dt: f32, t: f64) void {
        var force = zlm.Vec2.zero;
        const speed = 10;

        if (rl.isKeyDown(rl.KeyboardKey.key_a)) {
            force.x -= speed;
        }
        if (rl.isKeyDown(rl.KeyboardKey.key_d)) {
            force.x += speed;
        }
        if (rl.isKeyDown(rl.KeyboardKey.key_w)) {
            force.y -= speed;
        }
        if (rl.isKeyDown(rl.KeyboardKey.key_s)) {
            force.y += speed;
        }

        if (rl.isMouseButtonPressed(rl.MouseButton.mouse_button_left)) {
            for (0..1) |_| {
                self.addRectangle(
                    zlm.vec2(
                        @floatFromInt(rl.getMouseX()),
                        @floatFromInt(rl.getMouseY()),
                    ).sub(
                        cfg.size.div(zlm.Vec2.all(2)),
                    ),
                    self.randomSize(),
                    false,
                );
            }
        }

        if (self.reg.valid(self.player)) {
            const body = self.reg.get(RigidBody, self.player);
            body.applyForce(force);
        }

        //if (dt < 0.017) {
        if (bodiesAdded < 300 and t >= addt) {
            //self.addRandomRectangle();
            //self.addRandomCircle();
            self.addRectangle(randomPositions[bodiesAdded], self.randomSize(), false);
            bodiesAdded += 1;
            addt += 0.2;
        }

        self.physicsSystem.update(dt);
        //self.physicsSystem.updateDynamicSubSteps(dt, 0.0005);

        const collisions = self.physicsSystem.pollCollisions(*DebugScene, self, onCollision);
        _ = collisions;
    }

    pub fn onCollision(self: *DebugScene, collisionEvent: CollisionEvent) void {
        _ = self;
        _ = collisionEvent;
    }

    pub fn draw(self: *DebugScene) void {
        rl.beginDrawing();
        rl.clearBackground(rl.Color.dark_blue);
        self.drawSystem.draw(self.camera);
        rl.drawFPS(5, 5);
        defer rl.endDrawing();
    }
};
