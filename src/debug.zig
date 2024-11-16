const std = @import("std");
const Allocator = std.mem.Allocator;
const rl = @import("raylib");
const zlm = @import("zlm");
const ecs = @import("ecs");

const cfg = @import("config.zig");

const Screen = @import("screen.zig").Screen;
const PhysicsSystem = @import("physics.zig").PhysicsSystem;
const DrawSystem = @import("draw.zig").DrawSystem;
const Camera = @import("camera.zig").Camera;

const CollisionEvent = @import("physics.zig").CollisionEvent;

const RigidBody = @import("physics/rigid-body-flat.zig").RigidBodyFlat;
const RigidBodyStaticParams = @import("physics/rigid-body-static.zig").RigidBodyStaticParams;
const Rectangle = @import("physics/shape.zig").Rectangle;
const Circle = @import("physics/shape.zig").Circle;
const Densities = @import("physics/rigid-body-static.zig").Densities;

var screen = Screen.init(cfg.size.x, cfg.size.y);

pub const DebugScene = struct {
    reg: *ecs.Registry,
    rand: std.Random.DefaultPrng,
    player: ecs.Entity,

    screen: *const Screen,
    camera: Camera,
    physicsSystem: PhysicsSystem,
    drawSystem: DrawSystem,

    pub fn init(allocator: Allocator, reg: *ecs.Registry) DebugScene {
        return DebugScene{
            .reg = reg,
            .rand = std.Random.DefaultPrng.init(0),
            .player = 0,

            .screen = &screen,
            .camera = Camera.init(),
            .physicsSystem = PhysicsSystem.init(allocator, reg),
            .drawSystem = DrawSystem.init(allocator, reg, &screen),
        };
    }

    pub fn bind(self: *DebugScene) void {
        self.drawSystem.bind();
    }

    pub fn deinit(self: *DebugScene) void {
        self.physicsSystem.deinit();
        self.drawSystem.deinit();
    }

    pub fn randomPos(self: *DebugScene) zlm.Vec2 {
        return zlm.vec2(
            self.rand.random().float(f32) * cfg.size.x - cfg.size.x / 2,
            self.rand.random().float(f32) * cfg.size.y - cfg.size.y / 2,
        );
    }

    const maxSize = zlm.vec2(maxRadius * 2, maxRadius * 2);
    pub fn randomSize(self: *DebugScene) zlm.Vec2 {
        return zlm.vec2(
            self.randomRadius() * 2,
            self.randomRadius() * 2,
        );
    }

    const maxRadius = 1 + 5;
    pub fn randomRadius(self: *DebugScene) f32 {
        return 1 + 5 * self.rand.random().float(f32);
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
            .{ .rectangle = Rectangle.init(zlm.Vec2.zero, size) },
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
            .{ .rectangle = Rectangle.init(zlm.Vec2.zero, size) },
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

    pub fn addCircle(self: *DebugScene, position: zlm.Vec2, radius: f32, isStatic: bool) void {
        const e = self.reg.create();

        const result = RigidBodyStaticParams.init(
            .{ .circle = Circle.init(radius) },
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

    const numberOfBodiesToAdd = 3000;
    var bodiesAdded: usize = 0;
    var addt: f64 = 0;
    var randomPositions: [numberOfBodiesToAdd]zlm.Vec2 = randomPositions: {
        var rand = std.Random.DefaultPrng.init(0);
        var rps: [numberOfBodiesToAdd]zlm.Vec2 = undefined;

        @setEvalBranchQuota(3000000);

        for (0..rps.len) |i| {
            rps[i] = .{
                .x = std.math.clamp(rand.random().float(f32) * cfg.size.x - cfg.size.x / 2, -cfg.size.x / 2 + maxSize.x, cfg.size.x / 2 - maxSize.x),
                .y = std.math.clamp(rand.random().float(f32) * cfg.size.y - cfg.size.y / 2, -cfg.size.y / 2 + maxSize.y, cfg.size.y / 2 - maxSize.y),
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
        _ = t; // autofix
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
            const bodiesToAdd = 10;
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
        self.drawSystem.draw(self.camera);
        rl.drawFPS(5, 5);
        defer rl.endDrawing();
    }
};
