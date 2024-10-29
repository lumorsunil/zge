const std = @import("std");
const Allocator = std.mem.Allocator;
const ArrayList = std.ArrayList;
const rl = @import("raylib");
const ecs = @import("ecs");

const cfg = @import("config.zig");

const RigidBody = @import("physics/rigid-body.zig").RigidBodyFlat;
const Collision = RigidBody.Collision;

pub const CollisionEvent = struct {
    entityA: ecs.Entity,
    entityB: ecs.Entity,
};

const MAX_COLLISION_EVENTS = 200;

pub const PhysicsSystem = struct {
    collisionEvents: [MAX_COLLISION_EVENTS]CollisionEvent = undefined,
    view: ecs.BasicView(RigidBody),

    var numberOfCollisionEvent: usize = 0;

    pub fn init(reg: *ecs.Registry) PhysicsSystem {
        return PhysicsSystem{
            .view = reg.basicView(RigidBody),
        };
    }

    pub fn update(self: *PhysicsSystem, dt: f32) void {
        self.updatePositions(dt);
        self.updateCollisions();
    }

    fn updatePositions(self: PhysicsSystem, dt: f32) void {
        for (self.view.raw()) |*body| {
            if (body.s.isStatic) continue;

            body.d.v = body.d.v.add(body.d.a.scale(dt));

            //physics.v.x -= physics.v.x * (1 - physics.f) * dt;
            //physics.v.y -= physics.v.y * (1 - physics.f) * dt;

            body.d.p = body.d.p.add(body.d.v.scale(dt));

            body.d.rv += body.d.ra * dt;
            body.d.r += body.d.rv * dt;
        }
    }

    fn updateCollisions(self: *PhysicsSystem) void {
        const bodies = self.view.raw();

        if (bodies.len == 0) return;

        for (0.., bodies[0 .. bodies.len - 1]) |iA, *bodyA| {
            for (iA + 1.., bodies[iA + 1 .. bodies.len]) |iB, *bodyB| {
                const result = bodyA.checkCollision(bodyB.*);

                switch (result) {
                    .collision => |collision| {
                        self.resolveCollision(bodyA, bodyB, collision);

                        const event = CollisionEvent{
                            .entityA = self.view.data()[iA],
                            .entityB = self.view.data()[iB],
                        };
                        self.emitCollisionEvent(event);
                    },
                    .noCollision => continue,
                }
            }
        }
    }

    fn emitCollisionEvent(self: *PhysicsSystem, event: CollisionEvent) void {
        self.collisionEvents[numberOfCollisionEvent] = event;
        numberOfCollisionEvent += 1;
    }

    fn resolveCollision(_: PhysicsSystem, rbA: *RigidBody, rbB: *RigidBody, collision: Collision) void {
        RigidBody.resolveCollision(rbA, rbB, collision);
    }

    pub fn pollCollisions(self: PhysicsSystem, comptime T: type, context: T, onEvent: fn (context: T, collisionEvent: CollisionEvent) void) void {
        for (0..numberOfCollisionEvent) |i| {
            onEvent(context, self.collisionEvents[i]);
        }

        numberOfCollisionEvent = 0;
    }
};
