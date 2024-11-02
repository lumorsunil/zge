const std = @import("std");
const Allocator = std.mem.Allocator;
const ArrayList = std.ArrayList;
const rl = @import("raylib");
const ecs = @import("ecs");
const zlm = @import("zlm");
const ztracy = @import("ztracy");

const cfg = @import("config.zig");

const RigidBody = @import("physics/rigid-body-flat.zig").RigidBodyFlat;
const RigidBodyStaticParams = @import("physics/rigid-body-static.zig").RigidBodyStaticParams;
const RigidBodyDynamicParams = @import("physics/rigid-body-dynamic.zig").RigidBodyDynamicParams;
const RigidBodyContainer = @import("physics/rigid-body-container.zig").RigidBodyContainer;
const Collision = @import("physics/collision/result.zig").Collision;
const CollisionContainer = @import("physics/collision/container.zig").CollisionContainer;
const resolveCollision = @import("physics/collision.zig").resolveCollision;

pub const CollisionEvent = struct {
    entityA: ecs.Entity,
    entityB: ecs.Entity,
    collision: Collision,
};

const MAX_COLLISION_EVENTS = 10000;

pub const PhysicsSystem = struct {
    pendingCollisions: [MAX_COLLISION_EVENTS]Collision = undefined,
    collisionEvents: [MAX_COLLISION_EVENTS]CollisionEvent = undefined,
    gravity: zlm.Vec2,

    view: ecs.BasicView(RigidBody),

    reg: *ecs.Registry,
    bodyContainer: RigidBodyContainer,
    collisionContainer: CollisionContainer,

    var numberOfPendingCollisions: usize = 0;
    var numberOfCollisionEvents: usize = 0;

    pub fn init(allocator: Allocator, reg: *ecs.Registry) !PhysicsSystem {
        return PhysicsSystem{
            .gravity = zlm.vec2(0, 9.8),
            .view = reg.basicView(RigidBody),
            .reg = reg,
            .bodyContainer = try RigidBodyContainer.init(allocator),
            .collisionContainer = CollisionContainer.init(allocator),
        };
    }

    pub fn deinit(self: *const PhysicsSystem) void {
        self.bodyContainer.deinit();
        self.collisionContainer.deinit();
    }

    pub fn numberOfTimeSteps(self: PhysicsSystem, dt: f32, maxTimeStep: f32) f32 {
        _ = self; // autofix
        return dt / maxTimeStep;
    }

    pub fn update(self: *PhysicsSystem, dt: f32, maxTimeStep: f32) void {
        const zone = ztracy.ZoneNC(@src(), "physics upate", 0xff_00_00_00);
        defer zone.End();

        const timeStep = @min(dt, maxTimeStep);

        var timeLeft = dt;

        while (timeLeft > 0) {
            const actualTimeStep = @min(timeLeft, timeStep);
            timeLeft -= timeStep;
            self.updatePositions(actualTimeStep);
            self.updateCollisions();
            self.resolveCollisions();
        }
    }

    fn updatePositions(self: *PhysicsSystem, dt: f32) void {
        const zone = ztracy.ZoneNC(@src(), "update positions", 0xff_00_00_00);
        defer zone.End();

        self.bodyContainer.updatePositions(self.gravity, dt);

        self.removeFarBodies();

        for (self.view.data()) |entity| {
            const entityId = self.reg.entityId(entity);
            const body = self.view.get(entity);
            body.updateTransform();
            self.collisionContainer.updateBody(entityId, body);
        }
    }

    fn updatePositions0(self: PhysicsSystem, dt: f32) void {
        for (self.dynamicView.raw()) |*body| {
            body.a = body.a.add(self.gravity);

            body.v = body.v.add(body.a.scale(dt));
            body.p = body.p.add(body.v.scale(dt));

            body.rv += body.ra * dt;
            body.r += body.rv * dt;

            body.a = zlm.vec2(0, 0);

            body.updateTransform();
        }
    }

    fn removeFarBodies(self: *PhysicsSystem) void {
        for (self.view.data()) |entity| {
            const body = self.view.getConst(entity);
            if (body.d.p.x.* < -cfg.sizeHalfW or
                body.d.p.x.* > cfg.sizeHalfW or
                body.d.p.y.* < -cfg.sizeHalfH or
                body.d.p.y.* > cfg.sizeHalfH)
            {
                self.bodyContainer.removeRigidBody(self.reg.entityId(entity));
                self.reg.destroy(entity);
            }
        }
    }

    fn wrapPositions(body: *RigidBody) void {
        if (body.d.p.x < -cfg.sizeHalfW) {
            body.d.p.x = -body.d.p.x;
        }

        if (body.d.p.x > cfg.sizeHalfW) {
            body.d.p.x = -body.d.p.x;
        }

        if (body.d.p.y < -cfg.sizeHalfH) {
            body.d.p.y = body.d.p.y;
        }

        if (body.d.p.y > cfg.sizeHalfH) {
            body.d.p.y = body.d.p.y;
        }
    }

    fn updateCollisions(self: *PhysicsSystem) void {
        const zone = ztracy.ZoneNC(@src(), "update collisions", 0xff_00_00_00);
        defer zone.End();

        const bodies = self.view.raw();

        zone.Text("Bodies:");
        zone.Value(bodies.len);

        //self.updateCollisionsBruteForce();
        self.updateCollisionsWithContainer();
    }

    fn updateCollisionsWithContainer(self: *PhysicsSystem) void {
        for (self.view.raw()) |*body| {
            self.collisionContainer.checkCollision(body, self, emitPendingCollision);
        }
    }

    fn updateCollisionsBruteForce(self: *PhysicsSystem) void {
        const bodies = self.view.raw();

        if (bodies.len == 0) return;

        for (0.., bodies[0 .. bodies.len - 1]) |iA, *bodyA| {
            for (iA + 1.., bodies[iA + 1 .. bodies.len]) |iB, *bodyB| {
                _ = iB; // autofix
                const result = bodyA.checkCollision(bodyB);

                switch (result) {
                    .collision => |collision| {
                        //                        const event = CollisionEvent{
                        //                            .entityA = self.view.data()[iA],
                        //                            .entityB = self.view.data()[iB],
                        //                            .collision = collision,
                        //                        };
                        //                        self.emitCollisionEvent(event);
                        self.emitPendingCollision(collision);
                    },
                    .noCollision => continue,
                }
            }
        }
    }

    fn resolveCollisions(self: PhysicsSystem) void {
        const zone = ztracy.ZoneNC(@src(), "resolve collisions", 0xff_00_00_00);
        defer zone.End();

        for (self.pendingCollisions[0..numberOfPendingCollisions]) |collision| {
            resolveCollision(collision);
        }

        numberOfPendingCollisions = 0;
    }

    fn emitPendingCollision(self: *PhysicsSystem, collision: Collision) void {
        self.pendingCollisions[numberOfPendingCollisions] = collision;
        numberOfPendingCollisions += 1;
    }

    fn emitCollisionEvent(self: *PhysicsSystem, event: CollisionEvent) void {
        self.collisionEvents[numberOfCollisionEvents] = event;
        numberOfCollisionEvents += 1;
    }

    pub fn pollCollisions(self: PhysicsSystem, comptime T: type, context: T, onEvent: fn (context: T, collisionEvent: CollisionEvent) void) void {
        for (0..numberOfCollisionEvents) |i| {
            onEvent(context, self.collisionEvents[i]);
        }

        numberOfCollisionEvents = 0;
    }

    pub fn addRigidBody(
        self: *PhysicsSystem,
        entity: ecs.Entity,
        pos: zlm.Vec2,
        static: RigidBodyStaticParams,
    ) *RigidBody {
        const entityId = self.reg.entityId(entity);
        var isPointersValidated: bool = false;

        self.bodyContainer.setRigidBody(
            entityId,
            pos,
            zlm.vec2(0, 0),
            zlm.vec2(0, 0),
            0,
            0,
            0,
            static.isStatic,
            &isPointersValidated,
        );

        if (isPointersValidated) {
            self.updateRigidBodiesInRegistry();
        }

        const dynamic = self.bodyContainer.getRigidBody(entityId);

        self.reg.add(entity, RigidBody.init(static, dynamic));
        const body = self.view.get(entity);

        self.collisionContainer.insertBody(entityId, body);

        return body;
    }

    fn updateRigidBodiesInRegistry(self: *PhysicsSystem) void {
        const view = self.reg.basicView(RigidBody);

        for (view.data()) |entity| {
            const body = view.get(entity);
            const id = self.reg.entityId(entity);
            self.bodyContainer.updateRigidBodyPointers(id, &body.*.d);
        }
    }
};
