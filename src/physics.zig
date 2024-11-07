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

const sweep = @import("physics/collision/sweep.zig").sweep;
const SweepLine = @import("physics/collision/sweep.zig").SweepLine;

pub const CollisionEvent = struct {
    entityA: ecs.Entity,
    entityB: ecs.Entity,
    collision: Collision,
};

const MAX_COLLISION_EVENTS = 10000;
const PHYSICS_SUB_STEPS = 8;

const CollisionType = enum {
    rTree,
    sweep,
    bruteForce,
};

const collisionType: CollisionType = .rTree;

pub const PhysicsSystem = struct {
    pendingCollisions: [MAX_COLLISION_EVENTS]Collision = undefined,
    collisionEvents: [MAX_COLLISION_EVENTS]CollisionEvent = undefined,
    gravity: zlm.Vec2,

    view: ecs.BasicView(RigidBody),

    reg: *ecs.Registry,
    bodyContainer: RigidBodyContainer,
    collisionContainer: *CollisionContainer,

    sweepLineBuffer: ArrayList(SweepLine),
    overlappingBuffer: ArrayList(bool),

    var numberOfPendingCollisions: usize = 0;
    var numberOfCollisionEvents: usize = 0;

    pub fn init(allocator: Allocator, reg: *ecs.Registry) !PhysicsSystem {
        const cc = CollisionContainer.init(allocator, reg);
        const cce = reg.create();
        reg.add(cce, cc);
        const ccPtr = reg.get(CollisionContainer, cce);

        return PhysicsSystem{
            //.gravity = zlm.vec2(0, 9.8),
            .gravity = zlm.vec2(0, 0),
            .view = reg.basicView(RigidBody),
            .reg = reg,
            .bodyContainer = try RigidBodyContainer.init(allocator),
            .collisionContainer = ccPtr,

            .sweepLineBuffer = try ArrayList(SweepLine).initCapacity(allocator, 100),
            .overlappingBuffer = try ArrayList(bool).initCapacity(allocator, 100),
        };
    }

    pub fn deinit(self: *const PhysicsSystem) void {
        self.bodyContainer.deinit();
        self.collisionContainer.deinit();
        self.sweepLineBuffer.deinit();
        self.overlappingBuffer.deinit();
    }

    pub fn numberOfTimeSteps(self: PhysicsSystem, dt: f32, maxTimeStep: f32) f32 {
        _ = self; // autofix
        return dt / maxTimeStep;
    }

    pub fn update(self: *PhysicsSystem, dt: f32) void {
        const zone = ztracy.ZoneNC(@src(), "physics upate", 0xff_00_00_00);
        defer zone.End();

        const intervalTimeStep = dt / PHYSICS_SUB_STEPS;

        for (0..PHYSICS_SUB_STEPS) |_| {
            self.updatePositions(intervalTimeStep);
            self.updateCollisions();
            self.resolveCollisions();
        }
    }

    pub fn updateDynamicSubSteps(self: *PhysicsSystem, dt: f32, maxTimeStep: f32) void {
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

        //self.removeFarBodies();

        for (self.view.data()) |entity| {
            const body = self.view.get(entity);
            body.updateTransform();

            if (collisionType == .rTree) {
                self.collisionContainer.updateBody(entity);
            }
        }

        if (collisionType == .rTree) {
            self.collisionContainer.sync();
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
                self.removeRigidBody(entity);
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

        switch (collisionType) {
            .bruteForce => self.updateCollisionsBruteForce(),
            .rTree => self.updateCollisionsWithContainer(),
            .sweep => self.updateCollisionSweep(),
        }
    }

    fn updateCollisionsWithContainer(self: *PhysicsSystem) void {
        for (self.view.data()) |entity| {
            const zone = ztracy.ZoneN(@src(), "uc entity");
            defer zone.End();
            const body = self.view.get(entity);
            if (body.s.isStatic) continue;
            self.collisionContainer.checkCollision(body, entity, self, emitPendingCollision);
        }
    }

    fn updateCollisionSweep(self: *PhysicsSystem) void {
        const bodies = self.view.raw();
        self.sweepLineBuffer.ensureTotalCapacityPrecise(bodies.len * 2) catch unreachable;
        self.sweepLineBuffer.expandToCapacity();
        self.overlappingBuffer.ensureTotalCapacityPrecise(bodies.len * 2) catch unreachable;
        self.overlappingBuffer.expandToCapacity();

        sweep(
            RigidBody,
            bodies,
            self.sweepLineBuffer.items,
            self.overlappingBuffer.items,
            self,
            onAxisOverlap,
        );
    }

    fn onAxisOverlap(self: *PhysicsSystem, a: usize, bs: []bool, n: usize) void {
        const bodyA = &self.view.raw()[a];
        var left = n;

        for (0.., bs) |i, b| {
            if (!b or left <= 0) continue;
            const bodyB = &self.view.raw()[i];

            switch (bodyA.checkCollision(bodyB)) {
                .noCollision => {},
                .collision => |collision| self.emitPendingCollision(collision),
            }

            left -= 1;
        }
    }

    fn updateCollisionsBruteForce(self: *PhysicsSystem) void {
        const bodies = self.view.raw();

        if (bodies.len == 0) return;

        for (0.., bodies[0 .. bodies.len - 1]) |iA, *bodyA| {
            if (bodyA.s.isStatic) continue;

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
        std.log.info("RIGID BODY ADDED {}", .{entityId});
        var isPointersInvalidated: bool = false;

        self.bodyContainer.setRigidBody(
            entityId,
            pos,
            zlm.vec2(0, 0),
            zlm.vec2(0, 0),
            0,
            0,
            0,
            static.isStatic,
            &isPointersInvalidated,
        );

        if (isPointersInvalidated) {
            std.log.info("RIGID BODY POINTERS INVALIDATED", .{});
            self.updateRigidBodiesInRegistry();
        }

        const dynamic = self.bodyContainer.getRigidBody(entityId);

        self.reg.add(entity, RigidBody.init(static, dynamic));
        const body = self.view.get(entity);

        if (collisionType == .rTree) {
            self.collisionContainer.insertBody(entity);
        }

        return body;
    }

    pub fn removeRigidBody(self: *PhysicsSystem, entity: ecs.Entity) void {
        if (collisionType == .rTree) {
            self.collisionContainer.removeBody(entity);
        }
        self.bodyContainer.removeRigidBody(self.reg.entityId(entity));
        self.reg.destroy(entity);
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
