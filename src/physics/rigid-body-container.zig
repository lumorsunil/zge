const std = @import("std");
const Allocator = std.mem.Allocator;
const ArrayList = std.ArrayList;

const cfg = @import("../config.zig");

const RigidBodyStaticParams = @import("rigid-body.zig").RigidBodyStaticParams;
const RigidBodyDynamicParams = @import("rigid-body.zig").RigidBodyDynamicParams;

const MAX_RIGID_BODIES = cfg.MAX_ENTITIES;

pub const RigidBodyContainer = struct {
    rigidBodiesStaticParams: [MAX_RIGID_BODIES]RigidBodyStaticParams = undefined,

    rigidBodiesCapacity: usize,

    rigidBodiesPositionX: ArrayList(f32) = undefined,
    rigidBodiesPositionY: ArrayList(f32) = undefined,
    rigidBodiesVelocityX: ArrayList(f32) = undefined,
    rigidBodiesVelocityY: ArrayList(f32) = undefined,
    rigidBodiesAccelerationX: ArrayList(f32) = undefined,
    rigidBodiesAccelerationY: ArrayList(f32) = undefined,
    rigidBodiesRotation: ArrayList(f32) = undefined,
    rigidBodiesRotationalVelocity: ArrayList(f32) = undefined,

    allocator: Allocator,

    pub fn init(allocator: Allocator) !RigidBodyContainer {
        const initialCapacity = 100;

        var c = RigidBodyContainer{
            .allocator = allocator,
            .rigidBodiesCapacity = initialCapacity,
        };

        try c.initRigidBodies();

        return c;
    }

    pub fn deinit(self: *RigidBodyContainer) void {
        self.deinitRigidBodies();
    }

    fn deinitRigidBodies(self: *RigidBodyContainer) void {
        self.rigidBodiesPositionX.deinit();
        self.rigidBodiesPositionY.deinit();
        self.rigidBodiesVelocityX.deinit();
        self.rigidBodiesVelocityY.deinit();
        self.rigidBodiesAccelerationX.deinit();
        self.rigidBodiesAccelerationY.deinit();
        self.rigidBodiesRotation.deinit();
        self.rigidBodiesRotationalVelocity.deinit();
    }

    fn initRigidBodies(self: *RigidBodyContainer) !void {
        self.rigidBodiesPositionX = try ArrayList(f32).initCapacity(self.allocator, self.rigidBodiesCapacity);
        self.rigidBodiesPositionY = try ArrayList(f32).initCapacity(self.allocator, self.rigidBodiesCapacity);
        self.rigidBodiesVelocityX = try ArrayList(f32).initCapacity(self.allocator, self.rigidBodiesCapacity);
        self.rigidBodiesVelocityY = try ArrayList(f32).initCapacity(self.allocator, self.rigidBodiesCapacity);
        self.rigidBodiesAccelerationX = try ArrayList(f32).initCapacity(self.allocator, self.rigidBodiesCapacity);
        self.rigidBodiesAccelerationY = try ArrayList(f32).initCapacity(self.allocator, self.rigidBodiesCapacity);
        self.rigidBodiesRotation = try ArrayList(f32).initCapacity(self.allocator, self.rigidBodiesCapacity);
        self.rigidBodiesRotationalVelocity = try ArrayList(f32).initCapacity(self.allocator, self.rigidBodiesCapacity);
    }

    fn ensureAndExpandRigidBodyCapacity(self: *RigidBodyContainer) void {
        const capacity = self.rigidBodiesCapacity;

        self.rigidBodiesPositionX.ensureTotalCapacity(capacity);
        self.rigidBodiesPositionY.ensureTotalCapacity(capacity);
        self.rigidBodiesVelocityX.ensureTotalCapacity(capacity);
        self.rigidBodiesVelocityY.ensureTotalCapacity(capacity);
        self.rigidBodiesAccelerationX.ensureTotalCapacity(capacity);
        self.rigidBodiesAccelerationY.ensureTotalCapacity(capacity);
        self.rigidBodiesRotation.ensureTotalCapacity(capacity);
        self.rigidBodiesRotationalVelocity.ensureTotalCapacity(capacity);

        self.rigidBodiesPositionX.expandToCapacity();
        self.rigidBodiesPositionY.expandToCapacity();
        self.rigidBodiesVelocityX.expandToCapacity();
        self.rigidBodiesVelocityY.expandToCapacity();
        self.rigidBodiesAccelerationX.expandToCapacity();
        self.rigidBodiesAccelerationY.expandToCapacity();
        self.rigidBodiesRotation.expandToCapacity();
        self.rigidBodiesRotationalVelocity.expandToCapacity();
    }

    fn setRigidBodyCapacity(self: *RigidBodyContainer, capacity: usize) !void {
        const oldCapacity = self.rigidBodiesCapacity;
        self.rigidBodiesCapacity = capacity;

        if (capacity <= self.rigidBodiesCapacity) {
            return;
        }

        self.ensureAndExpandRigidBodyCapacity();

        for (oldCapacity..capacity) |i| {
            self.zeroRigidBody(i);
        }
    }

    fn resetRigidBodies(self: *RigidBodyContainer) void {
        self.rigidBodiesCapacity = 100;
        self.rigidBodiesPositionX.shrinkAndFree(self.rigidBodiesCapacity);
        self.rigidBodiesPositionY.shrinkAndFree(self.rigidBodiesCapacity);
        self.rigidBodiesVelocityX.shrinkAndFree(self.rigidBodiesCapacity);
        self.rigidBodiesVelocityY.shrinkAndFree(self.rigidBodiesCapacity);
        self.rigidBodiesAccelerationX.shrinkAndFree(self.rigidBodiesCapacity);
        self.rigidBodiesAccelerationY.shrinkAndFree(self.rigidBodiesCapacity);
        self.rigidBodiesRotation.shrinkAndFree(self.rigidBodiesCapacity);
        self.rigidBodiesRotationalVelocity.shrinkAndFree(self.rigidBodiesCapacity);

        self.ensureAndExpandRigidBodyCapacity();
    }

    pub fn reset(self: *RigidBodyContainer) void {
        self.resetRigidBodies();

        for (0..self.rigidBodiesStaticParams.len) |i| {
            self.rigidBodiesStaticParams[i] = RigidBodyStaticParams.Undefined;
        }

        for (0..self.rigidBodiesCapacity) |i| {
            self.zeroRigidBody(i);
        }
    }

    fn zeroRigidBody(self: *RigidBodyContainer, i: usize) void {
        self.rigidBodiesPositionX.items[i] = 0;
        self.rigidBodiesPositionY.items[i] = 0;
        self.rigidBodiesVelocityX.items[i] = 0;
        self.rigidBodiesVelocityY.items[i] = 0;
        self.rigidBodiesAccelerationX.items[i] = 0;
        self.rigidBodiesAccelerationY.items[i] = 0;
        self.rigidBodiesRotation.items[i] = 0;
        self.rigidBodiesRotationalVelocity.items[i] = 0;
    }

    pub fn removeRigidBody(self: *RigidBodyContainer, i: usize) void {
        self.zeroRigidBody(i);
    }

    fn updatePositions(self: *RigidBodyContainer, dt: f32) void {
        for (0..self.rigidBodiesCapacity) |i| {
            self.rigidBodiesVelocityX.items[i] += self.rigidBodiesAccelerationX.items[i] * dt;
            self.rigidBodiesVelocityY.items[i] += self.rigidBodiesAccelerationY.items[i] * dt;

            self.rigidBodiesPositionX.items[i] += self.rigidBodiesVelocityX.items[i] * dt;
            self.rigidBodiesPositionY.items[i] += self.rigidBodiesVelocityY.items[i] * dt;
        }
    }
};
