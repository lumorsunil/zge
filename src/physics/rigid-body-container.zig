const std = @import("std");
const zlm = @import("zlm");
const pzlm = zlm.SpecializeOn(*f32);
const Allocator = std.mem.Allocator;
const ArrayList = std.ArrayList;

const cfg = @import("../config.zig");

const VectorArrayList = @import("vector-array-list.zig").VectorArrayList;
const Vector = @import("vector-array-list.zig").Vector;

const RigidBodyStaticParams = @import("rigid-body-static.zig").RigidBodyStaticParams;
const RigidBodyDynamicParams = @import("rigid-body-dynamic.zig").RigidBodyDynamicParams;

const INITIAL_CAPACITY = 100;

pub const RigidBodyContainer = struct {
    rigidBodiesCapacity: usize,

    rigidBodiesDynamic: VectorArrayList = undefined,
    rigidBodiesPositionX: VectorArrayList = undefined,
    rigidBodiesPositionY: VectorArrayList = undefined,
    rigidBodiesVelocityX: VectorArrayList = undefined,
    rigidBodiesVelocityY: VectorArrayList = undefined,
    rigidBodiesAccelerationX: VectorArrayList = undefined,
    rigidBodiesAccelerationY: VectorArrayList = undefined,
    rigidBodiesRotation: VectorArrayList = undefined,
    rigidBodiesRotationalVelocity: VectorArrayList = undefined,
    rigidBodiesRotationalAcceleration: VectorArrayList = undefined,

    allocator: Allocator,

    pub fn init(allocator: Allocator) RigidBodyContainer {
        var c = RigidBodyContainer{
            .allocator = allocator,
            .rigidBodiesCapacity = INITIAL_CAPACITY,
        };

        c.initRigidBodies();

        return c;
    }

    pub fn deinit(self: *const RigidBodyContainer) void {
        self.deinitRigidBodies();
    }

    fn deinitRigidBodies(self: *const RigidBodyContainer) void {
        self.rigidBodiesDynamic.deinit();
        self.rigidBodiesPositionX.deinit();
        self.rigidBodiesPositionY.deinit();
        self.rigidBodiesVelocityX.deinit();
        self.rigidBodiesVelocityY.deinit();
        self.rigidBodiesAccelerationX.deinit();
        self.rigidBodiesAccelerationY.deinit();
        self.rigidBodiesRotation.deinit();
        self.rigidBodiesRotationalVelocity.deinit();
        self.rigidBodiesRotationalAcceleration.deinit();
    }

    fn initRigidBodies(self: *RigidBodyContainer) void {
        self.rigidBodiesDynamic = VectorArrayList.initCapacity(self.allocator, self.rigidBodiesCapacity);
        self.rigidBodiesPositionX = VectorArrayList.initCapacity(self.allocator, self.rigidBodiesCapacity);
        self.rigidBodiesPositionY = VectorArrayList.initCapacity(self.allocator, self.rigidBodiesCapacity);
        self.rigidBodiesVelocityX = VectorArrayList.initCapacity(self.allocator, self.rigidBodiesCapacity);
        self.rigidBodiesVelocityY = VectorArrayList.initCapacity(self.allocator, self.rigidBodiesCapacity);
        self.rigidBodiesAccelerationX = VectorArrayList.initCapacity(self.allocator, self.rigidBodiesCapacity);
        self.rigidBodiesAccelerationY = VectorArrayList.initCapacity(self.allocator, self.rigidBodiesCapacity);
        self.rigidBodiesRotation = VectorArrayList.initCapacity(self.allocator, self.rigidBodiesCapacity);
        self.rigidBodiesRotationalVelocity = VectorArrayList.initCapacity(self.allocator, self.rigidBodiesCapacity);
        self.rigidBodiesRotationalAcceleration = VectorArrayList.initCapacity(self.allocator, self.rigidBodiesCapacity);
    }

    fn ensureAndExpandRigidBodyCapacity(self: *RigidBodyContainer) void {
        const capacity = self.rigidBodiesCapacity;

        self.rigidBodiesDynamic.ensureTotalCapacity(capacity);
        self.rigidBodiesPositionX.ensureTotalCapacity(capacity);
        self.rigidBodiesPositionY.ensureTotalCapacity(capacity);
        self.rigidBodiesVelocityX.ensureTotalCapacity(capacity);
        self.rigidBodiesVelocityY.ensureTotalCapacity(capacity);
        self.rigidBodiesAccelerationX.ensureTotalCapacity(capacity);
        self.rigidBodiesAccelerationY.ensureTotalCapacity(capacity);
        self.rigidBodiesRotation.ensureTotalCapacity(capacity);
        self.rigidBodiesRotationalVelocity.ensureTotalCapacity(capacity);
        self.rigidBodiesRotationalAcceleration.ensureTotalCapacity(capacity);

        self.rigidBodiesDynamic.expandToCapacity();
        self.rigidBodiesPositionX.expandToCapacity();
        self.rigidBodiesPositionY.expandToCapacity();
        self.rigidBodiesVelocityX.expandToCapacity();
        self.rigidBodiesVelocityY.expandToCapacity();
        self.rigidBodiesAccelerationX.expandToCapacity();
        self.rigidBodiesAccelerationY.expandToCapacity();
        self.rigidBodiesRotation.expandToCapacity();
        self.rigidBodiesRotationalVelocity.expandToCapacity();
        self.rigidBodiesRotationalAcceleration.expandToCapacity();
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
        self.rigidBodiesCapacity = INITIAL_CAPACITY;
        self.rigidBodiesDynamic.shrinkAndFree(self.rigidBodiesCapacity);
        self.rigidBodiesPositionX.shrinkAndFree(self.rigidBodiesCapacity);
        self.rigidBodiesPositionY.shrinkAndFree(self.rigidBodiesCapacity);
        self.rigidBodiesVelocityX.shrinkAndFree(self.rigidBodiesCapacity);
        self.rigidBodiesVelocityY.shrinkAndFree(self.rigidBodiesCapacity);
        self.rigidBodiesAccelerationX.shrinkAndFree(self.rigidBodiesCapacity);
        self.rigidBodiesAccelerationY.shrinkAndFree(self.rigidBodiesCapacity);
        self.rigidBodiesRotation.shrinkAndFree(self.rigidBodiesCapacity);
        self.rigidBodiesRotationalVelocity.shrinkAndFree(self.rigidBodiesCapacity);
        self.rigidBodiesRotationalAcceleration.shrinkAndFree(self.rigidBodiesCapacity);

        self.ensureAndExpandRigidBodyCapacity();
    }

    pub fn reset(self: *RigidBodyContainer) void {
        self.resetRigidBodies();

        for (0..self.rigidBodiesCapacity) |i| {
            self.zeroRigidBody(i);
        }
    }

    fn zeroRigidBody(self: *RigidBodyContainer, i: usize) void {
        self.rigidBodiesDynamic.set(i, 0);
        self.rigidBodiesPositionX.set(i, 0);
        self.rigidBodiesPositionY.set(i, 0);
        self.rigidBodiesVelocityX.set(i, 0);
        self.rigidBodiesVelocityY.set(i, 0);
        self.rigidBodiesAccelerationX.set(i, 0);
        self.rigidBodiesAccelerationY.set(i, 0);
        self.rigidBodiesRotation.set(i, 0);
        self.rigidBodiesRotationalVelocity.set(i, 0);
        self.rigidBodiesRotationalAcceleration.set(i, 0);
    }

    pub fn setRigidBody(self: *RigidBodyContainer, i: usize, pos: zlm.Vec2, vel: zlm.Vec2, acc: zlm.Vec2, r: f32, rv: f32, ra: f32, isStatic: bool, isPointersInvalidated: ?*bool) void {
        self.rigidBodiesDynamic.set(i, if (isStatic) 0 else 1, isPointersInvalidated);
        self.rigidBodiesPositionX.set(i, pos.x, isPointersInvalidated);
        self.rigidBodiesPositionY.set(i, pos.y, isPointersInvalidated);
        self.rigidBodiesVelocityX.set(i, vel.x, isPointersInvalidated);
        self.rigidBodiesVelocityY.set(i, vel.y, isPointersInvalidated);
        self.rigidBodiesAccelerationX.set(i, acc.x, isPointersInvalidated);
        self.rigidBodiesAccelerationY.set(i, acc.y, isPointersInvalidated);
        self.rigidBodiesRotation.set(i, r, isPointersInvalidated);
        self.rigidBodiesRotationalVelocity.set(i, rv, isPointersInvalidated);
        self.rigidBodiesRotationalAcceleration.set(i, ra, isPointersInvalidated);
    }

    pub fn removeRigidBody(self: *RigidBodyContainer, i: usize) void {
        _ = i; // autofix
        _ = self; // autofix
        //self.zeroRigidBody(i);
    }

    /// Slow operation
    pub fn getPositionP(self: *RigidBodyContainer, i: usize) pzlm.Vec2 {
        return pzlm.vec2(self.rigidBodiesPositionX.getP(i), self.rigidBodiesPositionY.getP(i));
    }

    /// Slow operation
    pub fn getVelocityP(self: *RigidBodyContainer, i: usize) pzlm.Vec2 {
        return pzlm.vec2(self.rigidBodiesVelocityX.getP(i), self.rigidBodiesVelocityY.getP(i));
    }

    /// Slow operation
    pub fn getAccelerationP(self: *RigidBodyContainer, i: usize) pzlm.Vec2 {
        return pzlm.vec2(self.rigidBodiesAccelerationX.getP(i), self.rigidBodiesAccelerationY.getP(i));
    }

    /// Slow operation
    pub fn getRotationP(self: *RigidBodyContainer, i: usize) *f32 {
        return self.rigidBodiesRotation.getP(i);
    }

    /// Slow operation
    pub fn getRotationalVelocityP(self: *RigidBodyContainer, i: usize) *f32 {
        return self.rigidBodiesRotationalVelocity.getP(i);
    }

    /// Slow operation
    pub fn getRotationalAccelerationP(self: *RigidBodyContainer, i: usize) *f32 {
        return self.rigidBodiesRotationalAcceleration.getP(i);
    }

    /// Slow operation
    pub fn getPosition(self: *RigidBodyContainer, i: usize) zlm.Vec2 {
        return zlm.vec2(self.rigidBodiesPositionX.get(i), self.rigidBodiesPositionY.get(i));
    }

    /// Slow operation
    pub fn getVelocity(self: *RigidBodyContainer, i: usize) zlm.Vec2 {
        return zlm.vec2(self.rigidBodiesVelocityX.get(i), self.rigidBodiesVelocityY.get(i));
    }

    /// Slow operation
    pub fn getAcceleration(self: *RigidBodyContainer, i: usize) zlm.Vec2 {
        return zlm.vec2(self.rigidBodiesAccelerationX.get(i), self.rigidBodiesAccelerationY.get(i));
    }

    /// Slow operation
    pub fn getRotation(self: *RigidBodyContainer, i: usize) f32 {
        return zlm.vec2(self.rigidBodiesRotation.get(i));
    }

    /// Slow operation
    pub fn getRotationalVelocity(self: *RigidBodyContainer, i: usize) f32 {
        return zlm.vec2(self.rigidBodiesRotationalVelocity.get(i));
    }

    /// Slow operation
    pub fn getRotationalAcceleration(self: *RigidBodyContainer, i: usize) f32 {
        return zlm.vec2(self.rigidBodiesRotationalAcceleration.get(i));
    }

    /// Very slow operation
    pub fn getRigidBody(self: *RigidBodyContainer, i: usize) RigidBodyDynamicParams {
        return RigidBodyDynamicParams{
            .p = self.getPositionP(i),
            .v = self.getVelocityP(i),
            .a = self.getAccelerationP(i),
            .r = self.getRotationP(i),
            .rv = self.getRotationalVelocityP(i),
            .ra = self.getRotationalAccelerationP(i),
        };
    }

    /// Very slow operation
    pub fn updateRigidBodyPointers(self: *RigidBodyContainer, i: usize, body: *RigidBodyDynamicParams) void {
        body.p = self.getPositionP(i);
        body.v = self.getVelocityP(i);
        body.a = self.getAccelerationP(i);
        body.r = self.getRotationP(i);
        body.rv = self.getRotationalVelocityP(i);
        body.ra = self.getRotationalAccelerationP(i);
    }

    fn integrateScaledFn(dt: f32, a: *Vector, b: *Vector) void {
        const splat = @as(Vector, @splat(dt));

        a.* += b.* * splat;
    }

    fn integrateScaled(dt: f32, a: *VectorArrayList, b: *VectorArrayList) void {
        a.iterateC(f32, b, dt, integrateScaledFn);
    }

    fn applyGravityFn(gravity: *const Vector, acceleration: *Vector, dynamic: *Vector) void {
        acceleration.* += gravity.* * dynamic.*;
    }

    fn applyGravity(self: *RigidBodyContainer, gravity: zlm.Vec2) void {
        var gravityXVector = @as(Vector, @splat(gravity.x));
        self.rigidBodiesAccelerationX.iterateC(
            *Vector,
            &self.rigidBodiesDynamic,
            &gravityXVector,
            applyGravityFn,
        );
        var gravityYVector = @as(Vector, @splat(gravity.y));
        self.rigidBodiesAccelerationY.iterateC(
            *Vector,
            &self.rigidBodiesDynamic,
            &gravityYVector,
            applyGravityFn,
        );
    }

    pub fn updatePositions(self: *RigidBodyContainer, gravity: zlm.Vec2, dt: f32) void {
        @setRuntimeSafety(false);

        self.applyGravity(gravity);

        integrateScaled(dt, &self.rigidBodiesVelocityX, &self.rigidBodiesAccelerationX);
        integrateScaled(dt, &self.rigidBodiesVelocityY, &self.rigidBodiesAccelerationY);

        integrateScaled(dt, &self.rigidBodiesPositionX, &self.rigidBodiesVelocityX);
        integrateScaled(dt, &self.rigidBodiesPositionY, &self.rigidBodiesVelocityY);

        integrateScaled(dt, &self.rigidBodiesRotationalVelocity, &self.rigidBodiesRotationalAcceleration);
        integrateScaled(dt, &self.rigidBodiesRotation, &self.rigidBodiesRotationalVelocity);

        self.rigidBodiesAccelerationX.setScalar(0);
        self.rigidBodiesAccelerationY.setScalar(0);
    }
};
