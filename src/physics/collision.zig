const std = @import("std");
const zlm = @import("zlm");

const Shape = @import("shape.zig").Shape;
const RigidBodyDynamicParams = @import("rigid-body-dynamic.zig").RigidBodyDynamicParams;
const RigidBody = @import("rigid-body-flat.zig").RigidBodyFlat;

const CollisionResult = @import("collision/result.zig").CollisionResult;
const Collision = @import("collision/result.zig").Collision;

const SAT = @import("collision/sat.zig");
const AABB = @import("collision/aabb.zig");

pub fn checkCollision(bodyA: *RigidBody, bodyB: *RigidBody) CollisionResult {
    if (bodyA.*.s.shape == .rectangle and bodyB.*.s.shape == .rectangle and bodyA.*.d.r.* == 0 and bodyB.*.d.r.* == 0) {
        return AABB.checkCollisionBodies(bodyA, bodyB);
    } else {
        return SAT.checkCollisionShapes(bodyA, bodyB);
    }
}

pub fn resolveCollision(collision: Collision) void {
    const normal = collision.normal;
    const depth = collision.depth;
    const bodyA = collision.bodyA;
    const bodyB = collision.bodyB;

    const vA = bodyA.*.d.cloneVel();
    const vB = bodyB.*.d.cloneVel();

    const pA = bodyA.*.d.clonePos();
    const pB = bodyB.*.d.clonePos();

    const massA = bodyA.*.s.mass();
    const massB = bodyB.*.s.mass();

    const e = @min(bodyA.*.s.restitution, bodyB.*.s.restitution);
    const relV = vB.sub(vA);

    const j = relV.scale(-(1 + e)).dot(normal) / ((1 / massA) + (1 / massB));

    if (!bodyA.s.isStatic) {
        const resolution = vA.sub(normal.scale(j / massA));
        bodyA.*.d.setVel(resolution);
    }
    if (!bodyB.s.isStatic) {
        const resolution = vB.add(normal.scale(j / massA));
        bodyB.*.d.setVel(resolution);
    }

    if (bodyA.s.isStatic) {
        bodyB.*.d.setPos(pB.add(normal.scale(depth)));
    } else if (bodyB.s.isStatic) {
        bodyA.*.d.setPos(pA.sub(normal.scale(depth)));
    } else {
        bodyA.*.d.setPos(pA.sub(normal.scale(depth / 2)));
        bodyB.*.d.setPos(pB.add(normal.scale(depth / 2)));
    }
}
