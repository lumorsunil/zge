const std = @import("std");
const assert = std.debug.assert;
const ztracy = @import("ztracy");

const V = @import("../../vector.zig").V;
const Vector = @import("../../vector.zig").Vector;

const RigidBody = @import("../rigid-body-flat.zig").RigidBodyFlat;
const CollisionResult = @import("result.zig").CollisionResult;
const AABB = @import("../shape.zig").AABB;

pub fn checkCollision(a: AABB, b: AABB, depth: *f32, axis: *Vector) bool {
    const zone = ztracy.ZoneNC(@src(), "aabb check collision", 0x00_ff_ff_00);
    defer zone.End();

    const maxTl = @max(a.tl, b.tl);
    const minBr = @min(a.br, b.br);

    const depthV = minBr - maxTl;

    depth.* = V.min(depthV);

    if (depth.* <= 0) {
        return false;
    }

    const d = a.br - b.tl;

    if (V.x(depthV) < V.y(depthV)) {
        axis.* = V.onlyX(std.math.sign(V.x(d)));
        return true;
    } else if (V.y(depthV) < V.x(depthV)) {
        axis.* = V.onlyY(std.math.sign(V.y(d)));
        return true;
    }

    return false;
}

pub fn checkCollisionBodies(bodyA: *RigidBody, bodyB: *RigidBody) CollisionResult {
    assert(bodyA.*.s.shape == .rectangle);
    assert(bodyB.*.s.shape == .rectangle);
    assert(bodyA.*.d.r.* == 0);
    assert(bodyB.*.d.r.* == 0);

    var depth: f32 = undefined;
    var axis: Vector = undefined;

    if (!checkCollision(bodyA.aabb, bodyB.aabb, &depth, &axis)) {
        return .noCollision;
    }

    return CollisionResult{ .collision = .{
        .bodyA = bodyA,
        .bodyB = bodyB,
        .depth = depth,
        .normal = axis,
        .contact1 = undefined,
        .contact2 = undefined,
        .contactCount = undefined,
    } };
}
