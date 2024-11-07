const std = @import("std");
const assert = std.debug.assert;
const zlm = @import("zlm");
const ztracy = @import("ztracy");

const RigidBody = @import("../rigid-body-flat.zig").RigidBodyFlat;
const CollisionResult = @import("result.zig").CollisionResult;
const AABB = @import("../shape.zig").AABB;

pub fn checkCollision(a: AABB, b: AABB, depth: *f32, axis: *zlm.Vec2) bool {
    const zone = ztracy.ZoneNC(@src(), "aabb check collision", 0x00_ff_ff_00);
    defer zone.End();

    const maxL = @max(a.tl.x, b.tl.x);
    const minR = @min(a.br.x, b.br.x);

    const maxT = @max(a.tl.y, b.tl.y);
    const minB = @min(a.br.y, b.br.y);

    const depthX = minR - maxL;
    const depthY = minB - maxT;

    depth.* = @min(depthX, depthY);

    if (depth.* <= 0) {
        return false;
    }

    if (depthX < depthY) {
        axis.* = zlm.vec2(std.math.sign(a.br.x - b.tl.x), 0);
        return true;
    } else if (depthY < depthX) {
        axis.* = zlm.vec2(0, std.math.sign(a.br.y - b.tl.y));
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
    var axis: zlm.Vec2 = undefined;

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
