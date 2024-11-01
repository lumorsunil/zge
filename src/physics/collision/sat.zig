const std = @import("std");
const zlm = @import("zlm");
const ztracy = @import("ztracy");

const RigidBody = @import("../rigid-body-flat.zig").RigidBodyFlat;
const CollisionResult = @import("result.zig").CollisionResult;

const MinMaxProjections = struct {
    maxA: f32,
    minA: f32,
    maxB: f32,
    minB: f32,

    pub fn overlaps(self: MinMaxProjections) bool {
        return self.maxB > self.minA and self.maxA > self.minB;
    }

    pub fn overlap(self: MinMaxProjections) f32 {
        return @min(self.maxB - self.minA, self.maxA - self.minB);
    }
};

pub fn checkCollisionShapes(bodyA: *RigidBody, bodyB: *RigidBody) CollisionResult {
    const zone = ztracy.ZoneNC(@src(), "SAT check collision", 0x00_ff_ff_00);
    defer zone.End();

    var minDepth = std.math.inf(f32);
    var minAxis = zlm.vec2(std.math.inf(f32), std.math.inf(f32));

    if (!testCollision(bodyA, bodyB, &minDepth, &minAxis)) {
        return .noCollision;
    }

    if (!testCollision(bodyB, bodyA, &minDepth, &minAxis)) {
        return .noCollision;
    }

    const direction = bodyB.d.clonePos().sub(bodyA.d.clonePos());

    if (direction.dot(minAxis) < 0) {
        minAxis = minAxis.neg();
    }

    return CollisionResult{ .collision = .{
        .bodyA = bodyA,
        .bodyB = bodyB,
        .depth = minDepth,
        .normal = minAxis,
        .contact1 = undefined,
        .contact2 = undefined,
        .contactCount = undefined,
    } };
}

fn project(axis: zlm.Vec2, bodyA: *const RigidBody, bodyB: *const RigidBody) MinMaxProjections {
    var minA = std.math.inf(f32);
    var maxA = -std.math.inf(f32);
    var minB = std.math.inf(f32);
    var maxB = -std.math.inf(f32);

    projectShape(axis, bodyA, &maxA, &minA);
    projectShape(axis, bodyB, &maxB, &minB);

    return MinMaxProjections{ .maxA = maxA, .minA = minA, .maxB = maxB, .minB = minB };
}

fn projectShape(axis: zlm.Vec2, body: *const RigidBody, max: *f32, min: *f32) void {
    const v = body.transformedVertices();

    switch (body.s.shape) {
        .circle => |circle| {
            std.debug.assert(v.len == 1);
            const center = v[0];
            projectCircle(axis, center, circle.radius, max, min);
        },
        .rectangle => |_| {
            projectPolygon(axis, v, max, min);
        },
    }
}

fn projectPolygon(axis: zlm.Vec2, v: []const zlm.Vec2, max: *f32, min: *f32) void {
    for (0..v.len) |i| {
        const proj = v[i].dot(axis);

        max.* = @max(max.*, proj);
        min.* = @min(min.*, proj);
    }
}

fn projectCircle(axis: zlm.Vec2, center: zlm.Vec2, radius: f32, max: *f32, min: *f32) void {
    const direction = axis.normalize();
    const directionAndRadius = direction.scale(radius);

    const v0 = center.add(directionAndRadius);
    const v1 = center.sub(directionAndRadius);

    const proj0 = v0.dot(axis);
    const proj1 = v1.dot(axis);

    min.* = @min(proj0, proj1);
    max.* = @max(proj0, proj1);
}

const MAX_AXIS = 4;
fn getTestAxis(bodyA: *const RigidBody, bodyB: *const RigidBody, buffer: *[MAX_AXIS]zlm.Vec2) []zlm.Vec2 {
    const v = bodyA.transformedVertices();
    const other = bodyB.transformedVertices();

    return switch (bodyA.s.shape) {
        .circle => getTestAxisCircle(v, other, buffer),
        .rectangle => getTestAxisPolygon(v, buffer),
    };
}

fn getTestAxisCircle(v: []const zlm.Vec2, other: []const zlm.Vec2, buffer: *[MAX_AXIS]zlm.Vec2) []zlm.Vec2 {
    std.debug.assert(v.len == 1);

    const c = v[0];
    var minDistance = std.math.inf(f32);
    var minDistanceVertex: zlm.Vec2 = zlm.Vec2.zero;

    for (0..other.len) |i| {
        const distance = other[i].distance2(c);

        if (distance < minDistance) {
            minDistance = distance;
            minDistanceVertex = other[i];
        }
    }

    const axis = minDistanceVertex.sub(c).normalize();
    buffer[0] = axis;

    return buffer[0..1];
}

fn getTestAxisPolygon(v: []const zlm.Vec2, buffer: *[MAX_AXIS]zlm.Vec2) []zlm.Vec2 {
    for (0..v.len) |i| {
        const v0 = v[i];
        const v1 = v[@mod(i + 1, v.len)];

        const edge = v1.sub(v0);
        const axis = zlm.vec2(-edge.y, edge.x).normalize();

        buffer[i] = axis;
    }

    return buffer[0..v.len];
}

fn testCollision(bodyA: *const RigidBody, bodyB: *const RigidBody, minDepth: *f32, minAxis: *zlm.Vec2) bool {
    var axisBuffer: [MAX_AXIS]zlm.Vec2 = undefined;

    const axii = getTestAxis(bodyA, bodyB, &axisBuffer);

    for (axii) |axis| {
        const projections = project(axis, bodyA, bodyB);

        if (!projections.overlaps()) {
            return false;
        }

        const depth = projections.overlap();

        if (depth < minDepth.*) {
            minDepth.* = depth;
            minAxis.* = axis;
        }
    }

    return true;
}
