const std = @import("std");
// const ztracy = @import("ztracy");

const V = @import("../../vector.zig").V;
const Vector = @import("../../vector.zig").Vector;

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
    // const zone = ztracy.ZoneNC(@src(), "SAT check collision", 0x00_ff_ff_00);
    // defer zone.End();

    var minDepth = std.math.inf(f32);
    var minAxis = V.init(std.math.inf(f32), std.math.inf(f32));

    if (!testCollision(bodyA, bodyB, &minDepth, &minAxis)) {
        return .noCollision;
    }

    if (!testCollision(bodyB, bodyA, &minDepth, &minAxis)) {
        return .noCollision;
    }

    const direction = bodyB.d.clonePos() - bodyA.d.clonePos();

    if (V.dot(direction, minAxis) < 0) {
        minAxis = -minAxis;
    }

    return CollisionResult{ .collision = .{
        .bodyA = bodyA,
        .bodyB = bodyB,
        .depth = minDepth,
        .normal = minAxis,
        .energyTransferred = undefined,
        .contact1 = undefined,
        .contact2 = undefined,
        .contactCount = undefined,
    } };
}

fn project(axis: Vector, bodyA: *const RigidBody, bodyB: *const RigidBody) MinMaxProjections {
    var minA = std.math.inf(f32);
    var maxA = -std.math.inf(f32);
    var minB = std.math.inf(f32);
    var maxB = -std.math.inf(f32);

    projectShape(axis, bodyA, &maxA, &minA);
    projectShape(axis, bodyB, &maxB, &minB);

    return MinMaxProjections{ .maxA = maxA, .minA = minA, .maxB = maxB, .minB = minB };
}

fn projectShape(axis: Vector, body: *const RigidBody, max: *f32, min: *f32) void {
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

fn projectPolygon(axis: Vector, v: []const Vector, max: *f32, min: *f32) void {
    for (0..v.len) |i| {
        const proj = V.dot(v[i], axis);

        max.* = @max(max.*, proj);
        min.* = @min(min.*, proj);
    }
}

fn projectCircle(axis: Vector, center: Vector, radius: f32, max: *f32, min: *f32) void {
    const direction = V.normalize(axis);
    const directionAndRadius = direction * V.scalar(radius);

    const v0 = center + directionAndRadius;
    const v1 = center - directionAndRadius;

    const proj0 = V.dot(v0, axis);
    const proj1 = V.dot(v1, axis);

    min.* = @min(proj0, proj1);
    max.* = @max(proj0, proj1);
}

const MAX_AXIS = 4;
fn getTestAxis(bodyA: *const RigidBody, bodyB: *const RigidBody, buffer: *[MAX_AXIS]Vector) []Vector {
    const v = bodyA.transformedVertices();
    const other = bodyB.transformedVertices();

    return switch (bodyA.s.shape) {
        .circle => getTestAxisCircle(v, other, buffer),
        .rectangle => getTestAxisPolygon(v, buffer),
    };
}

fn getTestAxisCircle(v: []const Vector, other: []const Vector, buffer: *[MAX_AXIS]Vector) []Vector {
    std.debug.assert(v.len == 1);

    const c = v[0];
    var minDistance = std.math.inf(f32);
    var minDistanceVertex: Vector = V.zero;

    for (0..other.len) |i| {
        const distance = V.distance2(other[i], c);

        if (distance < minDistance) {
            minDistance = distance;
            minDistanceVertex = other[i];
        }
    }

    const axis = V.normalize(minDistanceVertex - c);
    buffer[0] = axis;

    return buffer[0..1];
}

fn getTestAxisPolygon(v: []const Vector, buffer: *[MAX_AXIS]Vector) []Vector {
    for (0..v.len) |i| {
        const v0 = v[i];
        const v1 = v[@mod(i + 1, v.len)];

        const edge = v1 - v0;
        const axis = V.normalize(V.init(-V.y(edge), V.x(edge)));

        buffer[i] = axis;
    }

    return buffer[0..v.len];
}

fn testCollision(bodyA: *const RigidBody, bodyB: *const RigidBody, minDepth: *f32, minAxis: *Vector) bool {
    var axisBuffer: [MAX_AXIS]Vector = undefined;

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
