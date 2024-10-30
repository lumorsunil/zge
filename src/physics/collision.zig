const std = @import("std");
const zlm = @import("zlm");

const Shape = @import("shape.zig").Shape;
const RigidBodyDynamicParams = @import("rigid-body-dynamic.zig").RigidBodyDynamicParams;

const transformVertices = @import("transform.zig").transformVertices;

pub const CollisionResult = union(enum) {
    collision: Collision,
    noCollision,

    pub const Collision = struct {
        normal: zlm.Vec2,
        depth: f32,
    };
};

pub fn checkCollision(shapeA: Shape, shapeB: Shape, dynamicA: RigidBodyDynamicParams, dynamicB: RigidBodyDynamicParams) CollisionResult {
    return checkCollisionShapes(shapeA, shapeB, dynamicA, dynamicB);
}

fn checkCollisionCircles(circleA: Shape.Circle, circleB: Shape.Circle, dynamicA: RigidBodyDynamicParams, dynamicB: RigidBodyDynamicParams) CollisionResult {
    const d2 = dynamicA.p.distance2(dynamicB.p);
    const r2 = (circleA.radius + circleB.radius) * (circleA.radius + circleB.radius);

    if (d2 >= r2) return .noCollision;

    return .{ .collision = .{
        .depth = std.math.sqrt(r2) - std.math.sqrt(d2),
        .normal = dynamicB.p.sub(dynamicA.p).normalize(),
    } };
}

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

fn SATproject(axis: zlm.Vec2, shapeA: Shape, shapeB: Shape, vA: []const zlm.Vec2, vB: []const zlm.Vec2) MinMaxProjections {
    var minA = std.math.inf(f32);
    var maxA = -std.math.inf(f32);
    var minB = std.math.inf(f32);
    var maxB = -std.math.inf(f32);

    SATprojectShape(axis, shapeA, vA, &maxA, &minA);
    SATprojectShape(axis, shapeB, vB, &maxB, &minB);

    return MinMaxProjections{ .maxA = maxA, .minA = minA, .maxB = maxB, .minB = minB };
}

fn SATprojectShape(axis: zlm.Vec2, shape: Shape, v: []const zlm.Vec2, max: *f32, min: *f32) void {
    switch (shape) {
        .circle => |circle| {
            std.debug.assert(v.len == 1);
            const center = v[0];
            SATprojectCircle(axis, center, circle.radius, max, min);
        },
        .rectangle => |_| {
            SATprojectPolygon(axis, v, max, min);
        },
    }
}

fn SATprojectPolygon(axis: zlm.Vec2, v: []const zlm.Vec2, max: *f32, min: *f32) void {
    for (0..v.len) |i| {
        const proj = v[i].dot(axis);

        max.* = @max(max.*, proj);
        min.* = @min(min.*, proj);
    }
}

fn SATprojectCircle(axis: zlm.Vec2, center: zlm.Vec2, radius: f32, max: *f32, min: *f32) void {
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
fn SATgetTestAxis(shape: Shape, v: []const zlm.Vec2, other: []const zlm.Vec2, buffer: *[MAX_AXIS]zlm.Vec2) []zlm.Vec2 {
    return switch (shape) {
        .circle => SATgetTestAxisCircle(v, other, buffer),
        .rectangle => SATgetTestAxisPolygon(v, buffer),
    };
}

fn SATgetTestAxisCircle(v: []const zlm.Vec2, other: []const zlm.Vec2, buffer: *[MAX_AXIS]zlm.Vec2) []zlm.Vec2 {
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

    const axis = minDistanceVertex.sub(c);
    buffer[0] = axis;

    return buffer[0..1];
}

fn SATgetTestAxisPolygon(v: []const zlm.Vec2, buffer: *[MAX_AXIS]zlm.Vec2) []zlm.Vec2 {
    for (0..v.len) |i| {
        const v0 = v[i];
        const v1 = v[@mod(i + 1, v.len)];

        const edge = v1.sub(v0);
        const axis = zlm.vec2(-edge.y, edge.x);

        buffer[i] = axis;
    }

    return buffer[0..v.len];
}

fn SATtestCollision(shapeA: Shape, shapeB: Shape, vA: []const zlm.Vec2, vB: []const zlm.Vec2, minDepth: *f32, minAxis: *zlm.Vec2) bool {
    var axisBuffer: [MAX_AXIS]zlm.Vec2 = undefined;

    const axii = SATgetTestAxis(shapeA, vA, vB, &axisBuffer);

    for (axii) |axis| {
        const projections = SATproject(axis, shapeA, shapeB, vA, vB);

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

fn checkCollisionShapes(shapeA: Shape, shapeB: Shape, dynamicA: RigidBodyDynamicParams, dynamicB: RigidBodyDynamicParams) CollisionResult {
    var minDepth = std.math.inf(f32);
    var minAxis = zlm.vec2(std.math.inf(f32), std.math.inf(f32));

    const pA = dynamicA.p;
    const pB = dynamicB.p;
    const rA = dynamicA.r;
    const rB = dynamicB.r;

    var tvABuffer: [4]zlm.Vec2 = undefined;
    const tvA = transformVertices(shapeA, pA, rA, &tvABuffer);

    var tvBBuffer: [4]zlm.Vec2 = undefined;
    const tvB = transformVertices(shapeB, pB, rB, &tvBBuffer);

    if (!SATtestCollision(shapeA, shapeB, tvA, tvB, &minDepth, &minAxis)) {
        return .noCollision;
    }

    if (!SATtestCollision(shapeB, shapeA, tvB, tvA, &minDepth, &minAxis)) {
        return .noCollision;
    }

    const direction = pB.sub(pA);

    if (direction.dot(minAxis) < 0) {
        minAxis = minAxis.neg();
    }

    return CollisionResult{ .collision = .{
        .depth = minDepth / minAxis.length(),
        .normal = minAxis.normalize(),
    } };
}

pub fn resolveCollision(shapeA: Shape, shapeB: Shape, pA: *zlm.Vec2, pB: *zlm.Vec2, collision: CollisionResult.Collision) void {
    _ = shapeA;
    _ = shapeB;

    const normal = collision.normal;
    const depth = collision.depth;

    pA.* = pA.add(normal.scale(depth / 2).neg());
    pB.* = pB.add(normal.scale(depth / 2));
}
