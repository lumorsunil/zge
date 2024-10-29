const std = @import("std");
const zlm = @import("zlm");

/// 1cm x 1cm
pub const MIN_AREA: f32 = 0.01 * 0.01;
/// 64m x 64m
pub const MAX_AREA: f32 = 64 * 64;

/// Density of air = 0.001225 g/cm^3
pub const MIN_DENSITY: f32 = 0.001225;
/// Density of osmium = 22.6 g/cm^3
pub const MAX_DENSITY: f32 = 22.6;

pub const MIN_RESTITUTION: f32 = 0;
pub const MAX_RESTITUTION: f32 = 1;

pub const Densities = struct {
    pub const Water = 1;
    pub const Osmium = 22.6;
};

pub fn Result(comptime T: type) type {
    return union(enum) {
        success: T,
        err: []const u8,
    };
}

pub const RigidBodyStaticParams = struct {
    density: f32,
    restitution: f32,

    isStatic: bool,

    shape: Shape,

    pub const Undefined = RigidBodyStaticParams{
        .density = 0,
        .restitution = 0,
        .isStatic = true,
        .shape = .{ .circle = .{ .radius = 0 } },
    };

    pub fn area(self: RigidBodyStaticParams) f32 {
        return self.shape.area();
    }

    pub fn volume(self: RigidBodyStaticParams) f32 {
        return self.shape.area();
    }

    pub fn mass(self: RigidBodyStaticParams) f32 {
        const v = self.volume();
        if (v == 0) return 0;
        const d = self.density;
        return d / v;
    }

    pub const Shape = union(enum) {
        circle: Circle,
        rectangle: Rectangle,

        pub fn area(self: Shape) f32 {
            return switch (self) {
                .circle => |circle| circle.area(),
                .rectangle => |rectangle| rectangle.area(),
            };
        }

        pub const Circle = struct {
            radius: f32,

            pub fn radiusSq(self: Circle) f32 {
                return self.radius * self.radius;
            }

            pub fn area(self: Circle) f32 {
                return self.radius * self.radius * std.math.pi;
            }
        };

        pub const Rectangle = struct {
            size: zlm.Vec2,
            vertices: [4]zlm.Vec2,

            pub fn init(size: zlm.Vec2) Rectangle {
                var rect = Rectangle{
                    .size = size,
                    .vertices = undefined,
                };

                vertices(rect.size, &rect.vertices);

                return rect;
            }

            pub fn width(self: Rectangle) f32 {
                return self.size.x;
            }

            pub fn height(self: Rectangle) f32 {
                return self.size.x;
            }

            pub fn area(self: Rectangle) f32 {
                return self.width() * self.height();
            }

            pub fn vertices(size: zlm.Vec2, buffer: *[4]zlm.Vec2) void {
                const l = -size.x / 2;
                const r = -l;
                const t = -size.y / 2;
                const b = -t;

                buffer[0] = zlm.vec2(l, t);
                buffer[1] = zlm.vec2(r, t);
                buffer[2] = zlm.vec2(r, b);
                buffer[3] = zlm.vec2(l, b);
            }
        };
    };

    pub fn init(shape: Shape, density: f32, restitution: f32, isStatic: bool) Result(RigidBodyStaticParams) {
        const a = shape.area();

        if (a < MIN_AREA) {
            return .{ .err = std.fmt.comptimePrint("Area cannot be less than {d:.4}", .{MIN_AREA}) };
        } else if (a > MAX_AREA) {
            return .{ .err = std.fmt.comptimePrint("Area cannot be greater than {d:.2}", .{MAX_AREA}) };
        }

        if (density < MIN_DENSITY) {
            return .{ .err = std.fmt.comptimePrint("Density cannot be less than {d:.6}", .{MIN_DENSITY}) };
        } else if (density > MAX_DENSITY) {
            return .{ .err = std.fmt.comptimePrint("Density cannot be greater than {d:.1}", .{MAX_DENSITY}) };
        }

        if (restitution < MIN_RESTITUTION) {
            return .{ .err = std.fmt.comptimePrint("Restitution cannot be less than {d:.0}", .{MIN_RESTITUTION}) };
        } else if (restitution > MAX_RESTITUTION) {
            return .{ .err = std.fmt.comptimePrint("Restitution cannot be greater than {d:.0}", .{MAX_RESTITUTION}) };
        }

        return .{ .success = RigidBodyStaticParams{
            .density = density,
            .restitution = restitution,
            .isStatic = isStatic,
            .shape = shape,
        } };
    }
};

pub const RigidBodyDynamicParams = struct {
    p: zlm.Vec2 = zlm.vec2(0, 0),
    v: zlm.Vec2 = zlm.vec2(0, 0),
    a: zlm.Vec2 = zlm.vec2(0, 0),
    r: f32 = 0,
    rv: f32 = 0,
    ra: f32 = 0,
};

pub const RigidBody = struct {
    static: *RigidBodyStaticParams,
    dynamic: *RigidBodyDynamicParams,
};

pub const RigidBodyFlat = struct {
    s: RigidBodyStaticParams,
    d: RigidBodyDynamicParams,

    pub const Shape = RigidBodyStaticParams.Shape;
    pub const Collision = CollisionResult.Collision;

    pub fn init(shape: RigidBodyStaticParams.Shape, density: f32, restitution: f32, isStatic: bool) Result(RigidBodyFlat) {
        const static = switch (RigidBodyStaticParams.init(shape, density, restitution, isStatic)) {
            .err => |err| return .{ .err = err },
            .success => |static| static,
        };

        return .{ .success = RigidBodyFlat{
            .s = static,
            .d = RigidBodyDynamicParams{},
        } };
    }

    pub fn checkCollision(self: RigidBodyFlat, other: RigidBodyFlat) CollisionResult {
        return globalCheckCollision(self.s.shape, other.s.shape, self.d, other.d);
    }

    pub fn resolveCollision(rbA: *RigidBodyFlat, rbB: *RigidBodyFlat, collision: Collision) void {
        return globalResolveCollision(rbA.s.shape, rbB.s.shape, &rbA.d.p, &rbB.d.p, collision);
    }
};

const RBShape = RigidBodyStaticParams.Shape;
fn globalCheckCollision(shapeA: RBShape, shapeB: RBShape, dynamicA: RigidBodyDynamicParams, dynamicB: RigidBodyDynamicParams) CollisionResult {
    return checkCollisionShapes(shapeA, shapeB, dynamicA, dynamicB);
}

pub const CollisionResult = union(enum) {
    collision: Collision,
    noCollision,

    pub const Collision = struct {
        normal: zlm.Vec2,
        depth: f32,
    };
};

fn checkCollisionCircles(circleA: RBShape.Circle, circleB: RBShape.Circle, dynamicA: RigidBodyDynamicParams, dynamicB: RigidBodyDynamicParams) CollisionResult {
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

fn SATproject(axis: zlm.Vec2, shapeA: RBShape, shapeB: RBShape, vA: []const zlm.Vec2, vB: []const zlm.Vec2) MinMaxProjections {
    var minA = std.math.inf(f32);
    var maxA = -std.math.inf(f32);
    var minB = std.math.inf(f32);
    var maxB = -std.math.inf(f32);

    SATprojectShape(axis, shapeA, vA, &maxA, &minA);
    SATprojectShape(axis, shapeB, vB, &maxB, &minB);

    return MinMaxProjections{ .maxA = maxA, .minA = minA, .maxB = maxB, .minB = minB };
}

fn SATprojectShape(axis: zlm.Vec2, shape: RBShape, v: []const zlm.Vec2, max: *f32, min: *f32) void {
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
fn SATgetTestAxis(shape: RBShape, v: []const zlm.Vec2, other: []const zlm.Vec2, buffer: *[MAX_AXIS]zlm.Vec2) []zlm.Vec2 {
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

fn SATtestCollision(shapeA: RBShape, shapeB: RBShape, vA: []const zlm.Vec2, vB: []const zlm.Vec2, minDepth: *f32, minAxis: *zlm.Vec2) bool {
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

fn transformVertices(shape: RBShape, translation: zlm.Vec2, rotation: f32, buffer: []zlm.Vec2) []zlm.Vec2 {
    return switch (shape) {
        .rectangle => |rectangle| transformVerticesRectangle(&rectangle.vertices, translation, rotation, buffer),
        .circle => transformVerticesCircle(translation, buffer),
    };
}

fn transformVertex(v: zlm.Vec2, translation: zlm.Vec2, rotation: f32) zlm.Vec2 {
    return v.rotate(rotation).add(translation);
}

fn transformVerticesRectangle(v: []const zlm.Vec2, translation: zlm.Vec2, rotation: f32, buffer: []zlm.Vec2) []zlm.Vec2 {
    std.debug.assert(v.len <= buffer.len);

    for (0..v.len) |i| {
        buffer[i] = transformVertex(v[i], translation, rotation);
    }

    return buffer[0..v.len];
}

fn transformVerticesCircle(translation: zlm.Vec2, buffer: []zlm.Vec2) []zlm.Vec2 {
    std.debug.assert(buffer.len > 1);

    buffer[0] = translation;

    return buffer[0..1];
}

fn checkCollisionShapes(shapeA: RBShape, shapeB: RBShape, dynamicA: RigidBodyDynamicParams, dynamicB: RigidBodyDynamicParams) CollisionResult {
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

fn globalResolveCollision(shapeA: RBShape, shapeB: RBShape, pA: *zlm.Vec2, pB: *zlm.Vec2, collision: CollisionResult.Collision) void {
    _ = shapeA;
    _ = shapeB;

    const normal = collision.normal;
    const depth = collision.depth;

    pA.* = pA.add(normal.scale(depth / 2).neg());
    pB.* = pB.add(normal.scale(depth / 2));
}
