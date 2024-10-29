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
    switch (shapeA) {
        .circle => |circleA| {
            switch (shapeB) {
                .circle => |circleB| return checkCollisionCircles(circleA, circleB, dynamicA, dynamicB),
                else => return .noCollision,
            }
        },
        .rectangle => |rectA| {
            switch (shapeB) {
                .rectangle => |rectB| return checkCollisionRectangles(rectA, rectB, dynamicA, dynamicB),
                else => return .noCollision,
            }
        },
    }

    return .noCollision;
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

fn checkCollisionRectangles(rectA: RBShape.Rectangle, rectB: RBShape.Rectangle, dynamicA: RigidBodyDynamicParams, dynamicB: RigidBodyDynamicParams) CollisionResult {
    var minDepth = std.math.inf(f32);
    var minAxis = zlm.vec2(std.math.inf(f32), std.math.inf(f32));

    const pA = dynamicA.p;
    const pB = dynamicB.p;
    const rA = dynamicA.r;
    const rB = dynamicB.r;

    var tvA: [4]zlm.Vec2 = undefined;
    for (0..rectA.vertices.len) |i| {
        tvA[i] = rectA.vertices[i].rotate(rA).add(pA);
    }

    var tvB: [4]zlm.Vec2 = undefined;
    for (0..rectB.vertices.len) |i| {
        tvB[i] = rectB.vertices[i].rotate(rB).add(pB);
    }

    for (0..tvA.len) |i| {
        const v0 = tvA[i];
        const v1 = tvA[@mod(i + 1, tvA.len)];

        const edge = v1.sub(v0);
        const axis = zlm.vec2(-edge.y, edge.x);

        var minA = std.math.inf(f32);
        var maxA = -std.math.inf(f32);
        var minB = std.math.inf(f32);
        var maxB = -std.math.inf(f32);

        for (0..tvA.len) |j| {
            const proj = tvA[j].dot(axis);

            minA = @min(minA, proj);
            maxA = @max(maxA, proj);
        }

        for (0..tvB.len) |j| {
            const proj = tvB[j].dot(axis);

            minB = @min(minB, proj);
            maxB = @max(maxB, proj);
        }

        if (maxA <= minB or maxB <= minA) {
            return .noCollision;
        }

        const depth = @min(maxB - minA, maxA - minB);

        if (depth < minDepth) {
            minDepth = depth;
            minAxis = axis;
        }
    }

    for (0..tvB.len) |i| {
        const v0 = tvB[i];
        const v1 = tvB[@mod(i + 1, tvB.len)];

        const edge = v1.sub(v0);
        const axis = zlm.vec2(-edge.y, edge.x);

        var minB = std.math.inf(f32);
        var maxB = -std.math.inf(f32);
        var minA = std.math.inf(f32);
        var maxA = -std.math.inf(f32);

        for (0..tvB.len) |j| {
            const proj = tvB[j].dot(axis);

            minB = @min(minB, proj);
            maxB = @max(maxB, proj);
        }

        for (0..tvA.len) |j| {
            const proj = tvA[j].dot(axis);

            minA = @min(minA, proj);
            maxA = @max(maxA, proj);
        }

        if (maxA <= minB or maxB <= minA) {
            return .noCollision;
        }

        const depth = @min(maxB - minA, maxA - minB);

        if (depth < minDepth) {
            minDepth = depth;
            minAxis = axis;
        }
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
