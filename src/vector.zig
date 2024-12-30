const std = @import("std");
const rl = @import("raylib");

pub fn VectorGeneric(comptime T: type) type {
    return @Vector(2, T);
}

pub const Vector = VectorGeneric(f32);

pub const V = struct {
    pub const zero: Vector = init(0, 0);
    pub const one: Vector = init(1, 1);
    pub const unitX: Vector = init(1, 0);
    pub const unitY: Vector = init(0, 1);
    pub const right: Vector = init(1, 0);
    pub const up: Vector = init(0, -1);
    pub const left: Vector = init(-1, 0);
    pub const down: Vector = init(0, 1);

    pub fn init(x_: f32, y_: f32) Vector {
        return .{ x_, y_ };
    }

    pub fn x(v: Vector) f32 {
        return v[0];
    }

    pub fn y(v: Vector) f32 {
        return v[1];
    }

    pub fn setX(v: *Vector, x_: f32) void {
        v.*[0] = x_;
    }

    pub fn setY(v: *Vector, y_: f32) void {
        v.*[1] = y_;
    }

    pub fn fromRl(rlVector: rl.Vector2) Vector {
        return fromXYV(rlVector);
    }

    pub fn fromXYV(xyv: anytype) Vector {
        return init(xyv.x, xyv.y);
    }

    pub fn fromInt(comptime Int: type, x_: Int, y_: Int) Vector {
        return init(
            @as(f32, @floatFromInt(x_)),
            @as(f32, @floatFromInt(y_)),
        );
    }

    pub fn fromP(p: PVector) Vector {
        return init(
            p[0].*,
            p[1].*,
        );
    }

    pub fn toRl(v: Vector) rl.Vector2 {
        return toXYV(rl.Vector2, v);
    }

    pub fn toXYV(comptime XYV: type, v: Vector) XYV {
        return .{ .x = x(v), .y = y(v) };
    }

    pub fn toInt(comptime Int: type, v: Vector) VectorGeneric(Int) {
        return .{
            @as(Int, @intFromFloat(V.x(v))),
            @as(Int, @intFromFloat(V.y(v))),
        };
    }

    pub fn rect(p: Vector, s: Vector) rl.Rectangle {
        return rl.Rectangle.init(x(p), y(p), x(s), y(s));
    }

    pub fn setP(p: PVector, v: Vector) void {
        p[0].* = x(v);
        p[1].* = y(v);
    }

    pub fn all(n: f32) Vector {
        return init(n, n);
    }

    pub fn onlyX(n: f32) Vector {
        return init(n, 0);
    }

    pub fn onlyY(n: f32) Vector {
        return init(0, n);
    }

    pub fn scalar(n: f32) Vector {
        return @splat(n);
    }

    pub fn random(rand: *std.Random.DefaultPrng) Vector {
        return init(
            rand.random().float(f32),
            rand.random().float(f32),
        );
    }

    pub fn min(v: Vector) f32 {
        return @min(x(v), y(v));
    }

    pub fn max(v: Vector) f32 {
        return @max(x(v), y(v));
    }

    pub fn distance(v: Vector, u: Vector) f32 {
        return @sqrt(distance2(v, u));
    }

    pub fn distance2(v: Vector, u: Vector) f32 {
        const d = u - v;
        return @reduce(.Add, d * d);
    }

    pub fn rotate(v: Vector, r: f32) Vector {
        const cosr = @cos(r);
        const sinr = @sin(r);

        return init(
            cosr * V.x(v) - sinr * V.y(v),
            cosr * V.y(v) + sinr * V.x(v),
        );
    }

    pub fn dot(v: Vector, u: Vector) f32 {
        return @reduce(.Add, v * u);
    }

    pub fn length(v: Vector) f32 {
        return @sqrt(V.length2(v));
    }

    pub fn length2(v: Vector) f32 {
        return V.dot(v, v);
    }

    pub fn normalize(v: Vector) Vector {
        const len = V.length(v);

        return if (len != 0.0)
            v * V.scalar(1.0 / V.length(v))
        else
            V.zero;
    }

    pub fn lessThan(v: Vector, u: Vector) bool {
        return @reduce(.And, v < u);
    }

    pub fn greaterThan(v: Vector, u: Vector) bool {
        return @reduce(.And, v > u);
    }
};

pub const PVector = VectorGeneric(*f32);

pub const PV = struct {
    pub fn init(x: *f32, y: *f32) PVector {
        return .{ x, y };
    }
};
