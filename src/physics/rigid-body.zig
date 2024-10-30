const std = @import("std");
const zlm = @import("zlm");

const Shape = @import("shape.zig").Shape;
const RigidBodyDynamicParams = @import("rigid-body-dynamic.zig").RigidBodyDynamicParams;
const col = @import("collision.zig");
const CollisionResult = col.CollisionResult;

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

pub const RigidBody = struct {
    static: *RigidBodyStaticParams,
    dynamic: *RigidBodyDynamicParams,
};

pub const RigidBodyFlat = struct {
    s: RigidBodyStaticParams,
    d: RigidBodyDynamicParams,

    pub const Collision = CollisionResult.Collision;

    pub fn init(shape: Shape, density: f32, restitution: f32, isStatic: bool) Result(RigidBodyFlat) {
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
        return col.checkCollision(self.s.shape, other.s.shape, self.d, other.d);
    }

    pub fn resolveCollision(rbA: *RigidBodyFlat, rbB: *RigidBodyFlat, collision: Collision) void {
        return col.resolveCollision(rbA.s.shape, rbB.s.shape, &rbA.d.p, &rbB.d.p, collision);
    }
};
