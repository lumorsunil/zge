const zlm = @import("zlm");
const ztracy = @import("ztracy");

const Shape = @import("shape.zig").Shape;
const AABB = @import("shape.zig").AABB;
const Result = @import("result.zig").Result;
const RigidBodyStaticParams = @import("rigid-body-static.zig").RigidBodyStaticParams;
const RigidBodyDynamicParams = @import("rigid-body-dynamic.zig").RigidBodyDynamicParams;
const col = @import("collision.zig");
const CollisionResult = @import("collision/result.zig").CollisionResult;

pub const RigidBodyFlat = struct {
    s: RigidBodyStaticParams,
    d: RigidBodyDynamicParams,

    pub const Collision = @import("collision/result.zig").Collision;

    pub fn init(static: RigidBodyStaticParams, dynamic: RigidBodyDynamicParams) RigidBodyFlat {
        return RigidBodyFlat{
            .s = static,
            .d = dynamic,
        };
    }

    pub fn aabb(self: RigidBodyFlat) AABB {
        return self.s.aabb();
    }

    pub fn radius(self: *const RigidBodyFlat) f32 {
        return self.s.radius();
    }

    pub fn vertices(self: *const RigidBodyFlat) []const zlm.Vec2 {
        return self.s.vertices();
    }

    pub fn transformedVertices(self: *const RigidBodyFlat) []const zlm.Vec2 {
        return self.s.transformedVertices();
    }

    pub fn updateTransform(self: *RigidBodyFlat) void {
        self.s.updateTransform(self.d.clonePos(), self.d.r.*);
    }

    pub fn checkCollision(self: *RigidBodyFlat, other: *RigidBodyFlat) CollisionResult {
        const zone = ztracy.ZoneNC(@src(), "check collision", 0x00_ff_ff_00);
        defer zone.End();

        if (self.s.isStatic and other.s.isStatic) return .noCollision;
        return col.checkCollision(self, other);
    }

    pub fn move(self: *RigidBodyFlat, delta: zlm.Vec2) void {
        const dest = self.d.p.add(delta);
        self.d.p.x.* = dest.x;
        self.d.p.y.* = dest.y;
        self.updateTransform();
    }

    pub fn moveAbsolute(self: *RigidBodyFlat, delta: zlm.Vec2) void {
        self.d.p.x.* = delta.x;
        self.d.p.y.* = delta.y;
        self.updateTransform();
    }

    pub fn applyForce(self: *RigidBodyFlat, force: zlm.Vec2) void {
        const mass = self.s.mass();
        const acc = force.div(.{ .x = mass, .y = mass });
        self.d.a.x.* += acc.x;
        self.d.a.y.* += acc.y;
    }
};
