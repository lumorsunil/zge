// const ztracy = @import("ztracy");
const ecs = @import("ecs");

const V = @import("../vector.zig").V;
const Vector = @import("../vector.zig").Vector;

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
    aabb: AABB,
    key: ecs.Entity,

    pub const Collision = @import("collision/result.zig").Collision;

    pub fn init(key: ecs.Entity, static: RigidBodyStaticParams, dynamic: RigidBodyDynamicParams) RigidBodyFlat {
        var body = RigidBodyFlat{
            .key = key,
            .s = static,
            .d = dynamic,
            .aabb = undefined,
        };

        body.updateTransform();

        return body;
    }

    pub fn momentum(self: RigidBodyFlat) Vector {
        return self.d.cloneVel() * V.scalar(self.s.mass());
    }

    pub fn aabb(self: RigidBodyFlat) AABB {
        return self.s.aabb(self.d.r.*).scale(self.d.s).add(V.fromP(self.d.p));
    }

    pub fn radius(self: *const RigidBodyFlat) f32 {
        return self.s.radius();
    }

    pub fn vertices(self: *const RigidBodyFlat) []const Vector {
        return self.s.vertices();
    }

    pub fn transformedVertices(self: *const RigidBodyFlat) []const Vector {
        return self.s.transformedVertices();
    }

    pub fn updateTransform(self: *RigidBodyFlat) void {
        self.s.updateTransform(self.d.clonePos(), self.d.r.*, self.d.s);
        self.aabb = RigidBodyFlat.aabb(self.*);
    }

    pub fn checkCollision(self: *RigidBodyFlat, other: *RigidBodyFlat) CollisionResult {
        // const zone = ztracy.ZoneNC(@src(), "check collision", 0x00_ff_ff_00);
        // defer zone.End();
        return col.checkCollision(self, other);
    }

    pub fn move(self: *RigidBodyFlat, delta: Vector) void {
        V.setP(self.d.p, V.fromP(self.d.p) + delta);
        self.updateTransform();
    }

    pub fn moveAbsolute(self: *RigidBodyFlat, pos: Vector) void {
        V.setP(self.d.p, pos);
        self.updateTransform();
    }

    pub fn applyForce(self: *RigidBodyFlat, force: Vector) void {
        const mass = self.s.mass();
        const acc = force / V.scalar(mass);
        V.setP(self.d.a, V.fromP(self.d.a) + acc);
    }

    pub fn applyImpulse(self: *RigidBodyFlat, impulse: Vector) void {
        const mass = self.s.mass();
        const vel = impulse / V.scalar(mass);
        V.setP(self.d.v, V.fromP(self.d.v) + vel);
    }
};
