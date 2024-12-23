const V = @import("../vector.zig").V;
const Vector = @import("../vector.zig").Vector;
const PVector = @import("../vector.zig").PVector;

pub const RigidBodyDynamicParams = struct {
    p: PVector,
    v: PVector,
    a: PVector,
    pa: Vector = V.zero,
    r: *f32,
    rv: *f32,
    ra: *f32,
    s: f32 = 1,

    pub fn init(p: PVector, v: PVector, a: PVector, r: *f32, rv: *f32, ra: *f32) RigidBodyDynamicParams {
        return RigidBodyDynamicParams{
            .p = p,
            .v = v,
            .a = a,
            .r = r,
            .rv = rv,
            .ra = ra,
        };
    }

    pub fn clonePos(self: *const RigidBodyDynamicParams) Vector {
        return V.fromP(self.p);
    }

    pub fn setPos(self: *RigidBodyDynamicParams, pos: Vector) void {
        V.setP(self.p, pos);
    }

    pub fn cloneVel(self: *const RigidBodyDynamicParams) Vector {
        return V.fromP(self.v);
    }

    pub fn setVel(self: *RigidBodyDynamicParams, vel: Vector) void {
        V.setP(self.v, vel);
    }

    pub fn cloneAccel(self: *const RigidBodyDynamicParams) Vector {
        return V.fromP(self.a);
    }

    pub fn setAccel(self: *RigidBodyDynamicParams, acc: Vector) void {
        V.setP(self.a, acc);
    }
};
