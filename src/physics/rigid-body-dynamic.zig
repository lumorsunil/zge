const zlm = @import("zlm");

const pzlm = zlm.SpecializeOn(*f32);

pub const RigidBodyDynamicParams = struct {
    p: pzlm.Vec2,
    v: pzlm.Vec2,
    a: pzlm.Vec2,
    r: *f32,
    rv: *f32,
    ra: *f32,

    pub fn init(p: pzlm.Vec2, v: pzlm.Vec2, a: pzlm.Vec2, r: *f32, rv: *f32, ra: *f32) RigidBodyDynamicParams {
        return RigidBodyDynamicParams{
            .p = p,
            .v = v,
            .a = a,
            .r = r,
            .rv = rv,
            .ra = ra,
        };
    }

    pub fn clonePos(self: *const RigidBodyDynamicParams) zlm.Vec2 {
        return zlm.vec2(self.p.x.*, self.p.y.*);
    }

    pub fn setPos(self: *RigidBodyDynamicParams, pos: zlm.Vec2) void {
        self.p.x.* = pos.x;
        self.p.y.* = pos.y;
    }

    pub fn cloneVel(self: *const RigidBodyDynamicParams) zlm.Vec2 {
        return zlm.vec2(self.v.x.*, self.v.y.*);
    }

    pub fn setVel(self: *RigidBodyDynamicParams, vel: zlm.Vec2) void {
        self.v.x.* = vel.x;
        self.v.y.* = vel.y;
    }
};
