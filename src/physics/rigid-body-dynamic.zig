const zlm = @import("zlm");

pub const RigidBodyDynamicParams = struct {
    p: zlm.Vec2 = zlm.vec2(0, 0),
    v: zlm.Vec2 = zlm.vec2(0, 0),
    a: zlm.Vec2 = zlm.vec2(0, 0),
    r: f32 = 0,
    rv: f32 = 0,
    ra: f32 = 0,
};
