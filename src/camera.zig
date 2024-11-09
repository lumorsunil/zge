const std = @import("std");
const zlm = @import("zlm");

const cfg = @import("config.zig");

pub const Camera = struct {
    position: zlm.Vec2,
    size: zlm.Vec2,
    /// Doesn't work yet
    r: f32,
    s: f32,

    pub fn init() Camera {
        return Camera{
            .position = zlm.vec2(0, 0),
            .size = cfg.size,
            .r = 0,
            .s = 1,
        };
    }

    pub fn angle(self: Camera) f32 {
        return self.r * 180 / std.math.pi;
    }

    pub fn transformV(self: Camera, v__: zlm.Vec2) zlm.Vec2 {
        const v_ = self.position.add(v__).scale(self.s);

        if (self.r == 0) return v_;

        unreachable;
    }

    pub fn transform(self: Camera, x: f32, y: f32) zlm.Vec2 {
        return self.transformV(zlm.vec2(x, y));
    }
};
