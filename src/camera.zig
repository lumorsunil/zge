const std = @import("std");

const cfg = @import("config.zig");

const V = @import("vector.zig").V;
const Vector = @import("vector.zig").Vector;

pub const Camera = struct {
    position: Vector,
    size: Vector,
    /// Doesn't work yet
    r: f32,
    s: f32,

    pub fn init() Camera {
        return Camera{
            .position = V.init(0, 0),
            .size = cfg.size,
            .r = 0,
            .s = 1,
        };
    }

    pub fn angle(self: Camera) f32 {
        return self.r * 180 / std.math.pi;
    }

    pub fn transformV(self: Camera, position: Vector) Vector {
        const transformedPosition = self.position + position * V.scalar(self.s);

        if (self.r == 0) return transformedPosition;

        unreachable;
    }

    pub fn transform(self: Camera, x: f32, y: f32) Vector {
        return self.transformV(V.init(x, y));
    }
};
