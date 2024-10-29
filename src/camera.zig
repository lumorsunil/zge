const std = @import("std");
const rl = @import("raylib");

const cfg = @import("config.zig");

pub const Camera = struct {
    rect: rl.Rectangle,
    /// Doesn't work
    r: f32,
    s: f32,

    pub fn init() Camera {
        return Camera{
            .rect = rl.Rectangle.init(0, 0, cfg.size.x, cfg.size.y),
            .r = 0,
            .s = 1,
        };
    }

    pub fn angle(self: Camera) f32 {
        return self.r * 180 / std.math.pi;
    }

    pub fn v(self: Camera, vector: rl.Vector2) rl.Vector2 {
        return self.vxy(vector.x, vector.y);
    }

    pub fn vxy(self: Camera, x: f32, y: f32) rl.Vector2 {
        const x_ = (x + self.rect.x) * self.s;
        const y_ = (y + self.rect.y) * self.s;

        if (self.r == 0) return rl.Vector2.init(x_, y_);

        const dx = x - self.rect.x;
        const dy = y - self.rect.y;
        const m = rl.Vector2.init(dx, dy).length();
        const a = std.math.atan2(dy, dx);

        const x__ = m * std.math.cos(a) * self.s;
        const y__ = m * std.math.sin(a) * self.s;

        return rl.Vector2.init(x__ + x, y__ + y);
    }
};
