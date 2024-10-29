const rl = @import("raylib");
const cfg = @import("config.zig");

const screenW2 = cfg.size.x / 2;
const screenH2 = cfg.size.y / 2;

pub fn screenPosition(x: f32, y: f32) rl.Vector2 {
    return rl.Vector2.init(x + screenW2, y + screenH2);
}

pub fn screenPositionV(v: rl.Vector2) rl.Vector2 {
    return rl.Vector2.init(v.x + screenW2, v.y + screenH2);
}
