const zlm = @import("zlm");
const rl = @import("raylib");

pub fn z2r(v: zlm.Vec2) rl.Vector2 {
    return rl.Vector2.init(v.x, v.y);
}

pub fn z2rect(p: zlm.Vec2, s: zlm.Vec2) rl.Rectangle {
    return rl.Rectangle.init(p.x, p.y, s.x, s.y);
}
