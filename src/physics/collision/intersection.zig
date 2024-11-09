const zlm = @import("zlm");

pub fn Intersection(comptime T: type) type {
    return struct {
        entry: T,
        depth: f32,
        axis: zlm.Vec2,
    };
}
