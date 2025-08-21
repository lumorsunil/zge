const Vector = @import("../../vector.zig").Vector;

pub fn Intersection(comptime K: type) type {
    return struct {
        entry: K,
        depth: f32,
        axis: Vector,
    };
}
