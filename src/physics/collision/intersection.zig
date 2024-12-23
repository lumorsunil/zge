const Vector = @import("../../vector.zig").Vector;

pub fn Intersection(comptime T: type) type {
    return struct {
        entry: T,
        depth: f32,
        axis: Vector,
    };
}
