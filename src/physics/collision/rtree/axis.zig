const std = @import("std");

const Point = struct { x: f32, y: f32 };
pub const Axis = std.meta.FieldEnum(Point);
pub fn getAxisValue(self: Point, axis: Axis) f32 {
    return @field(self, @tagName(axis)).*;
}
