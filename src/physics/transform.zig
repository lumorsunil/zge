const std = @import("std");
const zlm = @import("zlm");

const Shape = @import("shape.zig").Shape;

pub fn transformVertices(shape: Shape, translation: zlm.Vec2, rotation: f32, buffer: []zlm.Vec2) []zlm.Vec2 {
    return switch (shape) {
        .rectangle => |rectangle| transformVerticesRectangle(&rectangle.vertices, translation, rotation, buffer),
        .circle => transformVerticesCircle(translation, buffer),
    };
}

pub fn transformVertex(v: zlm.Vec2, translation: zlm.Vec2, rotation: f32) zlm.Vec2 {
    return v.rotate(rotation).add(translation);
}

fn transformVerticesRectangle(v: []const zlm.Vec2, translation: zlm.Vec2, rotation: f32, buffer: []zlm.Vec2) []zlm.Vec2 {
    std.debug.assert(v.len <= buffer.len);

    for (0..v.len) |i| {
        buffer[i] = transformVertex(v[i], translation, rotation);
    }

    return buffer[0..v.len];
}

fn transformVerticesCircle(translation: zlm.Vec2, buffer: []zlm.Vec2) []zlm.Vec2 {
    std.debug.assert(buffer.len > 1);

    buffer[0] = translation;

    return buffer[0..1];
}
