const std = @import("std");

const Vector = @import("../vector.zig").Vector;

const Shape = @import("shape.zig").Shape;

pub fn transformVertices(shape: Shape, translation: Vector, rotation: f32, buffer: []Vector) []Vector {
    return switch (shape) {
        .rectangle => |rectangle| transformVerticesRectangle(&rectangle.vertices, translation, rotation, buffer),
        .circle => transformVerticesCircle(translation, buffer),
    };
}

pub fn transformVertex(v: Vector, translation: Vector, rotation: f32) Vector {
    return v.rotate(rotation).add(translation);
}

fn transformVerticesRectangle(v: []const Vector, translation: Vector, rotation: f32, buffer: []Vector) []Vector {
    std.debug.assert(v.len <= buffer.len);

    for (0..v.len) |i| {
        buffer[i] = transformVertex(v[i], translation, rotation);
    }

    return buffer[0..v.len];
}

fn transformVerticesCircle(translation: Vector, buffer: []Vector) []Vector {
    std.debug.assert(buffer.len > 1);

    buffer[0] = translation;

    return buffer[0..1];
}
