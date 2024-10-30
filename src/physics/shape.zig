const std = @import("std");
const zlm = @import("zlm");

pub const Shape = union(enum) {
    circle: Circle,
    rectangle: Rectangle,

    pub fn area(self: Shape) f32 {
        return switch (self) {
            .circle => |circle| circle.area(),
            .rectangle => |rectangle| rectangle.area(),
        };
    }

    pub const Circle = struct {
        radius: f32,

        pub fn radiusSq(self: Circle) f32 {
            return self.radius * self.radius;
        }

        pub fn area(self: Circle) f32 {
            return self.radius * self.radius * std.math.pi;
        }
    };

    pub const Rectangle = struct {
        size: zlm.Vec2,
        vertices: [4]zlm.Vec2,

        pub fn init(size: zlm.Vec2) Rectangle {
            var rect = Rectangle{
                .size = size,
                .vertices = undefined,
            };

            vertices(rect.size, &rect.vertices);

            return rect;
        }

        pub fn width(self: Rectangle) f32 {
            return self.size.x;
        }

        pub fn height(self: Rectangle) f32 {
            return self.size.x;
        }

        pub fn area(self: Rectangle) f32 {
            return self.width() * self.height();
        }

        pub fn vertices(size: zlm.Vec2, buffer: *[4]zlm.Vec2) void {
            const l = -size.x / 2;
            const r = -l;
            const t = -size.y / 2;
            const b = -t;

            buffer[0] = zlm.vec2(l, t);
            buffer[1] = zlm.vec2(r, t);
            buffer[2] = zlm.vec2(r, b);
            buffer[3] = zlm.vec2(l, b);
        }
    };
};
