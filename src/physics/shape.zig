const std = @import("std");
const ztracy = @import("ztracy");
const zlm = @import("zlm");

pub const AABB = struct {
    tl: zlm.Vec2 = zlm.vec2(0, 0),
    br: zlm.Vec2 = zlm.vec2(0, 0),
    isMinimal: bool,

    pub const Intersection = struct {
        depth: f32,
        axis: zlm.Vec2,
    };

    pub fn format(value: AABB, comptime fmt: []const u8, options: std.fmt.FormatOptions, writer: anytype) !void {
        _ = options;
        _ = fmt;

        try writer.print("{{{};{}}}", .{ value.tl, value.br });
    }

    pub fn fromRadius(radius: f32, isMinimal: bool) AABB {
        return AABB{
            .tl = .{ .x = -radius, .y = -radius },
            .br = .{ .x = radius, .y = radius },
            .isMinimal = isMinimal,
        };
    }

    pub fn ensureContains(self: *AABB, aabb: AABB) void {
        if (self.area() == 0) {
            self.*.tl.x = aabb.tl.x;
            self.*.tl.y = aabb.tl.y;
            self.*.br.x = aabb.br.x;
            self.*.br.y = aabb.br.y;
        } else {
            self.*.tl.x = @min(self.tl.x, aabb.tl.x);
            self.*.tl.y = @min(self.tl.y, aabb.tl.y);
            self.*.br.x = @max(self.br.x, aabb.br.x);
            self.*.br.y = @max(self.br.y, aabb.br.y);
        }
    }

    pub fn intersects(self: AABB, other: AABB) bool {
        const zone = ztracy.ZoneNC(@src(), "AABB: intersects", 0xff_ff_ff_00);
        defer zone.End();
        return self.br.x > other.tl.x and other.br.x > self.tl.x and
            self.br.y > other.tl.y and other.br.y > self.tl.y;
    }

    pub fn lessThan(self: zlm.Vec2, other: zlm.Vec2) bool {
        return self.x < other.x and self.y < other.y;
    }

    pub fn greaterThan(self: zlm.Vec2, other: zlm.Vec2) bool {
        return self.x > other.x and self.y > other.y;
    }

    pub fn contains(self: AABB, other: AABB) bool {
        return lessThan(self.tl, other.tl) and greaterThan(self.br, other.br);
    }

    pub fn intersection(self: AABB, other: AABB) ?Intersection {
        const zone = ztracy.ZoneNC(@src(), "AABB: intersection", 0xff_ff_ff_00);
        defer zone.End();
        const maxL = @max(self.tl.x, other.tl.x);
        const minR = @min(self.br.x, other.br.x);
        const maxT = @max(self.tl.y, other.tl.y);
        const minB = @min(self.br.y, other.br.y);

        const depthX = minR - maxL;

        if (depthX <= 0) return null;

        const depthY = minB - maxT;

        const depth = @min(depthX, depthY);

        if (depth > 0) {
            var axis: zlm.Vec2 = undefined;

            if (depthX < depthY) {
                const sdx = std.math.sign(other.center().x - self.center().x);
                axis = zlm.vec2(sdx, 0);
            } else {
                const sdy = std.math.sign(other.center().y - self.center().y);
                axis = zlm.vec2(0, sdy);
            }

            return Intersection{ .depth = depth, .axis = axis };
        }

        return null;
    }

    pub fn width(self: AABB) f32 {
        return self.br.x - self.tl.x;
    }

    pub fn height(self: AABB) f32 {
        return self.br.y - self.tl.y;
    }

    pub fn size(self: AABB) zlm.Vec2 {
        return zlm.vec2(self.width(), self.height());
    }

    pub fn area(self: AABB) f32 {
        return self.width() * self.height();
    }

    pub fn areaDifferenceIfEnsureContains(self: AABB, aabb: AABB) f32 {
        var selfCopy = self;
        selfCopy.ensureContains(aabb);

        return selfCopy.area() - self.area();
    }

    pub fn margin(self: AABB) f32 {
        return (self.width() + self.height()) * 2;
    }

    pub fn center(self: AABB) zlm.Vec2 {
        return zlm.vec2(self.centerX(), self.centerY());
    }

    pub fn centerX(self: AABB) f32 {
        return self.tl.x + self.width() / 2;
    }

    pub fn centerY(self: AABB) f32 {
        return self.tl.y + self.height() / 2;
    }

    /// Top right
    pub fn tr(self: AABB) zlm.Vec2 {
        return zlm.vec2(self.br.x, self.tl.y);
    }

    /// Bottom left
    pub fn bl(self: AABB) zlm.Vec2 {
        return zlm.vec2(self.tl.x, self.br.y);
    }

    /// Top center
    pub fn tc(self: AABB) zlm.Vec2 {
        return zlm.vec2(self.centerX(), self.tl.y);
    }

    /// Bottom center
    pub fn bc(self: AABB) zlm.Vec2 {
        return zlm.vec2(self.centerX(), self.br.y);
    }

    /// Center left
    pub fn cl(self: AABB) zlm.Vec2 {
        return zlm.vec2(self.tl.x, self.centerY());
    }

    /// Center right
    pub fn cr(self: AABB) zlm.Vec2 {
        return zlm.vec2(self.br.x, self.centerY());
    }

    pub fn distanceSq(self: AABB, other: AABB) f32 {
        return self.center().distance2(other.center());
    }

    pub fn add(self: AABB, v: zlm.Vec2) AABB {
        return AABB{ .tl = self.tl.add(v), .br = self.br.add(v), .isMinimal = self.isMinimal };
    }

    pub fn scale(self: AABB, s: f32) AABB {
        return AABB{ .tl = self.tl.scale(s), .br = self.br.scale(s), .isMinimal = self.isMinimal };
    }
};

pub const Shape = union(enum) {
    circle: Circle,
    rectangle: Rectangle,

    pub fn radius(self: Shape) f32 {
        return switch (self) {
            .circle => |circle| circle.radius,
            .rectangle => |rectangle| rectangle.circleExtent(),
        };
    }

    pub fn aabb(self: Shape, rotation: f32) AABB {
        return switch (self) {
            .circle => |circle| circle.aabb(),
            .rectangle => |rectangle| rectangle.aabb(rotation),
        };
    }

    pub fn area(self: Shape) f32 {
        return switch (self) {
            .circle => |circle| circle.area(),
            .rectangle => |rectangle| rectangle.area(),
        };
    }

    pub fn vertices(self: *const Shape) []const zlm.Vec2 {
        return switch (self.*) {
            .circle => |*circle| &circle.vertices,
            .rectangle => |*rectangle| &rectangle.vertices,
        };
    }

    pub fn transformedVertices(self: *const Shape) []const zlm.Vec2 {
        return switch (self.*) {
            .circle => |*circle| &circle.transformedVertices,
            .rectangle => |*rectangle| &rectangle.transformedVertices,
        };
    }

    pub fn updateTransform(self: *Shape, translation: zlm.Vec2, rotation: f32, scale: f32) void {
        switch (self.*) {
            .circle => |*circle| circle.updateTransform(translation, rotation, scale),
            .rectangle => |*rectangle| rectangle.updateTransform(translation, rotation, scale),
        }
    }
};

pub const Circle = struct {
    radius: f32,
    vertices: [1]zlm.Vec2,
    transformedVertices: [1]zlm.Vec2,

    pub fn init(radius: f32) Circle {
        return Circle{
            .radius = radius,
            .vertices = .{zlm.Vec2.zero},
            .transformedVertices = .{zlm.Vec2.zero},
        };
    }

    pub fn aabb(self: Circle) AABB {
        return AABB.fromRadius(self.radius, true);
    }

    pub fn radiusSq(self: Circle) f32 {
        return self.radius * self.radius;
    }

    pub fn area(self: Circle) f32 {
        return self.radius * self.radius * std.math.pi;
    }

    pub fn updateTransform(self: *Circle, translation: zlm.Vec2, rotation: f32, scale: f32) void {
        _ = rotation;
        _ = scale;

        self.transformedVertices[0] = translation;
    }
};

pub const Rectangle = struct {
    offset: zlm.Vec2,
    size: zlm.Vec2,
    vertices: [4]zlm.Vec2,
    transformedVertices: [4]zlm.Vec2,
    currentRotation: u32 = 0,

    pub fn init(offset: zlm.Vec2, size: zlm.Vec2) Rectangle {
        var rect = Rectangle{
            .offset = offset,
            .size = size,
            .vertices = undefined,
            .transformedVertices = undefined,
        };

        vertices(rect.offset, rect.size, &rect.vertices);
        updateTransform(&rect, zlm.Vec2.zero, 0, 1);

        return rect;
    }

    /// Returns the *maximum* aabb bounding box, with potential rotation of the rectangle taken into account.
    pub fn aabb(self: Rectangle, rotation: f32) AABB {
        if (rotation == 0) {
            return AABB{ .tl = self.vertices[0], .br = self.vertices[2], .isMinimal = true };
        }
        return AABB.fromRadius(self.circleExtent(), false);
    }

    pub fn width(self: Rectangle) f32 {
        return self.size.x;
    }

    pub fn height(self: Rectangle) f32 {
        return self.size.y;
    }

    pub fn area(self: Rectangle) f32 {
        return self.width() * self.height();
    }

    /// Returns the radius of the smallest circle that can inscribe the rectangle, i.e:
    /// ```zig
    /// @max(w, h)
    /// ```
    pub fn circleExtent(self: Rectangle) f32 {
        return @max(self.width(), self.height());
    }

    pub fn vertices(offset: zlm.Vec2, size: zlm.Vec2, buffer: *[4]zlm.Vec2) void {
        const l = -size.x / 2 + offset.x;
        const r = size.x / 2 + offset.x;
        const t = -size.y / 2 + offset.y;
        const b = size.y / 2 + offset.y;

        buffer[0] = zlm.vec2(l, t);
        buffer[1] = zlm.vec2(r, t);
        buffer[2] = zlm.vec2(r, b);
        buffer[3] = zlm.vec2(l, b);
    }

    pub fn updateTransform(self: *Rectangle, translation: zlm.Vec2, rotation: f32, scale: f32) void {
        if (@as(u32, @bitCast(rotation)) != self.currentRotation) {
            self.currentRotation = @as(u32, @bitCast(rotation));
            for (0.., self.vertices) |i, v| {
                self.transformedVertices[i] = v.scale(scale).rotate(rotation).add(translation);
            }
        } else {
            for (0.., self.vertices) |i, v| {
                self.transformedVertices[i] = v.scale(scale).add(translation);
            }
        }
    }
};
