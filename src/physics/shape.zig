const std = @import("std");
// const ztracy = @import("ztracy");

const V = @import("../vector.zig").V;
const Vector = @import("../vector.zig").Vector;

pub const AABB = struct {
    tl: Vector = V.init(0, 0),
    br: Vector = V.init(0, 0),
    isMinimal: bool,

    pub const Intersection = struct {
        depth: f32,
        axis: Vector,
    };

    pub fn format(
        value: AABB,
        comptime fmt: []const u8,
        options: std.fmt.FormatOptions,
        writer: anytype,
    ) !void {
        _ = options;
        _ = fmt;

        try writer.print("{{{};{}}}", .{ value.tl, value.br });
    }

    pub fn fromRadius(radius: f32, isMinimal: bool) AABB {
        return AABB{
            .tl = V.all(-radius),
            .br = V.all(radius),
            .isMinimal = isMinimal,
        };
    }

    pub fn fromCenter(center_: Vector, size_: Vector, isMinimal: bool) AABB {
        const halfSize = size_ * V.scalar(0.5);

        return AABB{
            .tl = center_ - halfSize,
            .br = center_ + halfSize,
            .isMinimal = isMinimal,
        };
    }

    pub fn ensureContains(self: *AABB, aabb: AABB) void {
        if (self.area() == 0) {
            self.tl = aabb.tl;
            self.br = aabb.br;
        } else {
            self.tl = @min(self.tl, aabb.tl);
            self.br = @min(self.br, aabb.br);
        }
    }

    pub fn intersects(self: AABB, other: AABB) bool {
        // const zone = ztracy.ZoneNC(@src(), "AABB: intersects", 0xff_ff_ff_00);
        // defer zone.End();

        return @reduce(.And, self.br > other.tl) and
            @reduce(.And, other.br > self.tl);
    }

    pub fn lessThan(self: Vector, other: Vector) bool {
        return @reduce(.And, self < other);
    }

    pub fn greaterThan(self: Vector, other: Vector) bool {
        return @reduce(.And, self > other);
    }

    pub fn contains(self: AABB, other: AABB) bool {
        return lessThan(self.tl, other.tl) and greaterThan(self.br, other.br);
    }

    pub fn intersection(self: AABB, other: AABB) ?Intersection {
        // const zone = ztracy.ZoneNC(@src(), "AABB: intersection", 0xff_ff_ff_00);
        // defer zone.End();

        const maxTl = @max(self.tl, other.tl);
        const minBr = @min(self.br, other.br);

        const depthV = minBr - maxTl;

        if (depthV[0] <= 0) return null;

        const depth = V.min(depthV);

        if (depth > 0) {
            var axis: Vector = undefined;

            const otherCenter = other.center();
            const selfCenter = self.center();

            const sd = otherCenter - selfCenter;

            if (V.x(depthV) < V.y(depthV)) {
                const sdx = std.math.sign(V.x(sd));
                axis = V.init(sdx, 0);
            } else {
                const sdy = std.math.sign(V.y(sd));
                axis = V.init(0, sdy);
            }

            return Intersection{ .depth = depth, .axis = axis };
        }

        return null;
    }

    pub fn width(self: AABB) f32 {
        return V.x(self.br) - V.x(self.tl);
    }

    pub fn height(self: AABB) f32 {
        return V.y(self.br) - V.y(self.tl);
    }

    pub fn size(self: AABB) Vector {
        return V.init(self.width(), self.height());
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

    pub fn center(self: AABB) Vector {
        return V.init(self.centerX(), self.centerY());
    }

    pub fn centerX(self: AABB) f32 {
        return V.x(self.tl) + self.width() / 2;
    }

    pub fn centerY(self: AABB) f32 {
        return V.y(self.tl) + self.height() / 2;
    }

    /// Top right
    pub fn tr(self: AABB) Vector {
        return V.init(V.x(self.br), V.y(self.tl));
    }

    /// Bottom left
    pub fn bl(self: AABB) Vector {
        return V.init(V.x(self.tl), V.y(self.br));
    }

    /// Top center
    pub fn tc(self: AABB) Vector {
        return V.init(self.centerX(), V.y(self.tl));
    }

    /// Bottom center
    pub fn bc(self: AABB) Vector {
        return V.init(self.centerX(), V.y(self.br));
    }

    /// Center left
    pub fn cl(self: AABB) Vector {
        return V.init(V.x(self.tl), self.centerY());
    }

    /// Center right
    pub fn cr(self: AABB) Vector {
        return V.init(V.x(self.br), self.centerY());
    }

    pub fn left(self: AABB) f32 {
        return V.x(self.tl);
    }

    pub fn right(self: AABB) f32 {
        return V.x(self.br);
    }

    pub fn top(self: AABB) f32 {
        return V.y(self.tl);
    }

    pub fn bottom(self: AABB) f32 {
        return V.y(self.br);
    }

    pub fn distance(self: AABB, other: AABB) f32 {
        return V.distance(self.center(), other.center());
    }

    pub fn distanceSq(self: AABB, other: AABB) f32 {
        return V.distance2(self.center(), other.center());
    }

    pub fn add(self: AABB, v: Vector) AABB {
        return AABB{
            .tl = self.tl + v,
            .br = self.br + v,
            .isMinimal = self.isMinimal,
        };
    }

    pub fn expand(self: AABB, v: Vector) AABB {
        const x = V.x(v);
        const y = V.y(v);

        return AABB{
            .tl = V.init(
                V.x(self.tl) + if (x > 0) 0 else self.width() * x,
                V.y(self.tl) + if (y > 0) 0 else self.height() * y,
            ),
            .br = V.init(
                V.x(self.br) + if (x < 0) 0 else self.width() * x,
                V.y(self.br) + if (y < 0) 0 else self.height() * y,
            ),
            .isMinimal = self.isMinimal,
        };
    }

    pub fn scale(self: AABB, s: f32) AABB {
        const expansion = self.size() * V.scalar(s / 2);
        const c = self.center();

        return AABB{
            .tl = c - expansion,
            .br = c + expansion,
            .isMinimal = self.isMinimal,
        };
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

    pub fn getVertices(self: *const Shape) []const Vector {
        return switch (self.*) {
            .circle => |*circle| &circle.vertices,
            .rectangle => |*rectangle| &rectangle.vertices,
        };
    }

    pub fn transformedVertices(self: *const Shape) []const Vector {
        return switch (self.*) {
            .circle => |*circle| &circle.transformedVertices,
            .rectangle => |*rectangle| &rectangle.transformedVertices,
        };
    }

    pub fn updateTransform(
        self: *Shape,
        translation: Vector,
        rotation: f32,
        scale: f32,
    ) void {
        switch (self.*) {
            .circle => |*circle| circle.updateTransform(translation, rotation, scale),
            .rectangle => |*rectangle| rectangle.updateTransform(translation, rotation, scale),
        }
    }
};

pub const Circle = struct {
    offset: Vector,
    radius: f32,
    vertices: [1]Vector,
    transformedVertices: [1]Vector,

    pub fn init(radius: f32) Circle {
        return Circle{
            .offset = V.zero,
            .radius = radius,
            .vertices = .{V.zero},
            .transformedVertices = .{V.zero},
        };
    }

    pub fn aabb(self: Circle) AABB {
        return AABB.fromRadius(self.radius, true).add(self.offset);
    }

    pub fn radiusSq(self: Circle) f32 {
        return self.radius * self.radius;
    }

    pub fn area(self: Circle) f32 {
        return self.radius * self.radius * std.math.pi;
    }

    pub fn updateTransform(self: *Circle, translation: Vector, rotation: f32, scale: f32) void {
        _ = rotation;
        _ = scale;

        self.transformedVertices[0] = translation + self.offset;
    }
};

pub const Rectangle = struct {
    offset: Vector,
    size: Vector,
    vertices: [4]Vector,
    transformedVertices: [4]Vector,
    currentRotation: u32 = 0,

    pub fn init(offset: Vector, size: Vector) Rectangle {
        var rect = Rectangle{
            .offset = offset,
            .size = size,
            .vertices = undefined,
            .transformedVertices = undefined,
        };

        setVertices(rect.offset, rect.size, &rect.vertices);
        updateTransform(&rect, V.zero, 0, 1);

        return rect;
    }

    /// Returns the *maximum* aabb bounding box, with potential rotation of the rectangle taken into account.
    pub fn aabb(self: Rectangle, rotation: f32) AABB {
        if (rotation == 0) {
            return AABB{
                .tl = self.vertices[0],
                .br = self.vertices[2],
                .isMinimal = true,
            };
        }
        return AABB.fromRadius(self.circleExtent(), false);
    }

    pub fn width(self: Rectangle) f32 {
        return V.x(self.size);
    }

    pub fn height(self: Rectangle) f32 {
        return V.y(self.size);
    }

    pub fn area(self: Rectangle) f32 {
        return self.width() * self.height();
    }

    /// Returns the radius of the smallest circle that can inscribe the rectangle, i.e:
    /// ```zig
    /// @max(w, h)
    /// ```
    pub fn circleExtent(self: Rectangle) f32 {
        return V.max(self.size);
    }

    pub fn setVertices(offset: Vector, size: Vector, buffer: *[4]Vector) void {
        const halfSize = size / V.scalar(2);

        const l = V.x(offset) - V.x(halfSize);
        const r = V.x(offset) + V.x(halfSize);
        const t = V.y(offset) - V.y(halfSize);
        const b = V.y(offset) + V.y(halfSize);

        buffer[0] = V.init(l, t);
        buffer[1] = V.init(r, t);
        buffer[2] = V.init(r, b);
        buffer[3] = V.init(l, b);
    }

    pub fn updateTransform(
        self: *Rectangle,
        translation: Vector,
        rotation: f32,
        scale: f32,
    ) void {
        if (@as(u32, @bitCast(rotation)) != self.currentRotation) {
            self.currentRotation = @as(u32, @bitCast(rotation));
            for (0.., self.vertices) |i, v| {
                self.transformedVertices[i] = V.rotate(
                    v * V.scalar(scale),
                    rotation,
                ) + translation;
            }
        } else {
            for (0.., self.vertices) |i, v| {
                self.transformedVertices[i] = v * V.scalar(scale) + translation;
            }
        }
    }
};
