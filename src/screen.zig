const zlm = @import("zlm");

pub const Screen = struct {
    size: zlm.Vec2 = undefined,
    sizeHalf: zlm.Vec2 = undefined,

    pub fn init(width: f32, height: f32) Screen {
        var screen = Screen{};

        screen.setSize(zlm.vec2(width, height));

        return screen;
    }

    pub fn asInt(self: Screen, comptime T: type) zlm.SpecializeOn(T).Vec2 {
        return zlm.SpecializeOn(T).vec2(
            @as(T, @intFromFloat(self.size.x)),
            @as(T, @intFromFloat(self.size.y)),
        );
    }

    pub fn setSize(self: *Screen, size: zlm.Vec2) void {
        self.size = size;
        self.sync();
    }

    pub fn setSizeFromInt(self: *Screen, comptime T: type, size: zlm.SpecializeOn(T).Vec2) void {
        self.size = zlm.vec2(
            @as(f32, @floatFromInt(size.x)),
            @as(f32, @floatFromInt(size.y)),
        );
        self.sync();
    }

    fn sync(self: *Screen) void {
        self.sizeHalf = self.size.scale(0.5);
    }

    pub fn screenPosition(self: Screen, x: f32, y: f32) zlm.Vec2 {
        return self.screenPositionV(zlm.vec2(x, y));
    }

    pub fn screenPositionV(self: Screen, v: zlm.Vec2) zlm.Vec2 {
        return v.add(self.sizeHalf);
    }
};
