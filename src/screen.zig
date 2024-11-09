const zlm = @import("zlm");

pub const Screen = struct {
    size: zlm.Vec2 = undefined,
    sizeHalf: zlm.Vec2 = undefined,

    pub fn init(width: f32, height: f32) Screen {
        var screen = Screen{};

        screen.setSize(zlm.vec2(width, height));

        return screen;
    }

    pub fn setSize(self: *Screen, size: zlm.Vec2) void {
        self.size = size;
        self.sizeHalf = size.scale(0.5);
    }

    pub fn screenPosition(self: Screen, x: f32, y: f32) zlm.Vec2 {
        return self.screenPositionV(zlm.vec2(x, y));
    }

    pub fn screenPositionV(self: Screen, v: zlm.Vec2) zlm.Vec2 {
        return v.add(self.sizeHalf);
    }
};
