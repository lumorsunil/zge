const V = @import("vector.zig").V;
const Vector = @import("vector.zig").Vector;

pub const Screen = struct {
    size: Vector = undefined,
    sizeHalf: Vector = undefined,

    pub fn init(size: Vector) Screen {
        return Screen{
            .size = size,
            .sizeHalf = size / V.scalar(2),
        };
    }

    pub fn setSize(self: *Screen, size: Vector) void {
        self.size = size;
        self.sync();
    }

    fn sync(self: *Screen) void {
        self.sizeHalf = self.size / V.scalar(2);
    }

    pub fn screenPosition(self: Screen, x: f32, y: f32) Vector {
        return self.screenPositionV(V.init(x, y));
    }

    pub fn screenPositionV(self: Screen, v: Vector) Vector {
        return v + self.sizeHalf;
    }
};
