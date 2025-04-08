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
};
