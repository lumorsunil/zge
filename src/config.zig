const V = @import("vector.zig").V;

pub const size = V.init(1024, 1024);
pub const sizeHalf = size / V.scalar(2);

pub const MAX_ENTITIES = 10000;
