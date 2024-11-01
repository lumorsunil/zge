pub fn Result(comptime T: type) type {
    return union(enum) {
        success: T,
        err: []const u8,
    };
}
