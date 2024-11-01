const std = @import("std");

pub fn sortTo(comptime T: type, source: []const T, dest: []T, context: anytype, lessThanFn: fn (@TypeOf(context), T, T) bool) void {
    @memcpy(dest, source);
    std.mem.sort(T, dest, context, lessThanFn);
}
