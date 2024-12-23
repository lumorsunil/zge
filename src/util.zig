const std = @import("std");

pub fn filterTo(
    comptime T: type,
    source: []const T,
    dest: []T,
    context: anytype,
    filterFn: fn (@TypeOf(context), T) bool,
) []T {
    var i: usize = 0;

    for (source) |e| {
        if (!filterFn(context, e)) continue;

        dest[i] = e;
        i += 1;
    }

    return dest[0..i];
}

pub fn sortTo(
    comptime T: type,
    source: []const T,
    dest: []T,
    context: anytype,
    lessThanFn: fn (@TypeOf(context), T, T) bool,
) void {
    @memcpy(dest, source);
    std.mem.sort(T, dest, context, lessThanFn);
}
