const std = @import("std");

pub const Zone = struct {};
pub fn ZoneN(src: std.builtin.SourceLocation, label: []const u8) ZoneCtx {
    _ = src;
    _ = label;
    return .{};
}
pub fn ZoneNC(src: std.builtin.SourceLocation, label: []const u8, color: u32) ZoneCtx {
    _ = src;
    _ = label;
    _ = color;
    return .{};
}

pub const ZoneCtx = struct {
    pub fn End(_: @This()) void {}
};
