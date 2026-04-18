const std = @import("std");

pub fn main(init: std.process.Init) !void {
    const V = @Vector(4, f32);

    var v = V{ 2, 3.4, 1.2, 78 };
    const random = std.Random.IoSource{ .io = init.io };
    const len = @typeInfo(V).vector.len;
    const arr: *[len]f32 = @ptrCast(&v);
    for (0..len) |i| arr[i] = random.interface().float(f32);
    // const l = random.interface().uintLessThan(usize, len * 5);
    //
    // for (0..l) |i| {
    //     std.log.debug("{}", .{arr[@mod(i, len)]});
    // }

    std.log.debug("{}", .{v});
}
