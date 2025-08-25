const std = @import("std");
const rl = @import("raylib");
const ecs = @import("ecs");
// const ztracy = @import("ztracy");

const DebugScene = @import("debug.zig").DebugScene;
const RigidBody = @import("physics/rigid-body-flat.zig").RigidBodyFlat;

const V = @import("vector.zig").V;

const cfg = @import("config.zig");

pub fn main() !void {
    // const zone = ztracy.ZoneN(@src(), "main");
    // defer zone.End();

    var gpa = std.heap.GeneralPurposeAllocator(.{ .safety = true }){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var reg = ecs.Registry.init(allocator);
    defer reg.deinit();

    var scene = DebugScene.init(allocator, &reg);
    defer scene.deinit();
    scene.bind();

    scene.addPlayer(scene.randomPos(), V.init(10, 10));

    scene.addRectangle(
        V.init(0, 100),
        V.init(V.x(cfg.size) * 0.7, 40),
        true,
    );

    const sizeInt = V.toInt(i32, cfg.size);
    rl.initWindow(sizeInt[0], sizeInt[1], "Zig Game Engine Test");
    rl.setWindowPosition(5, 5);

    var lastSample: f64 = 0;

    var stdout_buffer: [1024]u8 = undefined;
    var stdout_writer = std.fs.File.stdout().writer(&stdout_buffer);
    const stdout = &stdout_writer.interface;

    try stdout.print("starting game\n", .{});
    try stdout.flush();

    while (!rl.windowShouldClose()) {
        const dt = rl.getFrameTime();
        const t = rl.getTime();

        if (lastSample + 5 < t) {
            lastSample = t;
            try stdout.print("bodies: {}, dt: {}, timeSteps: {d:.1}\n", .{ scene.reg.len(RigidBody), dt, scene.physicsSystem.numberOfTimeSteps(dt, 0.0005) });
        }

        scene.update(dt, t);
        scene.draw();

        try stdout.flush();
    }
}
