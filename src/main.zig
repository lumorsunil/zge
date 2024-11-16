const std = @import("std");
const rl = @import("raylib");
const ecs = @import("ecs");
const zlm = @import("zlm");
const ztracy = @import("ztracy");

const DebugScene = @import("debug.zig").DebugScene;
const RigidBody = @import("physics/rigid-body-flat.zig").RigidBodyFlat;

const cfg = @import("config.zig");

pub fn main() !void {
    const zone = ztracy.ZoneN(@src(), "main");
    defer zone.End();

    var gpa = std.heap.GeneralPurposeAllocator(.{ .safety = true }){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var reg = ecs.Registry.init(allocator);
    defer reg.deinit();

    var scene = DebugScene.init(allocator, &reg);
    defer scene.deinit();
    scene.bind();

    scene.addPlayer(scene.randomPos(), zlm.vec2(10, 10));

    scene.addRectangle(
        zlm.vec2(0, 100),
        zlm.vec2(cfg.size.x * 0.7, 40),
        true,
    );

    rl.initWindow(cfg.size.x, cfg.size.y, "Zig Game Engine Test");
    rl.setWindowPosition(5, 5);

    var lastSample: f64 = 0;

    const stdout = std.io.getStdOut().writer();
    try stdout.print("starting game\n", .{});

    while (!rl.windowShouldClose()) {
        const dt = rl.getFrameTime();
        const t = rl.getTime();

        if (lastSample + 5 < t) {
            lastSample = t;
            try stdout.print("bodies: {}, dt: {}, timeSteps: {d:.1}\n", .{ scene.reg.len(RigidBody), dt, scene.physicsSystem.numberOfTimeSteps(dt, 0.0005) });
        }

        scene.update(dt, t);
        scene.draw();
    }
}
