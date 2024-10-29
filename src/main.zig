const std = @import("std");
const rl = @import("raylib");
const ecs = @import("ecs");

const DebugScene = @import("debug.zig").DebugScene;

const cfg = @import("config.zig");

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{ .safety = true }){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var reg = ecs.Registry.init(allocator);
    defer reg.deinit();

    var scene = DebugScene.init(&reg);
    scene.camera.s = 2;

    for (0..10) |_| {
        scene.addRandomRectangle();
    }

    for (0..10) |_| {
        scene.addRandomCircle();
    }

    rl.initWindow(cfg.size.x, cfg.size.y, "Zig Game Engine Test");

    while (!rl.windowShouldClose()) {
        const dt = rl.getFrameTime();

        scene.update(dt);
        scene.draw();
    }
}
