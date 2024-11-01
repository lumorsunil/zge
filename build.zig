const std = @import("std");

pub fn build(b: *std.Build) void {
    const options = .{
        .enable_ztracy = b.option(bool, "enable_ztracy", "Enable Tracy profile markers") orelse false,
    };

    const target = b.standardTargetOptions(.{});

    const optimize = b.standardOptimizeOption(.{});

    _ = b.addModule("zge", .{
        .root_source_file = b.path("src/zge.zig"),
    });

    const exe = b.addExecutable(.{
        .name = "zig-game-engine",
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
    });

    const raylib_dep = b.dependency("raylib-zig", .{
        .target = target,
        .optimize = optimize,
    });

    const raylib = raylib_dep.module("raylib");
    const raygui = raylib_dep.module("raygui");
    const raylib_artifact = raylib_dep.artifact("raylib");
    exe.linkLibrary(raylib_artifact);
    exe.root_module.addImport("raylib", raylib);
    exe.root_module.addImport("raygui", raygui);

    const ecs_dep = b.dependency("entt", .{
        .target = target,
        .optimize = optimize,
    });
    const ecs = ecs_dep.module("zig-ecs");
    exe.root_module.addImport("ecs", ecs);

    const zlm_dep = b.dependency("zlm", .{});
    const zlm = zlm_dep.module("zlm");
    exe.root_module.addImport("zlm", zlm);

    const kdtree_dep = b.dependency("kdtree", .{});
    const kdtree = kdtree_dep.module("kdtree");
    exe.root_module.addImport("kdtree", kdtree);

    const ztracy_dep = b.dependency("ztracy", .{
        .enable_ztracy = options.enable_ztracy,
    });
    const ztracy = ztracy_dep.module("root");
    exe.root_module.addImport("ztracy", ztracy);
    exe.linkLibrary(ztracy_dep.artifact("tracy"));

    b.installArtifact(exe);

    const run_cmd = b.addRunArtifact(exe);

    run_cmd.step.dependOn(b.getInstallStep());

    if (b.args) |args| {
        run_cmd.addArgs(args);
    }

    const run_step = b.step("run", "Run the app");
    run_step.dependOn(&run_cmd.step);

    const exe_unit_tests = b.addTest(.{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
    });

    const run_exe_unit_tests = b.addRunArtifact(exe_unit_tests);

    const test_step = b.step("test", "Run unit tests");
    test_step.dependOn(&run_exe_unit_tests.step);
}
