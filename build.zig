const std = @import("std");

pub fn build(b: *std.Build) void {
    const options = .{
        .enable_ztracy = b.option(bool, "enable_ztracy", "Enable Tracy profile markers") orelse false,
        .enable_ztracy_fibers = b.option(bool, "enable_ztracy_fibers", "Enable ztracy fibers") orelse false,
    };

    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});
    const isWasm = target.result.cpu.arch.isWasm();

    const raylib_dep = if (!isWasm) b.dependency("raylib_zig", .{
        .target = target,
        .optimize = optimize,
    }) else b.dependency("raylib_zig", .{
        .target = target,
        .optimize = optimize,
        .rmodels = false,
    });

    const ztracy_dep = b.dependency("ztracy", .{
        .target = target,
        .optimize = optimize,
        .enable_ztracy = options.enable_ztracy,
        .enable_fibers = options.enable_ztracy_fibers,
    });
    const ztracy = ztracy_dep.module("root");
    // const ztracy = b.createModule(.{
    //     .target = target,
    //     .optimize = optimize,
    //     .root_source_file = b.path("ztracy/ztracy.zig"),
    // });

    const ecs_dep = b.dependency("entt", .{
        .target = target,
        .optimize = optimize,
    });
    const ecs = ecs_dep.module("zig-ecs");

    const zge = b.addModule("zge", .{
        .root_source_file = b.path("src/zge.zig"),
        .imports = &.{
            .{
                .name = "raylib",
                .module = raylib_dep.module("raylib"),
            },
            .{
                .name = "ecs",
                .module = ecs,
            },
            .{
                .name = "ztracy",
                .module = ztracy,
            },
        },
    });

    const exe = b.addExecutable(.{
        .name = "zge",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{
                    .name = "raylib",
                    .module = raylib_dep.module("raylib"),
                },
                .{
                    .name = "ecs",
                    .module = ecs,
                },
                .{
                    .name = "ztracy",
                    .module = ztracy,
                },
            },
        }),
    });

    // const raygui = raylib_dep.module("raygui");
    // const raylib_artifact = raylib_dep.artifact("raylib");
    // exe.linkLibrary(raylib_artifact);
    // exe.root_module.addImport("raylib", raylib);
    // exe.root_module.addImport("raygui", raygui);
    // zge.addImport("raylib", raylib);

    // exe.root_module.addImport("ecs", ecs);
    // zge.addImport("ecs", ecs);
    // exe.root_module.addImport("ztracy", ztracy);
    exe.root_module.linkLibrary(ztracy_dep.artifact("tracy"));
    zge.linkLibrary(ztracy_dep.artifact("tracy"));

    b.installArtifact(exe);
    // b.installArtifact(ztracy_dep.artifact("tracy"));

    const run_cmd = b.addRunArtifact(exe);

    run_cmd.step.dependOn(b.getInstallStep());

    if (b.args) |args| {
        run_cmd.addArgs(args);
    }

    const run_step = b.step("run", "Run the app");
    run_step.dependOn(&run_cmd.step);

    const exe_unit_tests = b.addTest(.{
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
        }),
    });

    const run_exe_unit_tests = b.addRunArtifact(exe_unit_tests);

    const test_step = b.step("test", "Run unit tests");
    test_step.dependOn(&run_exe_unit_tests.step);
}
