const std = @import("std");
const Allocator = std.mem.Allocator;
const zlm = @import("zlm");
const RTree = @import("r-tree.zig").RTree;
const ecs = @import("ecs");
const RigidBody = @import("../rigid-body-flat.zig").RigidBodyFlat;

pub const CollisionContainer = struct {
    tree: RTree(RigidBody),
    allocator: Allocator,

    pub fn init(allocator: Allocator) !CollisionContainer {
        return CollisionContainer{
            .tree = RTree(RigidBody).init(allocator),
            .allocator = allocator,
        };
    }

    pub fn deinit(self: CollisionContainer) void {
        self.tree.deinit();
    }
};
