const std = @import("std");
const Allocator = std.mem.Allocator;
const ArrayList = std.ArrayList;

const AABB = @import("../shape.zig").AABB;

const Point = struct { x: f32, y: f32 };
const Axis = std.meta.FieldEnum(Point);
fn getAxisValue(self: Point, axis: Axis) f32 {
    return @field(self, @tagName(axis)).*;
}

const INTERSECTING_INITIAL_CAPACITY = 100;

/// Leaf type must implement method:
/// ```zig
/// fn aabb() AABB
/// ```
pub fn RTree(comptime Leaf: type) type {
    return struct {
        allocator: Allocator,
        root: Node,

        const Self = @This();

        const Node = struct {
            level: u32,
            entries: ArrayList(Entry),

            pub fn init(allocator: Allocator, level: u32) Node {
                return Node{
                    .level = level,
                    .entries = ArrayList(Entry).init(allocator),
                };
            }

            pub fn deinit(self: Node) void {
                for (self.entries.items) |entry| {
                    switch (entry) {
                        .leaf => {},
                        .branch => |branch| {
                            branch.node.deinit();
                        },
                    }
                }
                self.entries.deinit();
            }
        };

        const Branch = struct {
            key: AABB,
            node: Node,
        };

        const Entry = union(enum) {
            leaf: Leaf,
            branch: Branch,
        };

        pub fn init(allocator: Allocator) Self {
            return Self{
                .allocator = allocator,
                .root = Node.init(allocator, 0),
            };
        }

        pub fn deinit(self: Self) void {
            self.root.deinit();
        }

        /// Caller owns returned array
        pub fn intersecting(self: Self, aabb: AABB) []Leaf {
            var leaves = ArrayList(Leaf).initCapacity(self.allocator, INTERSECTING_INITIAL_CAPACITY) catch unreachable;

            self.intersectingInner(aabb, self.root, &leaves);

            return leaves.toOwnedSlice() catch unreachable;
        }

        fn intersectingInner(self: Self, aabb: AABB, node: *const Node, leaves: *ArrayList(Leaf)) void {
            _ = self;

            for (node.*.entries.items) |entry| {
                switch (entry) {
                    .leaf => |leaf| {
                        const leafAabb: AABB = leaf.aabb();

                        if (leafAabb.intersects(aabb)) {
                            leaves.append(leaf);
                        }
                    },
                }
            }
        }
    };
}
