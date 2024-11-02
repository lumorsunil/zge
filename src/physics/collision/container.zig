const std = @import("std");
const Allocator = std.mem.Allocator;
const zlm = @import("zlm");
const RTree = @import("r-tree.zig").RTree;
const Entry = @import("r-tree.zig").Entry;
const ecs = @import("ecs");
const RigidBody = @import("../rigid-body-flat.zig").RigidBodyFlat;
const AABB = @import("../shape.zig").AABB;
const Collision = @import("result.zig").Collision;

const RTreeEntryExtension = struct {
    pub fn aabb(self: RTreeEntry) AABB {
        return static_aabb(self.value);
    }

    pub fn static_aabb(body: *const RigidBody) AABB {
        var bodyAabb = body.aabb();
        const p = body.d.clonePos();
        bodyAabb.tl = p.add(bodyAabb.tl);
        bodyAabb.br = p.add(bodyAabb.br);
        return bodyAabb;
    }

    pub fn init(id: ecs.Entity, body: *RigidBody) RTreeEntry {
        return RTreeEntry{
            .key = id,
            .value = body,
        };
    }
};

const RTreeEntry = Entry(ecs.Entity, *RigidBody, RTreeEntryExtension);
const CCRTree = RTree(RTreeEntry);

pub const CollisionContainer = struct {
    tree: CCRTree,
    allocator: Allocator,

    pub fn init(allocator: Allocator) CollisionContainer {
        return CollisionContainer{
            .tree = CCRTree.init(allocator),
            .allocator = allocator,
        };
    }

    pub fn deinit(self: CollisionContainer) void {
        self.tree.deinit();
    }

    pub fn insertBody(self: *CollisionContainer, id: ecs.Entity, body: *RigidBody) void {
        self.tree.insertEntry(RTreeEntryExtension.init(id, body));
    }

    pub fn removeEntry(self: *CollisionContainer, entry: RTreeEntry) void {
        self.tree.removeEntry(entry);
    }

    pub fn updateBody(self: *CollisionContainer, id: ecs.Entity, body: *RigidBody) void {
        self.tree.updateEntry(RTreeEntryExtension.init(id, body));
    }

    /// Caller owns returned array
    pub fn intersecting(self: CollisionContainer, body: *const RigidBody) []*RTreeEntry {
        return self.tree.intersecting(RTreeEntryExtension.static_aabb(body));
    }

    pub fn checkCollision(self: CollisionContainer, body: *RigidBody, context: anytype, callback: fn (context: @TypeOf(context), collision: Collision) void) void {
        const intersectingEntries = self.intersecting(body);
        defer self.allocator.free(intersectingEntries);

        for (intersectingEntries) |entry| {
            const result = body.checkCollision(entry.value);

            switch (result) {
                .noCollision => continue,
                .collision => |collision| callback(context, collision),
            }
        }
    }
};
