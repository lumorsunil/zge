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
    pub fn aabb(self: CCRTreeEntry) AABB {
        const body = self.value.view.getConst(self.key);
        return static_aabb(body);
    }

    pub fn id(self: CCRTreeEntry) ecs.Entity {
        return self.value.reg.entityId(self.key);
    }

    pub fn static_aabb(body: RigidBody) AABB {
        var bodyAabb = body.aabb();
        const p = body.d.clonePos();
        bodyAabb.tl = p.add(bodyAabb.tl);
        bodyAabb.br = p.add(bodyAabb.br);
        return bodyAabb;
    }

    pub fn init(entity: ecs.Entity, value: *CollisionContainer) CCRTreeEntry {
        return CCRTreeEntry{
            .key = entity,
            .value = value,
        };
    }
};

const CCRTreeEntry = Entry(ecs.Entity, *CollisionContainer, RTreeEntryExtension);
const CCRTree = RTree(CCRTreeEntry);

pub const CollisionContainer = struct {
    tree: CCRTree,
    allocator: Allocator,
    view: ecs.BasicView(RigidBody),
    reg: *ecs.Registry,

    pub const RTreeEntry = CCRTreeEntry;
    pub const RTree = CCRTree;

    pub fn init(allocator: Allocator, reg: *ecs.Registry) CollisionContainer {
        return CollisionContainer{
            .tree = CCRTree.init(allocator),
            .allocator = allocator,
            .view = reg.basicView(RigidBody),
            .reg = reg,
        };
    }

    pub fn deinit(self: CollisionContainer) void {
        self.tree.deinit();
    }

    pub fn insertBody(self: *CollisionContainer, id: ecs.Entity) void {
        self.tree.insertEntry(RTreeEntryExtension.init(id, self));
    }

    pub fn removeEntry(self: *CollisionContainer, entry: RTreeEntry) void {
        self.tree.removeEntry(entry);
    }

    pub fn updateBody(self: *CollisionContainer, id: ecs.Entity) void {
        self.tree.updateEntry(RTreeEntryExtension.init(id, self));
    }

    /// Caller owns returned array
    pub fn intersecting(self: CollisionContainer, body: RigidBody) []*RTreeEntry {
        return self.tree.intersecting(RTreeEntryExtension.static_aabb(body));
    }

    pub fn checkCollision(self: CollisionContainer, body: *RigidBody, context: anytype, callback: fn (context: @TypeOf(context), collision: Collision) void) void {
        const intersectingEntries = self.intersecting(body.*);
        defer self.allocator.free(intersectingEntries);

        for (intersectingEntries) |entry| {
            const other = self.view.get(entry.key);
            const result = body.checkCollision(other);

            switch (result) {
                .noCollision => continue,
                .collision => |collision| callback(context, collision),
            }
        }
    }

    pub fn entryAabb(entry: RTreeEntry) AABB {
        return RTreeEntryExtension.aabb(entry);
    }
};
