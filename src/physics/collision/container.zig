const std = @import("std");
const Allocator = std.mem.Allocator;
const ztracy = @import("ztracy");
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
        return body.aabb;
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

    pub fn insertBody(self: *CollisionContainer, entity: ecs.Entity) void {
        self.tree.insertEntry(RTreeEntryExtension.init(entity, self));
    }

    pub fn removeBody(self: *CollisionContainer, entity: ecs.Entity) void {
        self.tree.removeEntry(RTreeEntryExtension.init(entity, self));
    }

    pub fn updateBody(self: *CollisionContainer, entity: ecs.Entity) void {
        self.tree.updateEntry(RTreeEntryExtension.init(entity, self));
    }

    pub fn sync(self: *CollisionContainer) void {
        self.tree.sortLevels();
    }

    /// Result is invalidated when this function is called again
    pub fn intersecting(self: *CollisionContainer, body: RigidBody, entityId: ecs.Entity) []CCRTree.Intersection {
        const zone = ztracy.ZoneN(@src(), "intersecting");
        defer zone.End();
        return self.tree.intersecting(RTreeEntryExtension.static_aabb(body), entityId);
    }

    // TODO: Replace callback with returning a list or populating an event list in container for the physics system to consume instead
    pub fn checkCollision(self: *CollisionContainer, body: *RigidBody, entity: ecs.Entity, context: anytype, callback: fn (context: @TypeOf(context), collision: Collision) void) void {
        const zone = ztracy.ZoneNC(@src(), "CC: check collision", 0xff_ff_00_00);
        defer zone.End();
        const entityId = self.reg.entityId(entity);
        const intersections = self.intersecting(body.*, entityId);

        for (intersections) |intersection| {
            const forBodyZone = ztracy.ZoneNC(@src(), "CC: check collision for body", 0xff_ff_00_00);
            defer forBodyZone.End();
            const other = self.view.get(intersection.entry.key);

            if (body.aabb.isMinimal and other.aabb.isMinimal) {
                callback(context, Collision{
                    .bodyA = body,
                    .bodyB = other,
                    .depth = intersection.depth,
                    .normal = intersection.axis,
                    .contact1 = undefined,
                    .contact2 = undefined,
                    .contactCount = undefined,
                });
                continue;
            }

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
