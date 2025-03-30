const std = @import("std");
const Allocator = std.mem.Allocator;
const ArrayList = std.ArrayList;
const ztracy = @import("ztracy");
const RTree = @import("r-tree.zig").RTree;
const QuadTree = @import("quad-tree.zig").QuadTree;
const Entry = @import("r-tree.zig").Entry;
const ecs = @import("ecs");
const RigidBody = @import("../rigid-body-flat.zig").RigidBodyFlat;
const AABB = @import("../shape.zig").AABB;
const Circle = @import("../shape.zig").Circle;
const Collision = @import("result.zig").Collision;
const Intersection = @import("intersection.zig").Intersection;

const CollisionContainerAlgorithm = enum {
    rTree,
    quadTree,
};

pub const ccAlgorithm: CollisionContainerAlgorithm = .quadTree;

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
const CCRTree = RTree(CCRTreeEntry, RTreeEntryExtension.aabb, RTreeEntryExtension.id);

const QuadTreeGetAabb = struct {
    pub fn getAabb(entry: *QuadTreeEntry) AABB {
        return entry.*.aabb;
    }
};

const QuadTreeEntry = RigidBody;

pub const CollisionContainer = struct {
    tree: treeType: {
        if (ccAlgorithm == .rTree) break :treeType CCRTree else break :treeType QT;
    },
    allocator: Allocator,
    view: ecs.BasicView(RigidBody),
    reg: *ecs.Registry,
    collisions: ArrayList(Collision),

    pub const RTreeEntry = CCRTreeEntry;
    pub const RTree = CCRTree;
    pub const QT = QuadTree(QuadTreeEntry, QuadTreeGetAabb.getAabb);

    const EntryType = entryType: {
        if (ccAlgorithm == .rTree) {
            break :entryType CCRTreeEntry;
        } else {
            break :entryType QuadTreeEntry;
        }
    };

    pub fn init(allocator: Allocator, reg: *ecs.Registry) CollisionContainer {
        return CollisionContainer{
            .tree = treeValue: {
                if (ccAlgorithm == .rTree) {
                    break :treeValue CCRTree.init(allocator);
                } else {
                    break :treeValue QT.init(allocator);
                }
            },
            .allocator = allocator,
            .view = reg.basicView(RigidBody),
            .reg = reg,
            .collisions = ArrayList(Collision).init(allocator),
        };
    }

    pub fn deinit(self: CollisionContainer) void {
        self.tree.deinit();
        self.collisions.deinit();
    }

    pub fn insertBody(self: *CollisionContainer, entity: ecs.Entity) void {
        if (ccAlgorithm != .rTree) return;
        self.tree.insertEntry(RTreeEntryExtension.init(entity, self));
    }

    pub fn removeBody(self: *CollisionContainer, entity: ecs.Entity) void {
        if (ccAlgorithm != .rTree) return;
        self.tree.removeEntry(RTreeEntryExtension.init(entity, self));
    }

    pub fn updateBody(self: *CollisionContainer, entity: ecs.Entity) void {
        if (ccAlgorithm != .rTree) return;
        self.tree.updateEntry(RTreeEntryExtension.init(entity, self));
    }

    pub fn sync(self: *CollisionContainer) void {
        if (ccAlgorithm != .rTree) return;
        //self.tree.sortLevels();
        self.tree.optimizeOverlapPhase();
    }

    /// Result is invalidated when this function is called again
    pub fn intersecting(self: *CollisionContainer, body: RigidBody, entityId: ecs.Entity) []Intersection(*EntryType) {
        const zone = ztracy.ZoneN(@src(), "intersecting");
        defer zone.End();
        if (ccAlgorithm == .rTree) {
            return self.tree.intersecting(RTreeEntryExtension.static_aabb(body), entityId);
        } else {
            return self.tree.intersecting(body.aabb);
        }
    }

    /// Result is owned by caller
    pub fn intersectingBody(
        self: *CollisionContainer,
        body: RigidBody,
    ) []Intersection(*EntryType) {
        var result = ArrayList(Intersection(*EntryType)).init(self.allocator);

        const intersections = if (ccAlgorithm == .rTree)
            self.tree.intersecting(body.aabb, 0)
        else
            self.tree.intersecting(body.aabb);

        for (intersections) |intersection| {
            // TODO: Use real collision detection here (right now just using potentially non-minimal AABBs)
            result.append(intersection) catch unreachable;
        }

        return result.toOwnedSlice() catch unreachable;
    }

    /// Result is owned by caller
    pub fn intersectingAABB(
        self: *CollisionContainer,
        aabb: AABB,
    ) []Intersection(*EntryType) {
        var result = ArrayList(Intersection(*EntryType)).init(self.allocator);

        const intersections = if (ccAlgorithm == .rTree)
            self.tree.intersecting(aabb, 0)
        else
            self.tree.intersecting(aabb);

        for (intersections) |intersection| {
            // TODO: Use real collision detection here (right now just using potentially non-minimal AABBs)
            result.append(intersection) catch unreachable;
        }

        return result.toOwnedSlice() catch unreachable;
    }

    /// Result is owned by caller
    pub fn intersectingCircle(
        self: *CollisionContainer,
        circle: Circle,
    ) []Intersection(*EntryType) {
        const aabb = circle.aabb();
        var result = ArrayList(Intersection(*EntryType)).init(self.allocator);

        const intersections = if (ccAlgorithm == .rTree)
            self.tree.intersecting(aabb, 0)
        else
            self.tree.intersecting(aabb);

        for (intersections) |intersection| {
            const d = intersection.entry.aabb.distance(aabb);
            if (d < circle.radius + intersection.entry.aabb.width()) {
                // TODO: Recalculate the intersection based on circle here, or just use the SAT collision detection already implemented
                result.append(intersection) catch unreachable;
            }
        }

        return result.toOwnedSlice() catch unreachable;
    }

    pub fn checkCollisionsQT(self: *CollisionContainer, boundary: AABB) []Collision {
        self.collisions.resize(0) catch unreachable;
        self.tree.populateAndIntersect(boundary, self.view.raw(), self, intersectionHandler);
        return self.collisions.items;
    }

    fn intersectionHandler(
        self: *CollisionContainer,
        body: *RigidBody,
        intersections: []Intersection(*RigidBody),
    ) void {
        for (intersections) |intersection| {
            const other = intersection.entry;

            if (body.s.isStatic and other.s.isStatic) {
                continue;
            }

            if (!body.s.isSolid or !other.s.isSolid) {
                continue;
            }

            if (body.aabb.isMinimal and other.aabb.isMinimal) {
                self.collisions.append(Collision{
                    .bodyA = body,
                    .bodyB = other,
                    .depth = intersection.depth,
                    .normal = intersection.axis,
                    .energyTransferred = undefined,
                    .contact1 = undefined,
                    .contact2 = undefined,
                    .contactCount = undefined,
                }) catch unreachable;
                continue;
            }

            const result = body.checkCollision(other);

            switch (result) {
                .noCollision => continue,
                .collision => |collision| self.collisions.append(collision) catch unreachable,
            }
        }
    }

    pub fn checkCollision(
        self: *CollisionContainer,
        body: *RigidBody,
        entity: ecs.Entity,
        context: anytype,
        callback: fn (context: @TypeOf(context), collision: Collision) void,
    ) void {
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
