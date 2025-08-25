const std = @import("std");
const Allocator = std.mem.Allocator;
const ArrayList = std.ArrayList;
const ztracy = @import("ztracy");
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

const TreeTypes = switch (ccAlgorithm) {
    .rTree => brk: {
        const RTree = @import("r-tree.zig").RTree;

        const CCRTreeEntry = struct {
            key: ecs.Entity,
            value: *CollisionContainer,

            pub const KeyType = ecs.Entity;

            pub fn init(entity: ecs.Entity, value: *CollisionContainer) @This() {
                return .{
                    .key = entity,
                    .value = value,
                };
            }

            pub fn aabb(self: @This()) AABB {
                const body = self.value.view.getConst(self.key);
                return static_aabb(body);
            }

            pub fn id(self: @This()) ecs.Entity {
                return self.key.index;
            }

            pub fn static_aabb(body: RigidBody) AABB {
                return body.aabb;
            }
        };

        const CCRTree = RTree(CCRTreeEntry, CCRTreeEntry.aabb, CCRTreeEntry.id);

        break :brk struct {
            pub const Tree = CCRTree;
            pub const Entry = CCRTreeEntry;
        };
    },
    .quadTree => brk: {
        const QuadTree = @import("quad-tree.zig").QuadTree;

        const QuadTreeGetAabb = struct {
            pub fn getAabb(ctx: *anyopaque, entryKey: ecs.Entity) AABB {
                const reg: *ecs.Registry = @ptrCast(@alignCast(ctx));
                const body = reg.get(RigidBody, entryKey);
                return body.aabb;
            }
        };

        const QuadTreeEntry = RigidBody;

        break :brk struct {
            pub const Tree = QuadTree(ecs.Entity, QuadTreeGetAabb.getAabb);
            pub const Entry = QuadTreeEntry;
        };
    },
};

pub const CollisionContainer = struct {
    tree: TreeTypes.Tree,
    allocator: Allocator,
    view: ecs.BasicView(RigidBody),
    reg: *ecs.Registry,
    collisions: ArrayList(Collision),

    pub const Tree = TreeTypes.Tree;
    const EntryType = TreeTypes.Entry;

    pub fn init(allocator: Allocator, reg: *ecs.Registry) CollisionContainer {
        return CollisionContainer{
            .tree = .init(allocator),
            .allocator = allocator,
            .view = reg.basicView(RigidBody),
            .reg = reg,
            .collisions = .empty,
        };
    }

    pub fn deinit(self: *CollisionContainer) void {
        self.tree.deinit(self.allocator);
        self.collisions.deinit(self.allocator);
    }

    pub fn insertBody(self: *CollisionContainer, entity: ecs.Entity) void {
        if (ccAlgorithm != .rTree) return;
        self.tree.insertEntry(.init(entity, self));
    }

    pub fn removeBody(self: *CollisionContainer, entity: ecs.Entity) void {
        if (ccAlgorithm != .rTree) return;
        self.tree.removeEntry(.init(entity, self));
    }

    pub fn updateBody(self: *CollisionContainer, entity: ecs.Entity) void {
        if (ccAlgorithm != .rTree) return;
        self.tree.updateEntry(.init(entity, self));
    }

    pub fn sync(self: *CollisionContainer) void {
        if (ccAlgorithm != .rTree) return;
        //self.tree.sortLevels();
        self.tree.optimizeOverlapPhase();
    }

    /// Result is invalidated when this function is called again
    pub fn intersecting(
        self: *CollisionContainer,
        body: RigidBody,
        entityId: ecs.Entity,
    ) []Intersection(*EntryType) {
        const zone = ztracy.ZoneN(@src(), "intersecting");
        defer zone.End();
        if (ccAlgorithm == .rTree) {
            return self.tree.intersecting(.static_aabb(body), entityId);
        } else {
            return self.tree.intersecting(self.allocator, body.aabb);
        }
    }

    /// Result is owned by caller
    pub fn intersectingBody(
        self: *CollisionContainer,
        body: RigidBody,
    ) []Intersection(ecs.Entity) {
        const allocator = self.allocator;
        var result = ArrayList(Intersection(ecs.Entity)).empty;

        const intersections = if (ccAlgorithm == .rTree)
            self.tree.intersecting(body.aabb, 0)
        else
            self.tree.intersecting(allocator, body.aabb);

        for (intersections) |intersection| {
            // TODO: Use real collision detection here (right now just using potentially non-minimal AABBs)
            result.append(allocator, intersection) catch unreachable;
        }

        return result.toOwnedSlice(allocator) catch unreachable;
    }

    /// Result is owned by caller
    pub fn intersectingAABB(
        self: *CollisionContainer,
        aabb: AABB,
    ) []Intersection(ecs.Entity) {
        const allocator = self.allocator;
        var result = ArrayList(Intersection(ecs.Entity)).empty;

        const intersections = if (ccAlgorithm == .rTree)
            self.tree.intersecting(aabb, 0)
        else
            self.tree.intersecting(allocator, aabb);

        for (intersections) |intersection| {
            // TODO: Use real collision detection here (right now just using potentially non-minimal AABBs)
            result.append(allocator, intersection) catch unreachable;
        }

        return result.toOwnedSlice(allocator) catch unreachable;
    }

    /// Result is owned by caller
    pub fn intersectingCircle(
        self: *CollisionContainer,
        circle: Circle,
    ) []Intersection(*EntryType) {
        const allocator = self.allocator;
        const aabb = circle.aabb();
        var result = ArrayList(Intersection(*EntryType)).empty;

        const intersections = if (ccAlgorithm == .rTree)
            self.tree.intersecting(aabb, 0)
        else
            self.tree.intersecting(allocator, aabb);

        for (intersections) |intersection| {
            const d = intersection.entry.aabb.distance(aabb);
            if (d < circle.radius + intersection.entry.aabb.width()) {
                // TODO: Recalculate the intersection based on circle here, or just use the SAT collision detection already implemented
                result.append(allocator, intersection) catch unreachable;
            }
        }

        return result.toOwnedSlice(allocator) catch unreachable;
    }

    pub fn checkCollisionsQT(self: *CollisionContainer, boundary: AABB) []Collision {
        const allocator = self.allocator;
        self.collisions.resize(allocator, 0) catch unreachable;
        self.tree.populateAndIntersect(
            self.allocator,
            self.reg,
            boundary,
            self.view.data(),
            self,
            intersectionHandler,
        );
        return self.collisions.items;
    }

    fn intersectionHandler(
        self: *CollisionContainer,
        entity: ecs.Entity,
        intersections: []Intersection(ecs.Entity),
    ) void {
        const allocator = self.allocator;
        const body = self.reg.get(RigidBody, entity);

        for (intersections) |intersection| {
            const otherEntity = intersection.entry;
            const other = self.reg.get(RigidBody, otherEntity);

            if (body.s.isStatic and other.s.isStatic) {
                continue;
            }

            if (!body.s.isSolid or !other.s.isSolid) {
                continue;
            }

            if (body.aabb.isMinimal and other.aabb.isMinimal) {
                self.collisions.append(allocator, Collision{
                    .bodyA = body,
                    .bodyB = other,
                    .depth = intersection.depth,
                    .normal = intersection.axis,
                    .contact1 = undefined,
                    .contact2 = undefined,
                    .contactCount = undefined,
                }) catch unreachable;
                continue;
            }

            const result = body.checkCollision(other);

            switch (result) {
                .noCollision => continue,
                .collision => |collision| self.collisions.append(allocator, collision) catch unreachable,
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
        const entityId = entity.index;
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

    pub fn entryAabb(entry: EntryType) AABB {
        return entry.aabb();
    }
};
