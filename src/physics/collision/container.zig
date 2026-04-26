const std = @import("std");
const Allocator = std.mem.Allocator;
const Io = std.Io;
const ArrayList = std.ArrayList;
const ztracy = @import("ztracy");
const Entry = @import("r-tree.zig").Entry;
const ecs = @import("ecs");
const RigidBody = @import("../rigid-body-flat.zig").RigidBodyFlat;
const AABB = @import("../shape.zig").AABB;
const Vector = @import("../../vector.zig").Vector;
const Circle = @import("../shape.zig").Circle;
const Collision = @import("result.zig").Collision;
const Intersection = @import("intersection.zig").Intersection;
const ZGEConfig = @import("../../config.zig").ZGEConfig;
const GridHash = @import("grid.zig").GridHash;

fn TreeTypes(comptime config: ZGEConfig) type {
    const CC = CollisionContainer(config);

    return switch (config.physics.collision.dynamic_bodies_algorithm) {
        .rTree => brk: {
            const RTree = @import("r-tree.zig").RTree;

            const CCRTreeEntry = struct {
                key: ecs.Entity,
                value: *CC,

                pub const KeyType = ecs.Entity;

                pub fn init(entity: ecs.Entity, value: *CC) @This() {
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
                tree: Tree,

                pub const Tree = CCRTree;
                pub const Entry = CCRTreeEntry;
                pub const algorithm = config.physics.collision.dynamic_bodies_algorithm;

                pub fn insertBody(self: *@This(), cc: *CC, entity: ecs.Entity) void {
                    self.tree.insertEntry(.init(entity, cc));
                }

                pub fn removeBody(self: *@This(), cc: *CC, entity: ecs.Entity) void {
                    self.tree.removeEntry(.init(entity, cc));
                }

                pub fn updateBody(self: *@This(), cc: *CC, entity: ecs.Entity) void {
                    self.tree.updateEntry(.init(entity, cc));
                }

                pub fn sync(self: *@This()) void {
                    //self.tree.sortLevels();
                    self.tree.optimizeOverlapPhase();
                }

                /// Result is invalidated when this function is called again
                pub fn intersecting(
                    self: *CC,
                    body: RigidBody,
                    entityId: ecs.Entity,
                ) []Intersection(*@This().Entry) {
                    const zone = ztracy.ZoneN(@src(), "intersecting");
                    defer zone.End();

                    return self.tree.intersecting(.static_aabb(body), entityId);
                }

                pub fn checkCollisionRTree(
                    self: *@This(),
                    cc: *CC,
                    body: *RigidBody,
                    entity: ecs.Entity,
                    context: anytype,
                    callback: fn (context: @TypeOf(context), collision: Collision) void,
                ) void {
                    const zone = ztracy.ZoneNC(@src(), "CC: check collision", 0xff_ff_00_00);
                    defer zone.End();
                    const entityId = entity.index;
                    const intersections = self.tree.intersecting(self, body.*, entityId);

                    for (intersections) |intersection| {
                        const forBodyZone = ztracy.ZoneNC(@src(), "CC: check collision for body", 0xff_ff_00_00);
                        defer forBodyZone.End();
                        const other = cc.view.get(intersection.entry.key);

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
            };
        },
        .quadTree => brk: {
            const QuadTree = @import("quad-tree.zig").QuadTree;

            const QuadTreeGetAabb = struct {
                pub fn getAabb(ctx: *anyopaque, entryKey: ecs.Entity) AABB {
                    const zone = ztracy.ZoneNC(@src(), "getAabb", 0x00_ff_ff_00);
                    defer zone.End();
                    const reg: *ecs.Registry = @ptrCast(@alignCast(ctx));
                    const body = reg.get(RigidBody, entryKey);
                    return body.aabb;
                }
            };

            const QuadTreeEntry = RigidBody;

            break :brk struct {
                tree: Tree,

                pub const Tree = QuadTree(ecs.Entity, QuadTreeGetAabb.getAabb);
                pub const Entry = QuadTreeEntry;
                pub const algorithm = config.physics.collision.dynamic_bodies_algorithm;

                pub fn insertBody(_: *@This(), _: *CC, _: ecs.Entity) void {
                    unreachable;
                }

                pub fn removeBody(_: *@This(), _: *CC, _: ecs.Entity) void {
                    unreachable;
                }

                pub fn updateBody(_: *@This(), _: *CC, _: ecs.Entity) void {
                    unreachable;
                }

                pub fn sync(_: *@This()) void {}

                pub fn intersecting(
                    self: *@This(),
                    cc: *CC,
                    body: RigidBody,
                ) []Intersection(*@This().Entry) {
                    const zone = ztracy.ZoneN(@src(), "intersecting");
                    defer zone.End();

                    const allocator = cc.allocator;
                    var result = ArrayList(Intersection(ecs.Entity)).empty;
                    self.tree.intersecting(allocator, body.aabb, &result);
                    return result.toOwnedSlice(allocator) catch unreachable;
                }

                pub fn checkCollisionQT(
                    self: *@This(),
                    cc: *CC,
                    options: CC.CheckCollisionQTOptions,
                ) []Collision {
                    const io = cc.reg.singletons().getConst(std.Io);
                    const allocator = cc.allocator;
                    const reg = cc.reg;
                    cc.collisions.resize(allocator, 0) catch unreachable;
                    self.tree.intersection_check_count = 0;
                    // const intersecting_candidates = self.view.data();
                    const intersecting_candidates = cc.filtered_bodies.items;

                    switch (options) {
                        .buildAndCheck => |bac| {
                            self.tree.populateAndIntersectPageByPage(
                                io,
                                allocator,
                                reg,
                                bac.boundary,
                                intersecting_candidates,
                                cc,
                                CC.intersectionHandlerAll,
                            );
                            std.log.debug("intersection_check_count: {}", .{self.tree.intersection_check_count});
                        },
                        .check => {
                            self.tree.updatePositionsAndIntersect(
                                io,
                                allocator,
                                cc,
                                CC.intersectionHandlerAll,
                            );
                        },
                    }

                    return cc.collisions.items;
                }
            };
        },
    };
}

pub fn CollisionContainer(
    comptime config: ZGEConfig,
) type {
    return struct {
        tree: TT,
        allocator: Allocator,
        view: ecs.BasicView(RigidBody),
        filtered_bodies: std.ArrayList(ecs.Entity),
        reg: *ecs.Registry,
        collisions: ArrayList(Collision),
        grid_hash: if (config.isGridHashEnabled()) GridHash else GridHash.Stub,

        const TT = TreeTypes(config);
        pub const Tree = TT.Tree;
        const EntryType = TT.Entry;
        const ccAlgorithm = TT.algorithm;

        pub fn init(allocator: Allocator, reg: *ecs.Registry) @This() {
            return .{
                .tree = .{ .tree = .init(allocator) },
                .allocator = allocator,
                .view = reg.basicView(RigidBody),
                .filtered_bodies = .empty,
                .reg = reg,
                .collisions = .empty,
                .grid_hash = .{},
            };
        }

        pub fn deinit(self: *@This()) void {
            self.tree.tree.deinit(self.allocator);
            self.collisions.deinit(self.allocator);
            self.grid_hash.deinit(self.allocator);
        }

        pub fn startFrame(self: *@This()) void {
            self.populateFilteredBodies();
        }

        pub fn endFrame(self: *@This()) void {
            self.populateTreeWithAllEntities();
        }

        pub fn setGridHash(
            self: *@This(),
            context: anytype,
            gridSize: GridHash.GridCoord,
            cellSize: Vector,
            offset: Vector,
            isTileSolid: *const fn (@TypeOf(context), coord: GridHash.GridCoord) bool,
        ) void {
            const allocator = self.allocator;
            self.grid_hash.populateF(
                allocator,
                context,
                gridSize,
                cellSize,
                offset,
                isTileSolid,
            );
        }

        fn populateFilteredBodies(self: *@This()) void {
            self.filtered_bodies.clearRetainingCapacity();
            for (self.view.data()) |entity| {
                const body = self.view.getConst(entity);
                if (self.filterBody(body)) {
                    self.filtered_bodies.append(self.allocator, entity) catch unreachable;
                }
            }
        }

        fn populateTreeWithAllEntities(self: *@This()) void {
            self.tree.tree.populate(
                self.allocator,
                self.reg,
                self.tree.tree.getRoot().?.aabb,
                self.view.data(),
            );
        }

        fn filterBody(_: *@This(), body: RigidBody) bool {
            return body.s.isSolid;
        }

        pub fn insertBody(self: *@This(), entity: ecs.Entity) void {
            self.tree.insertBody(self, entity);
        }

        pub fn removeBody(self: *@This(), entity: ecs.Entity) void {
            self.tree.removeBody(self, entity);
        }

        pub fn updateBody(self: *@This(), entity: ecs.Entity) void {
            self.tree.updateBody(self, entity);
        }

        pub fn sync(self: *@This()) void {
            self.tree.sync();
        }

        /// Result is owned by caller
        pub fn intersectingBody(
            self: *@This(),
            body: RigidBody,
        ) []Intersection(ecs.Entity) {
            if (ccAlgorithm == .rTree) {
                return self.tree.intersecting(body.aabb, 0);
            } else {
                const allocator = self.allocator;
                var result = ArrayList(Intersection(ecs.Entity)).empty;
                self.tree.intersecting(allocator, body.aabb, &result);
                return result.toOwnedSlice(allocator) catch unreachable;
            }
        }

        /// Result is owned by caller
        pub fn intersectingAABB(
            self: *@This(),
            aabb: AABB,
        ) []Intersection(ecs.Entity) {
            if (ccAlgorithm == .rTree) {
                return self.tree.intersecting(aabb, 0);
            } else {
                const allocator = self.allocator;
                var result = ArrayList(Intersection(ecs.Entity)).empty;
                self.tree.intersecting(allocator, aabb, &result);
                return result.toOwnedSlice(allocator) catch unreachable;
            }
        }

        /// Result is owned by caller
        pub fn intersectingCircle(
            self: *@This(),
            circle: Circle,
        ) []Intersection(*EntryType) {
            const allocator = self.allocator;
            const aabb = circle.aabb();
            var result = ArrayList(Intersection(*EntryType)).empty;

            const intersections = if (ccAlgorithm == .rTree)
                self.tree.intersecting(aabb, 0)
            else brk: {
                self.tree.intersecting(allocator, aabb, &result);
                break :brk result.toOwnedSlice(allocator) catch unreachable;
            };

            for (intersections) |intersection| {
                const d = intersection.entry.aabb.distance(aabb);
                if (d < circle.radius + intersection.entry.aabb.width()) {
                    // TODO: Recalculate the intersection based on circle here, or just use the SAT collision detection already implemented
                    result.append(allocator, intersection) catch unreachable;
                }
            }

            return result.toOwnedSlice(allocator) catch unreachable;
        }

        /// Result is owned by caller
        pub fn intersectingLine(
            self: *@This(),
            start: Vector,
            end: Vector,
        ) []Intersection(ecs.Entity) {
            const allocator = self.allocator;
            var result = ArrayList(Intersection(ecs.Entity)).empty;
            self.tree.intersectingLine(allocator, start, end, &result);
            return result.toOwnedSlice(allocator) catch unreachable;
        }

        fn intersectionHandlerAll(
            self: *@This(),
            entity_keys: []const ecs.Entity,
            all_intersections: [][]Intersection(ecs.Entity),
        ) void {
            const zone = ztracy.ZoneNC(@src(), "intersectionHandlerAll", 0x00_ff_ff_00);
            defer zone.End();

            const allocator = self.allocator;

            for (entity_keys, all_intersections) |entity, intersections| {
                const body_zone = ztracy.ZoneN(@src(), "get body");
                const body = self.reg.get(RigidBody, entity);
                body_zone.End();

                for (intersections) |intersection| {
                    const intersection_zone = ztracy.ZoneN(@src(), "intersection");
                    defer intersection_zone.End();

                    const other_zone = ztracy.ZoneN(@src(), "get other");
                    const otherEntity = intersection.entry;
                    const other = self.reg.get(RigidBody, otherEntity);
                    other_zone.End();

                    {
                        const branching_zone = ztracy.ZoneN(@src(), "branching");
                        defer branching_zone.End();

                        if (body.s.isStatic and other.s.isStatic) {
                            continue;
                        }

                        if (!body.s.isSolid or !other.s.isSolid) {
                            continue;
                        }
                    }

                    const append_zone = ztracy.ZoneN(@src(), "append");
                    defer append_zone.End();
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
        }

        fn intersectionHandler(
            self: *@This(),
            entity: ecs.Entity,
            intersections: []Intersection(ecs.Entity),
        ) void {
            const zone = ztracy.ZoneNC(@src(), "intersectionHandler", 0x00_ff_ff_00);
            defer zone.End();

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

        pub fn checkCollisionRTree(
            self: *@This(),
            body: *RigidBody,
            entity: ecs.Entity,
            context: anytype,
            callback: fn (context: @TypeOf(context), collision: Collision) void,
        ) void {
            if (@hasDecl(@TypeOf(self.tree), "checkCollisionRTree")) {
                self.tree.checkCollisionRTree(self, body, entity, context, callback);
            }
        }

        pub fn checkCollisionQT(
            self: *@This(),
            options: CheckCollisionQTOptions,
        ) []Collision {
            if (@hasDecl(@TypeOf(self.tree), "checkCollisionQT")) {
                return self.tree.checkCollisionQT(self, options);
            }
        }

        pub fn entryAabb(entry: EntryType) AABB {
            return entry.aabb();
        }

        pub const CheckCollisionQTOptions = union(enum) {
            buildAndCheck: struct {
                boundary: AABB,
            },
            check,

            pub fn build(boundary: AABB) CheckCollisionQTOptions {
                return .{ .buildAndCheck = .{ .boundary = boundary } };
            }
        };
    };
}
