const std = @import("std");
const Allocator = std.mem.Allocator;
const ArrayList = std.ArrayList;
const BoundedArray = std.BoundedArray;
const ztracy = @import("ztracy");
const zlm = @import("zlm");

const AABB = @import("../shape.zig").AABB;

const Point = struct { x: f32, y: f32 };
const Axis = std.meta.FieldEnum(Point);
fn getAxisValue(self: Point, axis: Axis) f32 {
    return @field(self, @tagName(axis)).*;
}

const MAX_ENTRIES_IN_NODE = 5;
const MIN_ENTRIES_IN_NODE = 2;
const INTERSECTING_INITIAL_CAPACITY = MAX_ENTRIES_IN_NODE * 10;
const MAXIMUM_AREA_THRESHOLD_FACTOR = 1.5;
const STAR_P: usize = @max(1, @trunc(MAX_ENTRIES_IN_NODE * 0.3));

const RtreeInsertionAlgorithm = enum {
    linear,
    star,
};

const insertAlgorithm: RtreeInsertionAlgorithm = .star;

/// Extends type must implement methods:
/// ```zig
/// fn aabb(self: Entry(Key, Value, Ext)) AABB
/// fn id(self: Entry(Key, Value, Ext)) <unsigned integer>
///
/// // Example:
///
/// const MyEntryExtension = struct {
///     pub fn aabb(self: MyEntry) AABB {
///         return ...;
///     }
///
///     pub fn id(self: MyEntry) <unsigned integer> {
///         return ...;
///     }
/// };
///
/// const MyEntry = Entry(MyKey, MyValue, MyEntryExtension);
/// ```
pub fn Entry(comptime Key: type, comptime Value: type, comptime Extends: type) type {
    return struct {
        key: Key,
        value: Value,

        pub const KeyType = Key;

        usingnamespace Extends;
    };
}

/// EntryType must be of type `Entry(K, V, E)`
pub fn RTree(comptime EntryType: type) type {
    return struct {
        allocator: Allocator,
        root: *Page,
        levels: ArrayList(*Level),
        height: usize,
        keyToNodeMap: ArrayList(*Node),
        // TODO: break out into a runner to make it more multi-thread-friendly
        // TODO: try changing to a static array and fail if you have more collisions than capacity
        intersectingResult: IntersectingResultArray,

        const IntersectingResultArray = BoundedArray(Intersection, INTERSECTING_INITIAL_CAPACITY);
        const Self = @This();

        pub const Node = union(enum) {
            entry: NodeEntry,
            page: *Page,

            pub const NodeEntry = struct {
                entry: EntryType,
                parent: *Page,
            };

            pub fn init(allocator: Allocator, node: Node) *Node {
                const nodePtr = allocator.create(Node) catch unreachable;
                nodePtr.* = node;
                return nodePtr;
            }

            pub fn initEntry(allocator: Allocator, entry: EntryType, parent: *Page) *Node {
                return Node.init(allocator, Node{
                    .entry = NodeEntry{ .entry = entry, .parent = parent },
                });
            }

            pub fn initPage(allocator: Allocator, pageAabb: AABB, parent: ?*Page) *Node {
                return Node.init(allocator, Node{
                    .page = Page.init(allocator, pageAabb, parent),
                });
            }

            pub fn deinit(self: *Node, allocator: Allocator) void {
                switch (self.*) {
                    .page => |page| page.deinit(allocator),
                    .entry => {},
                }
                allocator.destroy(self);
            }

            pub fn aabb(self: Node) AABB {
                return switch (self) {
                    .entry => |entry| EntryType.aabb(entry.entry),
                    .page => |page| page.*.aabb,
                };
            }

            pub fn setParent(self: *Node, parent: *Page) void {
                switch (self.*) {
                    .entry => |*entry| entry.parent = parent,
                    .page => |*page| page.*.parent = parent,
                }
            }

            pub fn getParent(self: *Node) ?*Page {
                return switch (self.*) {
                    .entry => |entry| entry.parent,
                    .page => |page| page.parent,
                };
            }

            pub fn remove(self: *Node, allocator: Allocator) void {
                const parent = self.getParent() orelse unreachable;
                parent.removeByAddress(allocator, self);
            }

            pub fn calculateHeight(self: *Node) usize {
                return switch (self.*) {
                    .entry => 1,
                    .page => |page| return page.calculateHeight(),
                };
            }
        };

        pub const Level = struct {
            sortedX: ArrayList(SweepLine),
            active: ArrayList(*Node),

            pub const SweepLine = union(enum) {
                left: Data,
                right: Data,

                pub const Data = struct {
                    node: *Node,
                    x: f32,
                };

                pub fn x(self: SweepLine) f32 {
                    return switch (self) {
                        .left => |left| left.node.aabb().tl.x,
                        .right => |right| right.node.aabb().br.x,
                    };
                }
            };

            pub fn init(allocator: Allocator, treeLevel: usize) *Level {
                const level = allocator.create(Level) catch unreachable;

                const childrenCapacity = std.math.pow(usize, MAX_ENTRIES_IN_NODE, treeLevel);

                level.* = Level{
                    .sortedX = ArrayList(SweepLine).initCapacity(allocator, childrenCapacity * 2) catch unreachable,
                    .active = ArrayList(*Node).initCapacity(allocator, childrenCapacity) catch unreachable,
                };

                return level;
            }

            pub fn deinit(self: *Level, allocator: Allocator) void {
                self.sortedX.deinit();
                self.active.deinit();
                allocator.destroy(self);
            }

            pub fn reset(self: *Level) void {
                self.sortedX.resize(0) catch unreachable;
            }

            pub fn write(self: *Level, nodes: []*Node) void {
                for (nodes) |node| {
                    self.writeOne(node);
                }
            }

            pub fn writeOne(self: *Level, node: *Node) void {
                const aabb = node.aabb();
                const left = SweepLine{
                    .left = .{
                        .node = node,
                        .x = aabb.tl.x,
                    },
                };
                const right = SweepLine{
                    .right = .{
                        .node = node,
                        .x = aabb.br.x,
                    },
                };

                self.sortedX.append(left) catch unreachable;
                self.sortedX.append(right) catch unreachable;
            }

            pub fn remove(self: *Level, node: *Node) void {
                var leftIndex: ?usize = null;
                var rightIndex: ?usize = null;

                for (0.., self.sortedX.items) |i, item| {
                    switch (item) {
                        .left => |left| {
                            if (left.node != node) continue;

                            leftIndex = i;
                        },
                        .right => |right| {
                            if (right.node != node) continue;

                            rightIndex = i;
                        },
                    }
                }

                if (leftIndex) |li| {
                    std.debug.assert(rightIndex != null);
                    const ri = rightIndex.?;
                    const removeFirst = @max(li, ri);
                    const removeLast = @min(li, ri);
                    _ = self.sortedX.swapRemove(removeFirst);
                    _ = self.sortedX.swapRemove(removeLast);
                }
            }

            /// Assumes that self.sortedX is populated with the correct items in any order.
            pub fn sort(self: *Level) void {
                std.mem.sort(SweepLine, self.sortedX.items, self, sortFn);
            }

            fn sortFn(self: *Level, lhs: SweepLine, rhs: SweepLine) bool {
                _ = self;
                return lhs.x() < rhs.x();
            }

            pub fn sweep(self: *Level, target: AABB) []*Node {
                const zone = ztracy.ZoneNC(@src(), "RT: level sweep", 0xff_00_00_ff);
                defer zone.End();
                self.active.resize(0) catch unreachable;

                const epsilon = 0.0001;
                const targetRightAdjusted = target.br.x - epsilon;

                for (self.sortedX.items) |line| {
                    switch (line) {
                        .left => |left| {
                            if (target.br.x < left.node.aabb().tl.x) {
                                return self.active.items;
                            }

                            self.active.append(left.node) catch unreachable;
                        },
                        .right => |right| {
                            if (targetRightAdjusted < right.node.aabb().br.x) {
                                return self.active.items;
                            }

                            if (target.tl.x < right.node.aabb().br.x) continue;

                            _ = self.active.swapRemove(std.mem.indexOfScalar(*Node, self.active.items, right.node).?);
                        },
                    }
                }

                return self.active.items;
            }
        };

        pub const Page = struct {
            aabb: AABB,
            theoreticMinialArea: f32,
            children: ArrayList(*Node),
            parent: ?*Page,

            pub fn init(allocator: Allocator, aabb: AABB, parent: ?*Page) *Page {
                const page = allocator.create(Page) catch unreachable;
                page.* = Page{
                    .aabb = aabb,
                    .theoreticMinialArea = 0,
                    .children = ArrayList(*Node).initCapacity(allocator, MAX_ENTRIES_IN_NODE + 1) catch unreachable,
                    .parent = parent,
                };

                return page;
            }

            pub fn deinit(self: *Page, allocator: Allocator) void {
                for (self.children.items) |node| {
                    node.deinit(allocator);
                }
                self.children.deinit();
                allocator.destroy(self);
            }

            pub fn ensureContains(self: *Page, aabb: AABB) void {
                self.aabb.ensureContains(aabb);
            }

            pub fn findMinAreaCost(nodes: []*Node, ensureContainsAabb: AABB) *Node {
                var minIndex: usize = undefined;
                var minDifference: f32 = std.math.inf(f32);
                var minChildArea: f32 = std.math.inf(f32);

                for (0.., nodes) |i, child| {
                    const childAabb = child.aabb();
                    const childArea = childAabb.area();

                    switch (child.*) {
                        .page => |childPage| {
                            const childDifference = childPage.*.aabb.areaDifferenceIfEnsureContains(ensureContainsAabb);

                            if (minDifference > childDifference or (minDifference == childDifference and minChildArea > childArea)) {
                                minDifference = childDifference;
                                minChildArea = childArea;
                                minIndex = i;
                            }
                        },
                        else => continue,
                    }
                }

                return nodes[minIndex];
            }

            pub fn findMinOverlapCost(nodes: []*Node, aabb: AABB) *Node {
                var minIndex: usize = undefined;
                var minDifference: f32 = std.math.inf(f32);
                var minAreaDiff: f32 = std.math.inf(f32);

                std.debug.assert(nodes.len > 0);

                if (nodes.len == 1) {
                    return nodes[0];
                }

                for (0..nodes.len - 1) |i| {
                    const child = nodes[i];
                    var childAabb = child.aabb();
                    const oldArea = childAabb.area();
                    childAabb.ensureContains(aabb);
                    const areaDiff = childAabb.area() - oldArea;

                    for (i + 1..nodes.len) |j| {
                        const other = nodes[j];
                        const otherAabb = other.aabb();
                        const intersection = childAabb.intersection(otherAabb);
                        const childDifference = if (intersection) |n| n.depth else 0;

                        if (minDifference > childDifference or (minDifference == childDifference and minAreaDiff > areaDiff)) {
                            minDifference = childDifference;
                            minAreaDiff = areaDiff;
                            minIndex = i;
                        }
                    }
                }

                std.debug.assert(minIndex >= 0 and minIndex < nodes.len);

                return nodes[minIndex];
            }

            pub fn minimalAabb(nodes: []*Node) AABB {
                var minX = std.math.inf(f32);
                var maxX = -std.math.inf(f32);
                var minY = std.math.inf(f32);
                var maxY = -std.math.inf(f32);

                for (nodes) |node| {
                    const aabb = node.aabb();
                    minX = @min(aabb.tl.x, minX);
                    maxX = @max(aabb.br.x, maxX);
                    minY = @min(aabb.tl.y, minY);
                    maxY = @max(aabb.br.y, maxY);
                }

                return AABB{
                    .tl = .{ .x = minX, .y = minY },
                    .br = .{ .x = maxX, .y = maxY },
                    .isMinimal = true,
                };
            }

            pub fn canAccomodate(self: Page) bool {
                return self.children.items.len < MAX_ENTRIES_IN_NODE;
            }

            pub fn append(self: *Page, node: *Node) void {
                self.children.append(node) catch unreachable;
                node.setParent(self);
                self.minimize();
                self.theoreticMinialArea = self.minimalAreaInTheory();
            }

            pub fn remove(self: *Page, allocator: Allocator, i: usize) void {
                const node = self.children.swapRemove(i);
                node.deinit(allocator);
            }

            pub fn removeByAddress(self: *Page, allocator: Allocator, node: *Node) void {
                const i = std.mem.indexOfScalar(*Node, self.children.items, node) orelse unreachable;
                self.remove(allocator, i);
            }

            fn minimalAreaInTheory(self: *Page) f32 {
                var width: f32 = 0;
                var height: f32 = 0;

                for (self.children.items) |item| {
                    const aabb = item.aabb();
                    width += aabb.width();
                    height += aabb.height();
                }

                return width * height;
            }

            fn maximumAreaThreshold(self: *Page) f32 {
                return self.minimalAreaInTheory() * MAXIMUM_AREA_THRESHOLD_FACTOR;
            }

            fn hasExceededMaximumAreaThreshold(self: *Page) bool {
                return self.aabb.area() > self.maximumAreaThreshold();
            }

            pub fn sort(
                self: *Page,
                comptime lessThanFn: fn (*Page, lhs: *Node, rhs: *Node) bool,
            ) void {
                std.mem.sort(*Node, self.children.items, self, lessThanFn);
            }

            pub fn minimize(self: *Page) void {
                self.aabb = Page.minimalAabb(self.children.items);
            }

            fn minArea(self: *Page, lhs: *Node, rhs: *Node) bool {
                _ = self;

                const aabbA = lhs.aabb();
                const aabbB = rhs.aabb();

                const areaA = aabbA.area();
                const areaB = aabbB.area();

                return areaA < areaB;
            }

            pub fn minAxisMinFn(comptime axis: Axis) fn (*Page, *Node, *Node) bool {
                if (axis == .x) {
                    return Page.minXMin;
                } else {
                    return Page.minYMin;
                }
            }

            pub fn minAxisMaxFn(comptime axis: Axis) fn (*Page, *Node, *Node) bool {
                if (axis == .x) {
                    return Page.minXMax;
                } else {
                    return Page.minYMax;
                }
            }

            fn minXMin(self: *Page, lhs: *Node, rhs: *Node) bool {
                return self.minAxisMin(lhs, rhs, .x);
            }

            fn minXMax(self: *Page, lhs: *Node, rhs: *Node) bool {
                return self.minAxisMax(lhs, rhs, .x);
            }

            fn minYMin(self: *Page, lhs: *Node, rhs: *Node) bool {
                return self.minAxisMin(lhs, rhs, .y);
            }

            fn minYMax(self: *Page, lhs: *Node, rhs: *Node) bool {
                return self.minAxisMax(lhs, rhs, .y);
            }

            fn minAxisMin(self: *Page, lhs: *Node, rhs: *Node, comptime axis: Axis) bool {
                _ = self;

                const aabbA = lhs.aabb();
                const aabbB = rhs.aabb();

                if (axis == .x) {
                    return aabbA.tl.x < aabbB.tl.x;
                } else {
                    return aabbA.tl.y < aabbB.tl.y;
                }
            }

            fn minAxisMax(self: *Page, lhs: *Node, rhs: *Node, comptime axis: Axis) bool {
                _ = self;

                const aabbA = lhs.aabb();
                const aabbB = rhs.aabb();

                if (axis == .x) {
                    return aabbA.br.x < aabbB.br.x;
                } else {
                    return aabbA.br.y < aabbB.br.y;
                }
            }

            fn minDistanceToCenterOfPage(self: *Page, lhs: *Node, rhs: *Node) bool {
                const aabbA = lhs.aabb();
                const aabbB = rhs.aabb();

                const distA = aabbA.distanceSq(self.aabb);
                const distB = aabbB.distanceSq(self.aabb);

                return distA < distB;
            }

            fn maxDistanceToCenterOfPage(self: *Page, lhs: *Node, rhs: *Node) bool {
                const aabbA = lhs.aabb();
                const aabbB = rhs.aabb();

                const distA = aabbA.distanceSq(self.aabb);
                const distB = aabbB.distanceSq(self.aabb);

                return distA > distB;
            }

            pub fn calculateHeight(self: *Page) usize {
                return 1 + self.children.items[0].calculateHeight();
            }
        };

        pub const Intersection = struct {
            entry: *EntryType,
            depth: f32,
            axis: zlm.Vec2,
        };

        pub fn init(allocator: Allocator) Self {
            var rtree = Self{
                .allocator = allocator,
                .root = Page.init(allocator, AABB{ .isMinimal = true }, null),
                .levels = ArrayList(*Level).initCapacity(allocator, 30) catch unreachable,
                .height = 1,
                .keyToNodeMap = ArrayList(*Node).initCapacity(allocator, 100) catch unreachable,
                .intersectingResult = IntersectingResultArray.init(0) catch unreachable,
            };

            rtree.addLevel();

            return rtree;
        }

        pub fn deinit(self: Self) void {
            self.root.deinit(self.allocator);
            self.keyToNodeMap.deinit();
            for (self.levels.items) |level| {
                level.deinit(self.allocator);
            }
            self.levels.deinit();
        }

        /// Result is invalidated when this function is called again
        pub fn intersecting(self: *Self, aabb: AABB, skipKey: ?EntryType.KeyType) []Intersection {
            const zone = ztracy.ZoneNC(@src(), "RT: intersecting", 0xff_00_00_ff);
            defer zone.End();

            self.intersectingResult.resize(0) catch unreachable;

            //var initialIterationPages = [_]*const Page{self.root};

            self.intersectingInner(aabb, self.root, &self.intersectingResult, skipKey);
            //self.intersectingInnerIterationBased(aabb, &initialIterationPages, &self.intersectingResult, skipKey, 1);

            return self.intersectingResult.slice();
        }

        // TODO: try variant where this is not recursive, but rather returns an array of targets that need to be further resolved OR; the final result
        // Reason: Getting a list of targets will make it possible to do a sweep comparison among all pages instead of brute forcing each page, also the final step with the entries can be sweeped as well
        fn intersectingInner(self: Self, aabb: AABB, page: *const Page, result: *IntersectingResultArray, skipKey: ?EntryType.KeyType) void {
            const zone = ztracy.ZoneNC(@src(), "RT: intersecting inner", 0xff_00_00_ff);
            defer zone.End();
            for (page.*.children.items) |node| {
                const aabbZone = ztracy.ZoneNC(@src(), "RT: getting aabb", 0xff_00_00_ff);
                const entryAabb = node.aabb();
                aabbZone.End();

                const aabbIntersection = entryAabb.intersection(aabb);

                if (aabbIntersection == null) continue;

                switch (node.*) {
                    .entry => |*entry| {
                        const nullCheckZone = ztracy.ZoneNC(@src(), "checking null", 0xff_00_00_ff);
                        if (skipKey) |sk| {
                            if (sk == entry.entry.key) {
                                nullCheckZone.End();
                                continue;
                            }
                        }
                        nullCheckZone.End();
                        const entryZone = ztracy.ZoneNC(@src(), "RT: appending entry to result", 0xff_00_00_ff);
                        defer entryZone.End();
                        result.append(Intersection{
                            .depth = aabbIntersection.?.depth,
                            .axis = aabbIntersection.?.axis,
                            .entry = &entry.entry,
                        }) catch unreachable;
                    },
                    .page => |childPage| self.intersectingInner(aabb, childPage, result, skipKey),
                }
            }
        }

        fn intersectingInnerIterationBased(
            self: *Self,
            aabb: AABB,
            pages: []*const Page,
            result: *IntersectingResultArray,
            skipKey: ?EntryType.KeyType,
            treeLevel: usize,
        ) void {
            const zone = ztracy.ZoneNC(@src(), "RT: intersecting inner IT", 0xff_00_00_ff);
            defer zone.End();

            var pageArr: [100]*const Page = undefined;
            var pageLen: usize = 0;

            const candidates = self.levelSweep(aabb, treeLevel);

            for (candidates) |node| {
                if (node.getParent()) |parent| {
                    if (std.mem.indexOfScalar(*const Page, pages, parent) == null) {
                        continue;
                    }
                }

                const aabbZone = ztracy.ZoneNC(@src(), "RT: getting aabb", 0xff_00_00_ff);
                const entryAabb = node.aabb();
                aabbZone.End();

                const aabbIntersection = entryAabb.intersection(aabb);

                if (aabbIntersection == null) continue;

                switch (node.*) {
                    .entry => |*entry| {
                        const nullCheckZone = ztracy.ZoneNC(@src(), "checking null", 0xff_00_00_ff);
                        if (skipKey) |sk| {
                            if (sk == entry.entry.key) {
                                nullCheckZone.End();
                                continue;
                            }
                        }
                        nullCheckZone.End();
                        const entryZone = ztracy.ZoneNC(@src(), "RT: appending entry to result", 0xff_00_00_ff);
                        defer entryZone.End();
                        result.append(Intersection{
                            .depth = aabbIntersection.?.depth,
                            .axis = aabbIntersection.?.axis,
                            .entry = &entry.entry,
                        }) catch unreachable;
                    },
                    .page => |childPage| {
                        pageArr[pageLen] = childPage;
                        pageLen += 1;
                    },
                }
            }

            if (pageLen > 0) {
                self.intersectingInnerIterationBased(aabb, pageArr[0..pageLen], result, skipKey, treeLevel + 1);
            }
        }

        fn addLevel(self: *Self) void {
            self.levels.append(Level.init(self.allocator, self.height)) catch unreachable;

            for (self.levels.items) |level| {
                level.reset();
            }

            self.writeNodesToLevel(self.root, 1);
            self.sortLevels();
        }

        pub fn sortLevels(self: *Self) void {
            for (self.levels.items) |level| {
                level.sort();
            }
        }

        fn writeNodesToLevel(self: *Self, page: *Page, treeLevel: usize) void {
            const level = self.levels.items[treeLevel - 1];

            level.write(page.children.items);

            for (page.children.items) |child| {
                switch (child.*) {
                    .page => |childPage| self.writeNodesToLevel(childPage, treeLevel + 1),
                    .entry => return,
                }
            }
        }

        fn writeNodeToLevel(self: *Self, node: *Node, treeLevel: usize) void {
            const level = self.levels.items[treeLevel - 1];
            level.writeOne(node);
        }

        fn levelSweep(self: *Self, target: AABB, treeLevel: usize) []*Node {
            return self.levels.items[treeLevel - 1].sweep(target);
        }

        pub fn insertEntry(self: *Self, entry: EntryType) void {
            resetHasOverflowTreatedAtLevel();

            // Setting root as parent but will get overwritten when the node is inserted into a page
            const node = Node.initEntry(self.allocator, entry, self.root);
            const entryId = EntryType.id(entry);
            self.keyToNodeMap.ensureTotalCapacity(entryId + 1) catch unreachable;
            self.keyToNodeMap.expandToCapacity();
            self.keyToNodeMap.items[entryId] = node;
            self.insertNode(node, self.height);
        }

        fn insertNode(self: *Self, node: *Node, targetLevel: usize) void {
            std.log.info("insertNode", .{});
            const nodeAabb: AABB = node.aabb();
            const rootSplitNode = self.insertNodeInner(node, nodeAabb, self.root, 1, targetLevel);

            if (rootSplitNode) |rsn| {
                const newRootNode = Page.init(self.allocator, AABB{ .isMinimal = true }, null);
                const oldRootNode = self.root;
                newRootNode.append(Node.init(self.allocator, Node{ .page = oldRootNode }));
                newRootNode.append(rsn);
                self.root = newRootNode;
                self.height += 1;

                self.addLevel();
            } else {
                switch (node.*) {
                    .entry => |entry| self.writeNodeToLevel(self.keyToNodeMap.items[EntryType.id(entry.entry)], self.height),
                    .page => {},
                }
            }
        }

        fn insertNodeInner(
            self: *Self,
            node: *Node,
            nodeAabb: AABB,
            page: *Page,
            level: usize,
            targetLevel: usize,
        ) ?*Node {
            if (level == targetLevel) {
                return self.tryInsertNodeIntoPage(node, page, level);
            } else {
                return self.insertNodeInnerPage(node, nodeAabb, page, level, targetLevel);
            }
        }

        var hasOverflowTreatedAtLevelMask: u64 = 0;

        fn levelBitMask(level: usize) u64 {
            return @as(u64, 1) << (@as(u6, @intCast(level)) - 1);
        }

        fn hasOverflowTreatedAtLevel(level: usize) bool {
            std.debug.assert(level < 64);
            return hasOverflowTreatedAtLevelMask & levelBitMask(level) != 0;
        }

        fn setHasOverflowTreatedAtLevel(level: usize) void {
            std.debug.assert(level < 64);
            hasOverflowTreatedAtLevelMask |= levelBitMask(level);
        }

        fn resetHasOverflowTreatedAtLevel() void {
            hasOverflowTreatedAtLevelMask = 0;
        }

        fn insertNodeInnerPage(
            self: *Self,
            node: *Node,
            nodeAabb: AABB,
            page: *Page,
            level: usize,
            targetLevel: usize,
        ) ?*Node {
            std.log.info("chooseSubtree {}", .{level});
            const nextPage =
                if (insertAlgorithm == .linear)
                Page.findMinAreaCost(page.children.items, nodeAabb).page
            else
                self.starInsertChoosePage(page, nodeAabb, level);

            const splitNode = self.insertNodeInner(node, nodeAabb, nextPage, level + 1, targetLevel);

            if (splitNode) |sn| {
                return self.tryInsertNodeIntoPage(sn, page, level);
            } else {
                page.minimize();
                return null;
            }
        }

        fn starInsertChoosePage(self: *Self, page: *Page, entryAabb: AABB, level: usize) *Page {
            var nextPage: *Page = undefined;

            // Next level is leaf level
            if (level == self.height - 1) {
                nextPage = Page.findMinOverlapCost(page.children.items, entryAabb).page;
            } else {
                nextPage = Page.findMinAreaCost(page.children.items, entryAabb).page;
            }

            return nextPage;
        }

        fn tryInsertNodeIntoPage(self: *Self, node: *Node, page: *Page, level: usize) ?*Node {
            if (page.canAccomodate()) {
                page.append(node);
                return null;
            } else {
                return self.overflowTreatment(node, page, level);
            }
        }

        fn overflowTreatment(self: *Self, node: *Node, page: *Page, level: usize) ?*Node {
            std.log.info("overflowTreatment {} {}", .{ level, hasOverflowTreatedAtLevel(level) });

            const hasOverflowTreated = hasOverflowTreatedAtLevel(level);
            setHasOverflowTreatedAtLevel(level);

            if (insertAlgorithm == .linear) {
                return self.linearSplit(node, page);
            } else if (level != 1 and !hasOverflowTreated) {
                self.starReInsert(page, node, level);
                return null;
            } else {
                return self.starSplit(page, node);
            }
        }

        fn starReInsert(self: *Self, page: *Page, node: *Node, level: usize) void {
            std.log.info("starReInsert", .{});

            page.append(node);

            std.debug.assert(page.children.items.len == MAX_ENTRIES_IN_NODE + 1);

            page.sort(Page.maxDistanceToCenterOfPage);

            var nodesToReInsert: [STAR_P]*Node = undefined;
            for (0..STAR_P) |i| {
                nodesToReInsert[i] = page.children.swapRemove(i);
            }

            std.debug.assert(page.children.items.len == MAX_ENTRIES_IN_NODE + 1 - STAR_P);

            var i: usize = STAR_P;
            while (i > 0) {
                i -= 1;
                self.insertNode(nodesToReInsert[i], level);
            }

            page.minimize();
        }

        fn starSplit(self: *Self, page: *Page, node: *Node) *Node {
            std.log.info("starSplit", .{});
            std.debug.assert(page.children.items.len == MAX_ENTRIES_IN_NODE);
            page.children.append(node) catch unreachable;
            const axis = self.starChooseSplitAxis(page);
            return switch (axis) {
                .x => self.starSplitKnownAxis(page, .x),
                .y => self.starSplitKnownAxis(page, .y),
            };
        }

        fn starSplitKnownAxis(self: *Self, page: *Page, comptime axis: Axis) *Node {
            std.debug.assert(page.children.items.len == MAX_ENTRIES_IN_NODE + 1);

            const index = self.starChooseSplitIndex(page, axis);

            if (index.isMin) {
                page.sort(Page.minAxisMinFn(axis));
            } else {
                page.sort(Page.minAxisMaxFn(axis));
            }

            var newPage = Node.initPage(self.allocator, AABB{ .isMinimal = true }, null);

            const len = page.children.items.len;
            const start = MIN_ENTRIES_IN_NODE - 1 + index.k;
            std.debug.assert(start <= len);
            for (0..len - start) |i| {
                newPage.page.append(page.children.swapRemove(len - i - 1));
            }

            std.debug.assert(page.children.items.len == start);
            std.debug.assert(newPage.page.children.items.len == len - start);

            return newPage;
        }

        const numDistributions = MAX_ENTRIES_IN_NODE - 2 * MIN_ENTRIES_IN_NODE + 2;

        fn starChooseSplitAxis(self: *Self, page: *Page) Axis {
            std.debug.assert(page.children.items.len == MAX_ENTRIES_IN_NODE + 1);

            const SX = self.starChooseSplitAxisMeasure(page, .x, .marginSum);
            const SY = self.starChooseSplitAxisMeasure(page, .y, .marginSum);

            if (SX < SY) {
                return .x;
            } else {
                return .y;
            }
        }

        const GoodnessValueMeasurement = enum {
            marginSum,
            minOverlapDistribution,
        };

        fn starChooseSplitAxisMeasure(
            self: *Self,
            page: *Page,
            comptime axis: Axis,
            comptime measurement: GoodnessValueMeasurement,
        ) t: {
            if (measurement == .marginSum) {
                break :t f32;
            } else {
                break :t StarSplitIndexResult;
            }
        } {
            _ = self;

            const sortFns = comptime .{ Page.minAxisMinFn(axis), Page.minAxisMaxFn(axis) };

            var margin: f32 = 0;
            var minOverlap: f32 = 0;
            var minArea: f32 = 0;
            var distribution: usize = 0;
            var isMin: bool = true;

            inline for (sortFns) |sortFn| {
                page.sort(sortFn);

                std.debug.assert(page.children.items.len == MAX_ENTRIES_IN_NODE + 1);

                const fst = page.children.items[0].aabb();
                const snd = page.children.items[page.children.items.len - 1].aabb();
                const fstP = if (sortFn == sortFns[0]) fst.tl else fst.br;
                const sndP = if (sortFn == sortFns[0]) snd.tl else snd.br;
                const fstV = if (axis == .x) fstP.x else fstP.y;
                const sndV = if (axis == .x) sndP.x else sndP.y;
                std.debug.assert(fstV <= sndV);

                for (0..numDistributions) |i| {
                    const k = i + 1;

                    const groups = starGetDistribution(page.children.items, k);

                    const fAabb = Page.minimalAabb(groups.first);
                    const sAabb = Page.minimalAabb(groups.second);

                    if (measurement == .marginSum) {
                        margin += fAabb.margin() + sAabb.margin();
                    } else {
                        const intersection = fAabb.intersection(sAabb);
                        const overlap = if (intersection) |inter| inter.depth else -1;
                        const area = fAabb.area() + sAabb.area();

                        if (overlap < minOverlap or (overlap == minOverlap and area < minArea)) {
                            minArea = area;
                            minOverlap = overlap;
                            distribution = k;
                            isMin = sortFn == sortFns[0];
                        }
                    }
                }
            }

            if (measurement == .marginSum) {
                return margin;
            } else {
                return .{
                    .k = distribution,
                    .isMin = isMin,
                };
            }
        }

        fn starGetDistribution(
            nodes: []*Node,
            distribution: usize,
        ) struct { first: []*Node, second: []*Node } {
            const firstGroupLen = MIN_ENTRIES_IN_NODE - 1 + distribution;
            const firstGroup = nodes[0..firstGroupLen];
            const secondGroup = nodes[firstGroupLen..];

            return .{
                .first = firstGroup,
                .second = secondGroup,
            };
        }

        const StarSplitIndexResult = struct {
            k: usize,
            isMin: bool,
        };

        fn starChooseSplitIndex(self: *Self, page: *Page, comptime axis: Axis) StarSplitIndexResult {
            return self.starChooseSplitAxisMeasure(page, axis, .minOverlapDistribution);
        }

        fn linearSplit(self: *Self, node: *Node, page: *Page) *Node {
            std.debug.assert(page.children.items.len > 0);

            var newPage = Node.initPage(self.allocator, AABB{ .isMinimal = true }, null);

            const currentPageChildren = page.children.toOwnedSlice() catch unreachable;
            defer self.allocator.free(currentPageChildren);
            const numberOfNodesToSplit = currentPageChildren.len + 1;

            var nodesInSplit: [MAX_ENTRIES_IN_NODE + 1]?*Node = .{null} ** (MAX_ENTRIES_IN_NODE + 1);
            std.debug.assert(nodesInSplit.len >= numberOfNodesToSplit);
            for (0..currentPageChildren.len) |i| {
                nodesInSplit[i] = currentPageChildren[i];
            }
            nodesInSplit[currentPageChildren.len] = node;
            page.aabb = AABB{ .isMinimal = true };
            var numberOfNodesInSplitLeft = numberOfNodesToSplit;
            std.debug.assert(numberOfNodesToSplit == currentPageChildren.len + 1);

            var ie1: usize = undefined;
            var ie2: usize = undefined;

            const maxDistSlice = nodesInSplit[0..numberOfNodesToSplit];
            std.debug.assert(maxDistSlice.len > 1);
            findMaximumDistanceO(maxDistSlice, &ie1, &ie2);

            std.debug.assert(ie1 != ie2);

            const e1 = nodesInSplit[ie1] orelse unreachable;
            const e2 = nodesInSplit[ie2] orelse unreachable;
            nodesInSplit[ie1] = null;
            nodesInSplit[ie2] = null;
            numberOfNodesInSplitLeft -= 2;

            std.debug.assert(page.children.items.len == 0);
            std.debug.assert(newPage.page.children.items.len == 0);

            page.append(e1, &self.keyToNodeMap);
            newPage.page.append(e2, &self.keyToNodeMap);

            const numberOfNonSeedNodes = numberOfNodesInSplitLeft;

            var nn: usize = 0;
            for (nodesInSplit) |nis| {
                if (nis) |_| {
                    nn += 1;
                }
            }

            std.debug.assert(numberOfNonSeedNodes == nn);
            std.debug.assert(numberOfNonSeedNodes == MAX_ENTRIES_IN_NODE - 1);

            for (0..nodesInSplit.len) |i| {
                if (newPage.page.children.items.len <= MIN_ENTRIES_IN_NODE -| numberOfNodesInSplitLeft) {
                    for (nodesInSplit) |maybeNode| {
                        if (maybeNode) |nodeToSplit| {
                            newPage.page.append(nodeToSplit, &self.keyToNodeMap);
                            numberOfNodesInSplitLeft -= 1;
                        }
                    }
                    break;
                } else if (page.children.items.len <= MIN_ENTRIES_IN_NODE -| numberOfNodesInSplitLeft) {
                    for (nodesInSplit) |maybeNode| {
                        if (maybeNode) |nodeToSplit| {
                            page.append(nodeToSplit, &self.keyToNodeMap);
                            numberOfNodesInSplitLeft -= 1;
                        }
                    }
                    break;
                } else {
                    const e = nodesInSplit[i] orelse continue;
                    nodesInSplit[i] = null;
                    const eAabb = e.aabb();
                    const currentPageDiff = page.aabb.areaDifferenceIfEnsureContains(eAabb);
                    const newPageDiff = newPage.page.aabb.areaDifferenceIfEnsureContains(eAabb);

                    if (currentPageDiff < newPageDiff) {
                        page.append(e, &self.keyToNodeMap);
                    } else if (newPageDiff < currentPageDiff) {
                        newPage.page.append(e, &self.keyToNodeMap);
                    } else if (page.children.items.len < newPage.page.children.items.len) {
                        page.append(e, &self.keyToNodeMap);
                    } else {
                        newPage.page.append(e, &self.keyToNodeMap);
                    }

                    numberOfNodesInSplitLeft -= 1;
                }
            }

            std.debug.assert(numberOfNodesInSplitLeft == 0);

            return newPage;
        }

        fn findMaximumDistanceO(nodeList: []?*Node, ie1: *usize, ie2: *usize) void {
            var maxDist = -std.math.inf(f32);
            var maxA: usize = undefined;
            var maxB: usize = undefined;

            for (0..nodeList.len - 1) |i| {
                const itemA = nodeList[i] orelse unreachable;

                for (i + 1..nodeList.len) |j| {
                    const itemB = nodeList[j] orelse unreachable;
                    const distance = itemA.aabb().distanceSq(itemB.aabb());

                    if (distance > maxDist) {
                        maxDist = distance;
                        maxA = i;
                        maxB = j;
                    }
                }
            }

            ie1.* = maxA;
            ie2.* = maxB;
        }

        // TODO: Implement an optimizer that will trigger on certain conditions when updating an entry
        // The goal should be to minimize overlap between the page siblings
        pub fn updateEntry(self: *Self, entry: EntryType) void {
            const entryId = EntryType.id(entry);
            const node = self.keyToNodeMap.items[entryId];

            const aabb = node.aabb();

            self.ensureContainsBottomUp(node.getParent(), aabb);
        }

        fn ensureContainsBottomUp(self: *Self, page: ?*Page, aabb: AABB) void {
            if (page == null) return;

            page.?.ensureContains(aabb);

            if (page.?.parent) |parent| {
                self.ensureContainsBottomUp(parent, aabb);
            }
        }

        pub fn removeEntry(self: *Self, entry: EntryType) void {
            const entryId = EntryType.id(entry);
            const node = self.keyToNodeMap.items[entryId];
            self.levels.items[self.height - 1].remove(node);
            node.remove(self.allocator);
        }
    };
}
