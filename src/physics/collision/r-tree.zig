const std = @import("std");
const Allocator = std.mem.Allocator;
const ArrayList = std.ArrayList;
const zlm = @import("zlm");

const AABB = @import("../shape.zig").AABB;

const Point = struct { x: f32, y: f32 };
const Axis = std.meta.FieldEnum(Point);
fn getAxisValue(self: Point, axis: Axis) f32 {
    return @field(self, @tagName(axis)).*;
}

const INTERSECTING_INITIAL_CAPACITY = 10;
const MAX_ENTRIES_IN_NODE = 10;
const MIN_ENTRIES_IN_NODE = 2;

/// Extends type must implement method:
/// ```zig
/// fn aabb(self: @This()) AABB
///
/// // Example:
///
/// const MyEntryExtension = struct {
///     pub fn aabb(self: Entry(Key, Value)) AABB {
///         return self.key.aabb();
///     }
/// };
///
/// const MyEntry = Entry(MyKey, MyValue, MyEntryExtension);
/// ```
pub fn Entry(comptime Key: type, comptime Value: type, comptime Extends: type) type {
    return struct {
        key: Key,
        value: Value,

        usingnamespace Extends;
    };
}

/// EntryType must be of type `Entry(K, V, E)`
pub fn RTree(comptime EntryType: type) type {
    return struct {
        allocator: Allocator,
        root: *Page,
        height: usize,
        keyToNodeMap: ArrayList(*Node),

        const Self = @This();

        const Node = union(enum) {
            entry: NodeEntry,
            page: *Page,

            pub const NodeEntry = struct {
                entry: EntryType,
                parent: *Page,
            };

            pub fn initEntry(entry: EntryType, parent: *Page) Node {
                return Node{
                    .entry = NodeEntry{ .entry = entry, .parent = parent },
                };
            }

            pub fn initPage(allocator: Allocator, pageAabb: AABB, parent: ?*Page) Node {
                return Node{
                    .page = Page.init(allocator, pageAabb, parent),
                };
            }

            pub fn deinit(self: Node, allocator: Allocator) void {
                switch (self) {
                    .page => |page| page.deinit(allocator),
                    .entry => {},
                }
            }

            pub fn aabb(self: Node) AABB {
                return switch (self) {
                    .entry => |entry| entry.entry.aabb(),
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
        };

        const Page = struct {
            aabb: AABB,
            children: ArrayList(Node),
            parent: ?*Page,

            pub fn init(allocator: Allocator, aabb: AABB, parent: ?*Page) *Page {
                const page = allocator.create(Page) catch unreachable;
                page.* = Page{
                    .aabb = aabb,
                    .children = ArrayList(Node).initCapacity(allocator, MAX_ENTRIES_IN_NODE) catch unreachable,
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

            pub fn findMinAabb(self: Page, ensureContainsAabb: AABB) *Node {
                var minIndex: usize = undefined;
                var minDifference: f32 = std.math.inf(f32);

                for (0.., self.children.items) |i, child| {
                    switch (child) {
                        .page => |childPage| {
                            const childDifference = childPage.*.aabb.areaDifferenceIfEnsureContains(ensureContainsAabb);

                            if (minDifference > childDifference) {
                                minDifference = childDifference;
                                minIndex = i;
                            }
                        },
                        else => continue,
                    }
                }

                return &self.children.items[minIndex];
            }

            pub fn canAccomodate(self: Page) bool {
                return self.children.items.len < MAX_ENTRIES_IN_NODE;
            }

            pub fn append(self: *Page, node: Node, keyToNodeMap: *ArrayList(*Node)) void {
                self.children.append(node) catch unreachable;
                const appendedNode = &self.children.items[self.children.items.len - 1];
                appendedNode.setParent(self);

                switch (appendedNode.*) {
                    .entry => |entry| {
                        keyToNodeMap.ensureTotalCapacity(entry.entry.key) catch unreachable;
                        keyToNodeMap.expandToCapacity();
                        keyToNodeMap.items[entry.entry.key] = appendedNode;
                    },
                    .page => {},
                }
            }

            pub fn remove(self: *Page, allocator: Allocator, i: usize) void {
                const node = self.children.swapRemove(i);
                node.deinit(allocator);
            }

            pub fn removeByAddress(self: *Page, allocator: Allocator, node: *Node) void {
                const i = std.mem.indexOfScalar(*Node, self.children.items, node) orelse unreachable;
                self.remove(allocator, i);
            }
        };

        pub fn init(allocator: Allocator) Self {
            return Self{
                .allocator = allocator,
                .root = Page.init(allocator, AABB{}, null),
                .height = 1,
                .keyToNodeMap = ArrayList(*Node).initCapacity(allocator, 100) catch unreachable,
            };
        }

        pub fn deinit(self: Self) void {
            self.root.deinit(self.allocator);
            self.keyToNodeMap.deinit();
        }

        /// Caller owns returned array
        pub fn intersecting(self: Self, aabb: AABB) []*EntryType {
            var result = ArrayList(*EntryType).initCapacity(self.allocator, INTERSECTING_INITIAL_CAPACITY) catch unreachable;

            self.intersectingInner(aabb, self.root, &result);

            return result.toOwnedSlice() catch unreachable;
        }

        fn intersectingInner(self: Self, aabb: AABB, page: *const Page, result: *ArrayList(*EntryType)) void {
            for (page.*.children.items) |*node| {
                const entryAabb = node.aabb();

                if (!entryAabb.intersects(aabb)) continue;

                switch (node.*) {
                    .entry => |*entry| result.append(&entry.entry) catch unreachable,
                    .page => |childPage| self.intersectingInner(aabb, childPage, result),
                }
            }
        }

        pub fn insertEntry(self: *Self, entry: EntryType) void {
            const entryAabb: AABB = entry.aabb();
            const rootSplitNode = self.insertEntryInner(entry, entryAabb, self.root, 1);

            if (rootSplitNode) |rsn| {
                const newRootNode = Page.init(self.allocator, AABB{}, null);
                const oldRootNode = self.root;
                newRootNode.append(Node{ .page = oldRootNode }, &self.keyToNodeMap);
                newRootNode.append(rsn, &self.keyToNodeMap);
                self.root = newRootNode;
                self.root.ensureContains(oldRootNode.aabb);
                self.root.ensureContains(rsn.aabb());
            }
        }

        fn insertEntryInner(self: *Self, entry: EntryType, entryAabb: AABB, page: *Page, level: usize) ?Node {
            if (level == self.height) {
                return self.insertEntryIntoLeaf(entry, entryAabb, page);
            } else {
                const minDifferencePage = page.findMinAabb(entryAabb).page;

                const splitNode = self.insertEntryInner(entry, entryAabb, minDifferencePage, level + 1);

                if (splitNode) |sn| {
                    return self.insertNodeIntoPage(sn, page);
                } else {
                    page.ensureContains(entryAabb);
                    return null;
                }
            }
        }

        fn insertNodeIntoPage(self: *Self, node: Node, page: *Page) ?Node {
            if (page.canAccomodate()) {
                page.append(node, &self.keyToNodeMap);
                page.ensureContains(node.aabb());
                return null;
            } else {
                return self.linearSplit(node, page);
            }
        }

        fn insertEntryIntoLeaf(self: *Self, entry: EntryType, entryAabb: AABB, leaf: *Page) ?Node {
            const entryNode = Node.initEntry(entry, leaf);

            if (leaf.canAccomodate()) {
                leaf.append(entryNode, &self.keyToNodeMap);
                leaf.ensureContains(entryAabb);
                return null;
            } else {
                return self.linearSplit(entryNode, leaf);
            }
        }

        fn linearSplit(self: *Self, node: Node, page: *Page) Node {
            var newPage = Page.init(self.allocator, AABB{}, null);

            const currentPageChildren = page.children.toOwnedSlice() catch unreachable;
            defer self.allocator.free(currentPageChildren);
            const numberOfNodesToSplit = currentPageChildren.len + 1;

            var nodesInSplit: [MAX_ENTRIES_IN_NODE + 1]?Node = undefined;
            nodesInSplit[0] = node;
            for (0..currentPageChildren.len) |i| {
                nodesInSplit[i + 1] = currentPageChildren[i];
            }
            page.aabb = AABB{};
            var numberOfNodesInSplitLeft = numberOfNodesToSplit;

            var ie1: usize = undefined;
            var ie2: usize = undefined;

            findMaximumDistance(nodesInSplit[0..numberOfNodesToSplit], &ie1, &ie2);

            const e1 = nodesInSplit[ie1] orelse unreachable;
            const e2 = nodesInSplit[ie2] orelse unreachable;
            numberOfNodesInSplitLeft -= 2;

            page.append(e1, &self.keyToNodeMap);
            newPage.append(e2, &self.keyToNodeMap);

            page.ensureContains(e1.aabb());
            newPage.ensureContains(e2.aabb());

            for (0..nodesInSplit.len) |i| {
                if (newPage.children.items.len <= MIN_ENTRIES_IN_NODE - numberOfNodesInSplitLeft) {
                    for (0..numberOfNodesToSplit) |j| {
                        const item = nodesInSplit[j] orelse unreachable;
                        newPage.append(item, &self.keyToNodeMap);
                        newPage.ensureContains(item.aabb());
                    }
                    break;
                } else if (page.children.items.len <= MIN_ENTRIES_IN_NODE - numberOfNodesInSplitLeft) {
                    for (0..numberOfNodesToSplit) |j| {
                        const item = nodesInSplit[j] orelse continue;
                        page.append(item, &self.keyToNodeMap);
                        page.ensureContains(item.aabb());
                    }
                    break;
                } else {
                    const e = nodesInSplit[i] orelse continue;
                    const eAabb = e.aabb();
                    const currentPageDiff = page.aabb.areaDifferenceIfEnsureContains(eAabb);
                    const newPaggeDiff = newPage.aabb.areaDifferenceIfEnsureContains(eAabb);

                    if (currentPageDiff < newPaggeDiff) {
                        page.append(e, &self.keyToNodeMap);
                        page.ensureContains(eAabb);
                    } else if (newPaggeDiff < currentPageDiff) {
                        newPage.append(e, &self.keyToNodeMap);
                        newPage.ensureContains(eAabb);
                    } else if (page.children.items.len < newPage.children.items.len) {
                        page.append(e, &self.keyToNodeMap);
                        page.ensureContains(eAabb);
                    } else {
                        newPage.append(e, &self.keyToNodeMap);
                        newPage.ensureContains(eAabb);
                    }

                    numberOfNodesInSplitLeft -= 1;
                }
            }

            return Node{ .page = newPage };
        }

        fn findMaximumDistance(nodeList: []?Node, ie1: *usize, ie2: *usize) void {
            var maxDist = -std.math.inf(f32);
            var maxA: usize = undefined;
            var maxB: usize = undefined;

            for (0..nodeList.len - 1) |i| {
                const itemA = nodeList[i] orelse unreachable;

                for (i..nodeList.len) |j| {
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

        pub fn updateEntry(self: *Self, entry: EntryType) void {
            const node = self.keyToNodeMap.items[entry.key];
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
            const node = self.keyToNodeMap.items[entry.key];
            node.remove(self.allocator);
        }
    };
}
