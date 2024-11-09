const std = @import("std");
const Allocator = std.mem.Allocator;
const ArrayList = std.ArrayList;
const zlm = @import("zlm");
const AABB = @import("../../shape.zig").AABB;
const cfg = @import("cfg.zig");
const Axis = @import("axis.zig").Axis;

pub fn GNode(
    comptime EntryType: type,
    comptime getEntryAabb: fn (EntryType) AABB,
) type {
    return union(enum) {
        entry: NodeEntry,
        page: *Page,

        pub const NodeEntry = struct {
            entry: EntryType,
            parent: *Page,
            hasUpdated: bool,
            lastDistanceToCenter: f32,
            lastCenter: zlm.Vec2,

            pub fn setUpdateFlag(self: *NodeEntry) void {
                self.hasUpdated = true;
            }

            pub fn clearUpdateFlag(self: *NodeEntry) void {
                self.hasUpdated = false;
            }
        };

        pub const Node = @This();

        pub const Page = struct {
            aabb: AABB,
            theoreticMinialArea: f32,
            children: ArrayList(*Node),
            parent: ?*Page,
            hasUpdated: bool,

            pub fn init(allocator: Allocator, parent: ?*Page) *Page {
                const page = allocator.create(Page) catch unreachable;
                page.* = Page{
                    .aabb = AABB{ .isMinimal = true },
                    .theoreticMinialArea = 0,
                    .children = ArrayList(*Node).initCapacity(allocator, cfg.MAX_ENTRIES_IN_NODE + 1) catch unreachable,
                    .parent = parent,
                    .hasUpdated = false,
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

            pub fn setUpdateFlag(self: *Page) void {
                self.hasUpdated = true;
            }

            pub fn clearUpdateFlag(self: *Page) void {
                self.hasUpdated = false;
            }

            pub fn ensureContains(self: *Page, aabb: AABB) void {
                self.aabb.ensureContains(aabb);
            }

            pub fn minimalAabb(nodes: []*Node) AABB {
                var minX = std.math.inf(f32);
                var maxX = -std.math.inf(f32);
                var minY = std.math.inf(f32);
                var maxY = -std.math.inf(f32);

                for (nodes) |node| {
                    const aabb = node.getAabb();
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
                return self.children.items.len < cfg.MAX_ENTRIES_IN_NODE;
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

            pub fn removeNoDeinit(self: *Page, i: usize) *Node {
                return self.children.swapRemove(i);
            }

            pub fn removeByAddress(self: *Page, allocator: Allocator, node: *Node) void {
                const i = std.mem.indexOfScalar(*Node, self.children.items, node) orelse unreachable;
                self.remove(allocator, i);
            }

            pub fn removeByPage(self: *Page, allocator: Allocator, page: *Page) void {
                for (0..self.children.items.len) |i| {
                    if (self.children.items[i].page == page) {
                        self.remove(allocator, i);
                        return;
                    }
                }
            }

            pub fn removeByAddressNoDeinit(self: *Page, allocator: Allocator, node: *Node) void {
                const i = std.mem.indexOfScalar(*Node, self.children.items, node) orelse unreachable;
                self.removeNoDeinit(allocator, i);
            }

            fn minimalAreaInTheory(self: *Page) f32 {
                var width: f32 = 0;
                var height: f32 = 0;

                for (self.children.items) |item| {
                    const aabb = item.getAabb();
                    width += aabb.width();
                    height += aabb.height();
                }

                return width * height;
            }

            fn maximumAreaThreshold(self: *Page) f32 {
                return self.minimalAreaInTheory() * cfg.MAXIMUM_AREA_THRESHOLD_FACTOR;
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

                const aabbA = lhs.getAabb();
                const aabbB = rhs.getAabb();

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

                const aabbA = lhs.getAabb();
                const aabbB = rhs.getAabb();

                if (axis == .x) {
                    return aabbA.tl.x < aabbB.tl.x;
                } else {
                    return aabbA.tl.y < aabbB.tl.y;
                }
            }

            fn minAxisMax(self: *Page, lhs: *Node, rhs: *Node, comptime axis: Axis) bool {
                _ = self;

                const aabbA = lhs.getAabb();
                const aabbB = rhs.getAabb();

                if (axis == .x) {
                    return aabbA.br.x < aabbB.br.x;
                } else {
                    return aabbA.br.y < aabbB.br.y;
                }
            }

            fn minDistanceToCenterOfPage(self: *Page, lhs: *Node, rhs: *Node) bool {
                const aabbA = lhs.getAabb();
                const aabbB = rhs.getAabb();

                const distA = aabbA.distanceSq(self.aabb);
                const distB = aabbB.distanceSq(self.aabb);

                return distA < distB;
            }

            pub fn maxDistanceToCenterOfPage(self: *Page, lhs: *Node, rhs: *Node) bool {
                const aabbA = lhs.getAabb();
                const aabbB = rhs.getAabb();

                const distA = aabbA.distanceSq(self.aabb);
                const distB = aabbB.distanceSq(self.aabb);

                return distA > distB;
            }

            pub fn calculateHeight(self: *Page) usize {
                return 1 + self.children.items[0].calculateHeight();
            }
        };

        pub fn init(allocator: Allocator, node: Node) *Node {
            const nodePtr = allocator.create(Node) catch unreachable;
            nodePtr.* = node;
            return nodePtr;
        }

        pub fn initEntry(allocator: Allocator, entry: EntryType, parent: *Page) *Node {
            return Node.init(allocator, Node{
                .entry = NodeEntry{
                    .entry = entry,
                    .parent = parent,
                    .hasUpdated = false,
                    .lastDistanceToCenter = 0,
                    .lastCenter = zlm.Vec2.zero,
                },
            });
        }

        pub fn initPage(allocator: Allocator, parent: ?*Page) *Node {
            return Node.init(allocator, Node{
                .page = Page.init(allocator, parent),
            });
        }

        pub fn deinit(self: *Node, allocator: Allocator) void {
            switch (self.*) {
                .page => |page| page.deinit(allocator),
                .entry => {},
            }
            allocator.destroy(self);
        }

        pub fn setUpdateFlag(self: *Node) void {
            switch (self.*) {
                .page => |page| page.setUpdateFlag(),
                .entry => |*entry| entry.setUpdateFlag(),
            }
        }

        pub fn clearUpdateFlag(self: *Node) void {
            switch (self.*) {
                .page => |page| page.clearUpdateFlag(),
                .entry => |entry| entry.clearUpdateFlag(),
            }
        }

        pub fn getAabb(self: Node) AABB {
            return switch (self) {
                .entry => |entry| getEntryAabb(entry.entry),
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
}
