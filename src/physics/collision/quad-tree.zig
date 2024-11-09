const std = @import("std");
const Allocator = std.mem.Allocator;
const ArrayList = std.ArrayList;
const zlm = @import("zlm");

const AABB = @import("../shape.zig").AABB;
const Intersection = @import("intersection.zig").Intersection;

// 2D Quad Tree
//
// Global Collision Phase
//     1. For each row of squares at the first level (largest squares)
//          2. Sweep and prune on all bodies that belong to the row
//
// Find Intersections Within a Target (Rectangle / Circle)
//     1. Quad Tree intersects algorithm

const INITIAL_NODE_CAPACITY = 1000;
const MAX_ENTRIES_IN_PAGE = 10;
// Entries + page itself
const PAGE_SIZE = MAX_ENTRIES_IN_PAGE + 1;

pub fn QuadTree(comptime T: type, comptime getEntryAabb: fn (*T) AABB) type {
    return struct {
        const QT = QuadTree(T, getEntryAabb);

        pub const Node = union(enum) {
            entry: Entry,
            page: Page,

            pub fn initEntry(value: *T) Node {
                return Node{
                    .entry = Entry.init(value),
                };
            }

            pub fn initPage(aabb: AABB, children: []Node) Node {
                return Node{
                    .page = Page.init(aabb, children),
                };
            }
        };

        pub const Entry = struct {
            value: *T,
            aabb: AABB,

            pub fn init(value: *T) Entry {
                return Entry{
                    .value = value,
                    .aabb = getEntryAabb(value),
                };
            }
        };

        fn Subdivision(comptime SDT: type) type {
            return struct {
                tl: SDT = undefined,
                tr: SDT = undefined,
                bl: SDT = undefined,
                br: SDT = undefined,
            };
        }

        pub const Page = struct {
            aabb: AABB,
            entries: BufferArrayList(Node),
            quadrants: ?Subdivision(*Node) = null,

            pub fn init(aabb: AABB, entriesBuffer: []Node) Page {
                return Page{
                    .aabb = aabb,
                    .entries = BufferArrayList(Node).init(entriesBuffer),
                };
            }

            pub fn canAccomodate(self: Page) bool {
                return self.entries.len < MAX_ENTRIES_IN_PAGE;
            }

            pub fn canFit(self: Page, aabb: AABB) bool {
                return self.aabb.contains(aabb);
            }

            pub fn subdivisions(self: Page) Subdivision(AABB) {
                const halfSize = self.aabb.size().scale(0.5);
                const halfX = halfSize.x;
                const halfY = halfSize.y;

                return Subdivision(AABB){
                    .tl = AABB{
                        .tl = self.aabb.tl,
                        .br = self.aabb.br.sub(halfSize),
                        .isMinimal = false,
                    },
                    .tr = AABB{
                        .tl = self.aabb.tl.add(zlm.vec2(halfX, 0)),
                        .br = self.aabb.br.add(zlm.vec2(0, -halfY)),
                        .isMinimal = false,
                    },
                    .bl = AABB{
                        .tl = self.aabb.tl.add(zlm.vec2(0, halfY)),
                        .br = self.aabb.br.add(zlm.vec2(-halfX, 0)),
                        .isMinimal = false,
                    },
                    .br = AABB{
                        .tl = self.aabb.tl.add(halfSize),
                        .br = self.aabb.br,
                        .isMinimal = false,
                    },
                };
            }
        };

        nodes: ArrayList(Node),
        currentIndex: usize = 0,
        intersectingBuffer: ArrayList(Intersection(*T)),

        pub fn init(allocator: Allocator) QT {
            return QT{
                .nodes = ArrayList(Node).initCapacity(allocator, INITIAL_NODE_CAPACITY) catch unreachable,
                .intersectingBuffer = ArrayList(Intersection(*T)).initCapacity(allocator, INITIAL_NODE_CAPACITY / 2) catch unreachable,
            };
        }

        pub fn deinit(self: QT) void {
            self.nodes.deinit();
            self.intersectingBuffer.deinit();
        }

        pub fn reset(self: *QT) void {
            self.nodes.resize(0) catch unreachable;
            self.currentIndex = 0;
        }

        pub fn getRoot(self: QT) *Node {
            return &self.nodes.items[0];
        }

        pub fn populate(self: *QT, boundary: AABB, entries: []T) void {
            self.reset();

            const requiredPages = 4 * (entries.len / MAX_ENTRIES_IN_PAGE + 1);
            self.preparePagesToAdd(requiredPages);

            _ = self.addPage(boundary);

            for (entries) |*entry| {
                _ = self.insert(entry);
            }
        }

        pub fn insert(self: *QT, entry: *T) bool {
            if (self.currentIndex == 0) return false;
            const entryAabb = getEntryAabb(entry);
            return self.insertIntoPage(&self.getRoot().page, entry, entryAabb);
        }

        pub fn intersecting(self: *QT, aabb: AABB) []Intersection(*T) {
            self.intersectingBuffer.resize(0) catch unreachable;
            if (self.currentIndex == 0) return &.{};
            self.intersectingForPage(aabb, &self.getRoot().page);
            return self.intersectingBuffer.items;
        }

        fn intersectingForPage(self: *QT, aabb: AABB, page: *Page) void {
            if (!page.aabb.intersects(aabb)) return;

            for (page.entries.items()) |entry| {
                switch (entry) {
                    .page => {
                        std.log.info("encountered page", .{});
                    },
                    .entry => {
                        if (aabb.intersection(entry.entry.aabb)) |intersection| {
                            self.intersectingBuffer.append(Intersection(*T){
                                .entry = entry.entry.value,
                                .axis = intersection.axis,
                                .depth = intersection.depth,
                            }) catch unreachable;
                        }
                    },
                }
            }

            if (page.quadrants) |quadrants| {
                self.intersectingForPage(aabb, &quadrants.tl.page);
                self.intersectingForPage(aabb, &quadrants.tr.page);
                self.intersectingForPage(aabb, &quadrants.bl.page);
                self.intersectingForPage(aabb, &quadrants.br.page);
            }
        }

        fn insertIntoPage(self: *QT, page: *Page, entry: *T, entryAabb: AABB) bool {
            if (!page.canFit(entryAabb)) return false;

            if (page.quadrants) |_| {
                if (self.insertToChild(page, entry, entryAabb)) return true;

                if (!page.canAccomodate()) return false;

                page.entries.append(Node.initEntry(entry));
                return true;
            } else if (page.canAccomodate()) {
                page.entries.append(Node.initEntry(entry));
                return true;
            } else {
                _ = self.subdivide(page);
                return self.insertToChild(page, entry, entryAabb);
            }
        }

        fn insertToChild(self: *QT, page: *Page, entry: *T, entryAabb: AABB) bool {
            return self.insertIntoPage(&page.quadrants.?.tl.page, entry, entryAabb) or
                self.insertIntoPage(&page.quadrants.?.tr.page, entry, entryAabb) or
                self.insertIntoPage(&page.quadrants.?.bl.page, entry, entryAabb) or
                self.insertIntoPage(&page.quadrants.?.br.page, entry, entryAabb);
        }

        fn subdivide(self: *QT, page: *Page) Subdivision(*Node) {
            const aabbSubdivision = page.subdivisions();

            self.preparePagesToAdd(4);

            const subdivision = Subdivision(*Node){
                .tl = self.addPagePrepared(aabbSubdivision.tl),
                .tr = self.addPagePrepared(aabbSubdivision.tr),
                .bl = self.addPagePrepared(aabbSubdivision.bl),
                .br = self.addPagePrepared(aabbSubdivision.br),
            };

            page.quadrants = subdivision;

            return subdivision;
        }

        fn nodesSlice(self: *QT, offset: usize, len: usize) []Node {
            return self.nodes.items[offset .. offset + len];
        }

        fn addPage(self: *QT, boundary: AABB) *Node {
            self.preparePagesToAdd(1);
            return self.addPagePrepared(boundary);
        }

        fn preparePagesToAdd(self: *QT, numberOfPages: usize) void {
            const pagesToPrepare = numberOfPages - @min(numberOfPages, self.availablePages());
            if (pagesToPrepare == 0) return;
            self.nodes.resize(self.nodes.items.len + PAGE_SIZE * pagesToPrepare) catch unreachable;
        }

        fn availablePages(self: *QT) usize {
            return @divExact((self.nodes.items.len - self.currentIndex), PAGE_SIZE);
        }

        fn addPagePrepared(self: *QT, boundary: AABB) *Node {
            std.debug.assert(self.nodes.items.len - self.currentIndex >= PAGE_SIZE);
            const entriesBuffer = self.nodesSlice(self.currentIndex + 1, MAX_ENTRIES_IN_PAGE);
            const page = Node.initPage(boundary, entriesBuffer);
            self.nodes.items[self.currentIndex] = page;
            const pagePtr = &self.nodes.items[self.currentIndex];
            self.currentIndex += PAGE_SIZE;
            return pagePtr;
        }
    };
}

pub fn BufferArrayList(comptime T: type) type {
    return struct {
        buffer: []T,
        len: usize,

        const BAL = @This();

        pub fn init(buffer: []T) BAL {
            return BAL{
                .buffer = buffer,
                .len = 0,
            };
        }

        pub fn append(self: *BAL, element: T) void {
            std.debug.assert(self.len < self.buffer.len);
            self.buffer[self.len] = element;
            self.len += 1;
        }

        pub fn swapRemove(self: *BAL, i: usize) T {
            std.debug.assert(i < self.buffer.len);
            const element = self.buffer[i];
            self.buffer[i] = self.buffer[self.len - 1];
            self.len -= 1;
            return element;
        }

        pub fn items(self: BAL) []T {
            return self.buffer[0..self.len];
        }
    };
}
