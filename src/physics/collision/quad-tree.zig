const std = @import("std");
const Allocator = std.mem.Allocator;
const ArrayList = std.ArrayList;

// const ztracy = @import("ztracy");

const V = @import("../../vector.zig").V;
const Vector = @import("../../vector.zig").Vector;

const AABB = @import("../shape.zig").AABB;
const Intersection = @import("intersection.zig").Intersection;

// 2D Quad Tree
//
// Global Collision Phase
//     1. For each body, call intersect on quad tree
//     2. When populating the quad tree, also check for intersections for each insert
//
// Find Intersections Within a Target (Rectangle / Circle)
//     1. Quad Tree intersects algorithm

const INITIAL_ENTRIES_CAPACITY = MAX_ENTRIES_IN_PAGE * INITIAL_PAGES_CAPACITY;
const INITIAL_PAGES_CAPACITY = 100;
const MAX_ENTRIES_IN_PAGE = 10;
/// How many levels to make space for after the pages needed calculation depending on the number of entries
const PAGE_LEVELS_PADDING = 4;

pub fn QuadTree(comptime K: type, comptime getEntryAabb: fn (*anyopaque, K) AABB) type {
    return struct {
        const QT = QuadTree(K, getEntryAabb);

        pub const Entry = struct {
            value: K,
            aabb: AABB,
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
            offset: usize,
            entries: struct {
                buffer: [MAX_ENTRIES_IN_PAGE]usize = undefined,
                len: usize = 0,

                pub const empty: @This() = .{};

                pub fn append(self: *@This(), item: usize) void {
                    self.buffer[self.len] = item;
                    self.len += 1;
                }
            },
            quadrants: Subdivision(?*Page) = Subdivision(?*Page){
                .tl = null,
                .tr = null,
                .bl = null,
                .br = null,
            },

            pub fn canAccomodate(self: Page) bool {
                return self.entries.len < MAX_ENTRIES_IN_PAGE;
            }

            pub fn subdivisions(self: Page) Subdivision(AABB) {
                const halfSize = self.aabb.size() / V.scalar(2);
                const halfX = V.onlyX(V.x(halfSize));
                const halfY = V.onlyY(V.y(halfSize));

                return Subdivision(AABB){
                    .tl = AABB{
                        .tl = self.aabb.tl,
                        .br = self.aabb.br - halfSize,
                        .isMinimal = false,
                    },
                    .tr = AABB{
                        .tl = self.aabb.tl + halfX,
                        .br = self.aabb.br - halfY,
                        .isMinimal = false,
                    },
                    .bl = AABB{
                        .tl = self.aabb.tl + halfY,
                        .br = self.aabb.br - halfX,
                        .isMinimal = false,
                    },
                    .br = AABB{
                        .tl = self.aabb.tl + halfSize,
                        .br = self.aabb.br,
                        .isMinimal = false,
                    },
                };
            }
        };

        context: *anyopaque = undefined,
        pages: ArrayList(Page),
        entries: ArrayList(AABB),
        entryValues: []const K,
        orphanedEntries: ArrayList(Entry),
        currentPagesIndex: usize,
        currentEntriesIndex: usize,
        intersectingBuffer: ArrayList(Intersection(K)),

        pub fn init(allocator: Allocator) QT {
            return QT{
                .pages = ArrayList(Page).initCapacity(allocator, INITIAL_PAGES_CAPACITY) catch unreachable,
                .entries = ArrayList(AABB).initCapacity(allocator, INITIAL_ENTRIES_CAPACITY) catch unreachable,
                .entryValues = &.{},
                .orphanedEntries = ArrayList(Entry).initCapacity(allocator, MAX_ENTRIES_IN_PAGE * 10) catch unreachable,
                .intersectingBuffer = ArrayList(Intersection(K)).initCapacity(allocator, 100) catch unreachable,
                .currentPagesIndex = 0,
                .currentEntriesIndex = 0,
            };
        }

        pub fn deinit(self: *QT, allocator: Allocator) void {
            self.pages.deinit(allocator);
            self.entries.deinit(allocator);
            if (self.entryValues.len > 0) allocator.free(self.entryValues);
            self.entryValues = &.{};
            self.orphanedEntries.deinit(allocator);
            self.intersectingBuffer.deinit(allocator);
        }

        pub fn reset(self: *QT, allocator: Allocator) void {
            self.pages.resize(allocator, 0) catch unreachable;
            self.entries.resize(allocator, 0) catch unreachable;
            if (self.entryValues.len > 0) allocator.free(self.entryValues);
            self.entryValues = &.{};
            self.orphanedEntries.resize(allocator, 0) catch unreachable;
            self.intersectingBuffer.resize(allocator, 0) catch unreachable;
            self.currentPagesIndex = 0;
            self.currentEntriesIndex = 0;
        }

        pub fn getRoot(self: QT) ?*Page {
            if (self.currentPagesIndex == 0) return null;
            return &self.pages.items[0];
        }

        fn calcPagesNeeded(entries: usize) usize {
            var n: usize = entries;
            var powerOf4: usize = 1;
            var nextThreshold: usize = powerOf4 * MAX_ENTRIES_IN_PAGE;
            var pagesNeeded: usize = powerOf4;

            while (n > nextThreshold) {
                n -= nextThreshold;
                powerOf4 *= 4;
                nextThreshold += powerOf4 * MAX_ENTRIES_IN_PAGE;
                pagesNeeded += powerOf4;
            }

            // Add some more levels to make sure we got enough space,
            // this really depends on the size of the bodies and how crammed they are into the space
            for (0..PAGE_LEVELS_PADDING) |_| {
                powerOf4 *= 4;
                pagesNeeded += powerOf4;
            }

            return pagesNeeded;
        }

        pub fn populateAndIntersect(
            self: *QT,
            allocator: Allocator,
            getEntryAabbContext: *anyopaque,
            boundary: AABB,
            entries: []const K,
            context: anytype,
            comptime intersectionHandler: fn (context: @TypeOf(context), entry: K, []Intersection(K)) void,
        ) void {
            // const paiZone = ztracy.ZoneN(@src(), "QT: Populate and Intersect");
            // defer paiZone.End();

            self.reset(allocator);
            self.entryValues = allocator.dupe(K, entries) catch unreachable;
            self.context = getEntryAabbContext;

            // const prepareZone = ztracy.ZoneN(@src(), "QT: Prepare Pages");
            const pagesNeeded = calcPagesNeeded(entries.len);
            //std.log.info("populating {} entries with {} pages", .{ entries.len, pagesNeeded });
            self.prepareToAddPages(allocator, pagesNeeded);
            // prepareZone.End();

            // const addPagesZone = ztracy.ZoneN(@src(), "QT: Add Pages");
            // defer addPagesZone.End();
            _ = self.addPagePrepared(boundary);

            for (0..entries.len) |entry| {
                const entryValue = entries[entry];
                const entryAabb = getEntryAabb(self.context, entryValue);

                if (!self.insert(entry, entryAabb)) {
                    self.orphanedEntries.append(allocator, Entry{ .value = entryValue, .aabb = entryAabb }) catch unreachable;
                }
                const intersections = self.intersecting(allocator, entryAabb);
                intersectionHandler(context, entryValue, intersections);
            }
        }

        pub fn insert(self: *QT, entry: usize, entryAabb: AABB) bool {
            // const zone = ztracy.ZoneN(@src(), "QT: Insert");
            // defer zone.End();

            const root = self.getRoot().?;
            return self.insertIntoPage(root, entry, entryAabb);
        }

        pub fn intersecting(
            self: *QT,
            allocator: Allocator,
            aabb: AABB,
        ) []Intersection(K) {
            // const zone = ztracy.ZoneN(@src(), "QT: Intersecting");
            // defer zone.End();

            const root = self.getRoot();
            std.debug.assert(root != null);
            self.intersectingBuffer.shrinkRetainingCapacity(0);
            if (self.currentPagesIndex == 0) return &.{};
            self.intersectingForPage(allocator, aabb, root.?);
            self.intersectingOrphaned(allocator, aabb);
            return self.intersectingBuffer.items;
        }

        pub fn isEntryInTree(self: QT, entryKey: K) bool {
            for (self.entryValues) |e| {
                if (e == entryKey) {
                    return true;
                }
            }
            return false;
        }

        fn insertIntoPage(self: *QT, page: *Page, entry: usize, entryAabb: AABB) bool {
            // const zone = ztracy.ZoneN(@src(), "QT: Insert into Page");
            // defer zone.End();

            if (!page.aabb.contains(entryAabb)) return false;
            return self.insertIntoPageSkipBoundaryCheck(page, entry, entryAabb);
        }

        fn insertIntoPageSkipBoundaryCheck(self: *QT, page: *Page, entry: usize, entryAabb: AABB) bool {
            if (self.insertIntoQuadrants(page, entry, entryAabb)) {
                return true;
            }

            if (self.insertIntoPageIfCanAccomodate(page, entry, entryAabb)) {
                return true;
            }

            return false;
        }

        fn insertIntoPageIfCanAccomodate(self: *QT, page: *Page, entry: usize, entryAabb: AABB) bool {
            if (!page.canAccomodate()) return false;

            self.entries.items[entry] = entryAabb;
            page.entries.append(entry);

            return true;
        }

        fn insertIntoQuadrants(self: *QT, page: *Page, entry: usize, entryAabb: AABB) bool {
            // const zone = ztracy.ZoneN(@src(), "QT: Insert into Quadrants");
            // defer zone.End();

            const subdivisions = page.subdivisions();

            inline for (std.meta.fields(Subdivision(AABB))) |field| {
                if (@field(subdivisions, field.name).contains(entryAabb)) {
                    if (@field(page.quadrants, field.name) == null) {
                        @field(page.quadrants, field.name) = self.addPagePrepared(@field(subdivisions, field.name));
                    }
                    return self.insertIntoPageSkipBoundaryCheck(@field(page.quadrants, field.name).?, entry, entryAabb);
                }
            }

            return false;
        }

        fn intersectingOrphaned(self: *QT, allocator: Allocator, aabb: AABB) void {
            // const zone = ztracy.ZoneN(@src(), "QT: Intersecting Orphaned");
            // defer zone.End();

            for (self.orphanedEntries.items) |entry| {
                if (aabb.intersection(entry.aabb)) |intersection| {
                    self.intersectingBuffer.append(allocator, .{
                        .entry = entry.value,
                        .axis = intersection.axis,
                        .depth = intersection.depth,
                    }) catch unreachable;
                }
            }
        }

        fn intersectingEntry(
            self: *QT,
            allocator: Allocator,
            aabb: AABB,
            entry: usize,
        ) void {
            // const zone = ztracy.ZoneN(@src(), "QT: Intersecting Entry");
            // defer zone.End();

            const entryAabb = self.entries.items[entry];
            if (aabb.intersection(entryAabb)) |intersection| {
                self.intersectingBuffer.append(allocator, .{
                    .entry = self.entryValues[entry],
                    .axis = intersection.axis,
                    .depth = intersection.depth,
                }) catch unreachable;
            }
        }

        fn intersectingForPage(
            self: *QT,
            allocator: Allocator,
            aabb: AABB,
            page: *Page,
        ) void {
            // const zone = ztracy.ZoneN(@src(), "QT: Intersecting for Page");
            // defer zone.End();

            if (!page.aabb.intersects(aabb)) return;

            for (0..page.entries.len) |i| {
                const entry = page.entries.buffer[i];
                self.intersectingEntry(allocator, aabb, entry);
            }

            self.intersectingForQuadrants(allocator, aabb, page);
        }

        const SubdivisionFieldEnum = std.meta.FieldEnum(Subdivision(AABB));
        const quadrantCheckOrder = [_][4][]const u8{
            // Center point in TopLeft
            .{
                "tl",
                "tr",
                "bl",
                "br",
            },
            // Center point in TopRight
            .{
                "tr",
                "tl",
                "br",
                "bl",
            },
            // Center point in BottomLeft
            .{
                "bl",
                "br",
                "tl",
                "tr",
            },
            // Center point in BottomRight
            .{
                "br",
                "bl",
                "tr",
                "tl",
            },
        };

        fn getQuadrantCheckOrder(aabb: AABB, page: *Page) usize {
            const center = aabb.center();
            const pageCenter = page.aabb.center();

            if (V.lessThan(center, pageCenter)) {
                return 0;
            } else {
                return 3;
            }
        }

        fn intersectingForQuadrants(
            self: *QT,
            allocator: Allocator,
            aabb: AABB,
            page: *Page,
        ) void {
            // const zone = ztracy.ZoneN(@src(), "QT: Intersecting for Quadrants");
            // defer zone.End();

            const checkOrderIdx = getQuadrantCheckOrder(aabb, page);

            inline for (0..4) |i| {
                if (checkOrderIdx == i) {
                    inline for (quadrantCheckOrder[i]) |field| {
                        if (@field(page.quadrants, field)) |q| {
                            self.intersectingForPage(allocator, aabb, q);

                            if (q.aabb.contains(aabb)) {
                                return;
                            }
                        }
                    }
                }
            }
        }

        fn prepareToAddPages(
            self: *QT,
            allocator: Allocator,
            numberOfPages: usize,
        ) void {
            self.pages.resize(allocator, self.pages.items.len + numberOfPages) catch unreachable;
            self.entries.resize(allocator, self.entries.items.len + numberOfPages * MAX_ENTRIES_IN_PAGE) catch unreachable;
        }

        fn addPagePrepared(self: *QT, boundary: AABB) *Page {
            // const zone = ztracy.ZoneN(@src(), "QT: Add Page");
            // defer zone.End();

            self.pages.items[self.currentPagesIndex] = Page{
                .aabb = boundary,
                .entries = .empty,
                .offset = self.currentEntriesIndex,
            };

            const pagePtr = &self.pages.items[self.currentPagesIndex];
            self.currentPagesIndex += 1;
            self.currentEntriesIndex += MAX_ENTRIES_IN_PAGE;
            return pagePtr;
        }
    };
}
