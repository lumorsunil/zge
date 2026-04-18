const std = @import("std");
const rl = @import("raylib");
const Allocator = std.mem.Allocator;
const ArrayList = std.ArrayList;

const ztracy = @import("ztracy");
const isTracingEnabled = @import("../../config.zig").isTracingEnabled;

const V = @import("../../vector.zig").V;
const Vector = @import("../../vector.zig").Vector;

const AABB = @import("../shape.zig").AABB;
const Intersection = @import("intersection.zig").Intersection;

const sweepByAABB = @import("sweep.zig").sweepByAABB;
const SweepLineByAABB = @import("sweep.zig").SweepLineByAABB;

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
const MAX_ENTRIES_IN_PAGE = 30;
/// How many levels to make space for after the pages needed calculation depending on the number of entries
const PAGE_LEVELS_PADDING = 4;
const MIN_ENTRIES_FOR_SUBDIVISION = 10;

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

                pub fn initAll(value: SDT) @This() {
                    return .{
                        .tl = value,
                        .tr = value,
                        .bl = value,
                        .br = value,
                    };
                }
            };
        }

        pub const PageEntryList = struct {
            buffer: [MAX_ENTRIES_IN_PAGE]usize = undefined,
            len: usize = 0,

            pub const empty: @This() = .{};

            pub fn append(self: *@This(), item: usize) void {
                self.buffer[self.len] = item;
                self.len += 1;
            }

            pub fn appendSlice(self: *@This(), items: []usize) void {
                for (0..items.len) |i| {
                    self.buffer[self.len + i] = items[i];
                }

                self.len += items.len;
            }

            pub fn slice(self: *const PageEntryList) []const usize {
                return self.buffer[0..self.len];
            }
        };

        pub const Page = struct {
            parent: ?*Page,
            aabb: AABB,
            offset: usize,
            entries: PageEntryList,
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

        allocator: Allocator,
        context: *anyopaque = undefined,
        pages: ArrayList(Page),
        entries: ArrayList(AABB),
        entryValues: []const K,
        orphanedEntries: ArrayList(Entry),
        entriesToPages: ArrayList(?*Page),
        populateStorage: PopulateStorage,
        currentPagesIndex: usize,
        currentEntriesIndex: usize,
        sweepLineBuffer: ArrayList(SweepLineByAABB),
        overlappingBuffer: ArrayList(bool),
        intersectingBuffer: ArrayList(Intersection(K)),

        pub fn init(allocator: Allocator) QT {
            return QT{
                .allocator = allocator,
                .pages = ArrayList(Page).initCapacity(allocator, INITIAL_PAGES_CAPACITY) catch unreachable,
                .entries = ArrayList(AABB).initCapacity(allocator, INITIAL_ENTRIES_CAPACITY) catch unreachable,
                .entryValues = &.{},
                .orphanedEntries = ArrayList(Entry).initCapacity(allocator, INITIAL_ENTRIES_CAPACITY) catch unreachable,
                .entriesToPages = ArrayList(?*Page).initCapacity(allocator, INITIAL_ENTRIES_CAPACITY) catch unreachable,
                .populateStorage = PopulateStorage.init(allocator, INITIAL_ENTRIES_CAPACITY, calcPagesNeeded(INITIAL_ENTRIES_CAPACITY)),
                .sweepLineBuffer = .empty,
                .overlappingBuffer = .empty,
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
            self.entriesToPages.deinit(allocator);
            self.intersectingBuffer.deinit(allocator);
            self.populateStorage.deinit(allocator);
        }

        pub fn reset(self: *QT, allocator: Allocator) void {
            self.pages.resize(allocator, 0) catch unreachable;
            self.entries.resize(allocator, 0) catch unreachable;
            if (self.entryValues.len > 0) allocator.free(self.entryValues);
            self.entryValues = &.{};
            self.orphanedEntries.resize(allocator, 0) catch unreachable;
            self.entriesToPages.resize(allocator, 0) catch unreachable;
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
            const paiZone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Populate and Intersect") else null;
            defer if (paiZone) |zone| zone.End();

            self.reset(allocator);
            self.entryValues = allocator.dupe(K, entries) catch unreachable;
            self.context = getEntryAabbContext;

            const prepareZone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Prepare Pages") else null;
            const pagesNeeded = calcPagesNeeded(entries.len);
            //std.log.info("populating {} entries with {} pages", .{ entries.len, pagesNeeded });
            self.prepareToAddPages(allocator, pagesNeeded);
            if (prepareZone) |zone| zone.End();

            const addPagesZone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Add Pages") else null;
            defer if (addPagesZone) |zone| zone.End();
            _ = self.addPagePrepared(boundary, null);

            const preCopyZone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Pre-Copy AABB's") else null;
            for (0..entries.len) |i| {
                const entryAabb = getEntryAabb(self.context, entries[i]);
                self.entries.items[i] = entryAabb;
            }
            if (preCopyZone) |zone| zone.End();

            for (0..entries.len) |i| {
                const entry = entries[i];
                // const entryAabb = getEntryAabb(self.context, entry);
                const entryAabb = self.entries.items[i];

                if (self.insert(i, entryAabb)) |page| {
                    self.entriesToPages.appendAssumeCapacity(page);
                } else {
                    self.orphanedEntries.appendAssumeCapacity(Entry{ .value = entry, .aabb = entryAabb });
                    self.entriesToPages.appendAssumeCapacity(null);
                }

                const intersections = self.intersecting(allocator, i, entryAabb);
                // const intersections = self.intersecting(allocator, entryAabb);
                intersectionHandler(context, entry, intersections);
            }
        }

        const PopulateStorage = struct {
            entries: []K,
            indices: []usize,
            unitCapacity: usize,
            maxUnits: usize,
            index: usize = 0,

            pub fn init(
                allocator: Allocator,
                unitCapacity: usize,
                maxUnits: usize,
            ) PopulateStorage {
                const totalCapacity = unitCapacity * maxUnits;
                // const unitSize = @sizeOf(usize) + @sizeOf(K);
                // std.log.debug("populate storage allocating {} bytes", .{totalCapacity * unitSize});
                return .{
                    .entries = allocator.alloc(K, totalCapacity) catch unreachable,
                    .indices = allocator.alloc(usize, totalCapacity) catch unreachable,
                    .unitCapacity = unitCapacity,
                    .maxUnits = maxUnits,
                };
            }

            pub fn deinit(self: PopulateStorage, allocator: Allocator) void {
                allocator.free(self.indices);
                allocator.free(self.entries);
            }

            pub fn resetIndex(self: *PopulateStorage) void {
                self.index = 0;
            }

            pub fn ensureCapacity(
                self: *PopulateStorage,
                allocator: Allocator,
                unitCapacity: usize,
                maxUnits: usize,
            ) void {
                const currentTotalCapacity = self.unitCapacity * self.maxUnits;
                const totalCapacity = unitCapacity * maxUnits;

                self.unitCapacity = unitCapacity;
                self.maxUnits = maxUnits;

                if (totalCapacity <= currentTotalCapacity) return;

                allocator.free(self.indices);
                allocator.free(self.entries);

                self.entries = allocator.alloc(K, totalCapacity) catch unreachable;
                self.indices = allocator.alloc(usize, totalCapacity) catch unreachable;
            }

            pub const Unit = struct {
                entries: ArrayList(K),
                indices: ArrayList(usize),
            };

            pub fn requestUnit(
                self: *PopulateStorage,
            ) Unit {
                const start = self.index * self.unitCapacity;
                const end = start + self.unitCapacity;
                defer self.index += 1;

                return .{
                    .entries = ArrayList(K).initBuffer(self.entries[start..end]),
                    .indices = ArrayList(usize).initBuffer(self.indices[start..end]),
                };
            }
        };

        pub fn populateAndIntersectPageByPage(
            self: *QT,
            allocator: Allocator,
            getEntryAabbContext: *anyopaque,
            boundary: AABB,
            entries: []const K,
            context: anytype,
            comptime intersectionHandler: fn (context: @TypeOf(context), entry: K, []Intersection(K)) void,
        ) void {
            const paiZone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Populate and Intersect (Page by Page)") else null;
            defer if (paiZone) |zone| zone.End();

            const resetZone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Reset") else null;
            self.reset(allocator);
            self.entryValues = allocator.dupe(K, entries) catch unreachable;
            self.context = getEntryAabbContext;
            if (resetZone) |zone| zone.End();

            const prepareZone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Prepare Pages") else null;
            const pagesNeeded = calcPagesNeeded(entries.len);
            //std.log.info("populating {} entries with {} pages", .{ entries.len, pagesNeeded });
            self.prepareToAddPages(allocator, pagesNeeded);
            if (prepareZone) |zone| zone.End();

            const addPagesZone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Add Pages") else null;
            const root = self.addPagePrepared(boundary, null);
            if (addPagesZone) |zone| zone.End();

            const preCopyZone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Pre-Copy AABB's") else null;
            for (0..entries.len) |i| {
                const entryAabb = getEntryAabb(self.context, entries[i]);
                self.entries.items[i] = entryAabb;
            }
            if (preCopyZone) |zone| zone.End();

            const allocZone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Alloc Storage") else null;
            self.entriesToPages.resize(allocator, entries.len) catch unreachable;
            self.populateStorage.ensureCapacity(allocator, entries.len, pagesNeeded);
            self.populateStorage.resetIndex();
            if (allocZone) |zone| zone.End();

            self.populatePage(allocator, entries, &.{}, root);
            self.searchAndReportIntersections(allocator, context, intersectionHandler);
        }

        fn searchAndReportIntersections(
            self: *QT,
            allocator: Allocator,
            context: anytype,
            comptime intersectionHandler: fn (context: @TypeOf(context), entry: K, []Intersection(K)) void,
        ) void {
            for (self.entryValues, 0..) |entry, i| {
                const entryAabb = self.entries.items[i];
                const intersections = self.intersectingBottomUp(allocator, i, entryAabb);
                intersectionHandler(context, entry, intersections);
            }
        }

        pub fn populatePage(
            self: *QT,
            allocator: Allocator,
            entries: []const K,
            indices: []const usize,
            page: *Page,
        ) void {
            const mainZone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Populate Page") else null;
            defer if (mainZone) |zone| zone.End();

            const subdivisions = page.subdivisions();
            var quadrantsEntries = Subdivision(PopulateStorage.Unit){
                .tl = self.populateStorage.requestUnit(),
                .tr = self.populateStorage.requestUnit(),
                .bl = self.populateStorage.requestUnit(),
                .br = self.populateStorage.requestUnit(),
            };

            const choosingPage: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Choosing Page") else null;
            for (entries, 0..) |entry, i| {
                const index = if (indices.len == 0) i else indices[i];
                const entryAabb = self.entries.items[index];
                var quadrant: ?SubdivisionFieldEnum = null;

                inner: inline for (0..std.meta.tags(SubdivisionFieldEnum).len) |j| {
                    const tag = comptime std.meta.tags(SubdivisionFieldEnum)[j];
                    const subdivision: AABB = @field(subdivisions, @tagName(tag));
                    if (subdivision.contains(entryAabb)) {
                        quadrant = tag;
                        break :inner;
                    }
                }

                if (quadrant) |q| {
                    switch (q) {
                        inline else => |t| {
                            const e = &@field(quadrantsEntries, @tagName(t));
                            e.indices.appendAssumeCapacity(index);
                            e.entries.appendAssumeCapacity(entry);
                        },
                    }

                    continue;
                }

                var currentParent = page.parent;
                inner: while (currentParent) |parent| {
                    if (parent.canAccomodate()) {
                        self.entriesToPages.items[index] = parent;
                        parent.entries.append(index);
                        break :inner;
                    } else {
                        currentParent = parent.parent;
                    }
                } else {
                    self.entriesToPages.items[index] = null;
                    self.orphanedEntries.appendAssumeCapacity(
                        Entry{ .value = entry, .aabb = entryAabb },
                    );
                }
            }
            if (choosingPage) |zone| zone.End();

            const popZone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Populating Children") else null;
            inline for (std.meta.fields(SubdivisionFieldEnum)) |field| {
                const e: PopulateStorage.Unit = @field(quadrantsEntries, field.name);

                if (e.entries.items.len > 0) {
                    const quadrantPage = self.addPagePrepared(
                        @field(subdivisions, field.name),
                        page,
                    );
                    @field(page.quadrants, field.name) = quadrantPage;

                    if (e.entries.items.len < MIN_ENTRIES_FOR_SUBDIVISION) {
                        quadrantPage.entries.appendSlice(e.indices.items);

                        for (e.indices.items) |i| {
                            self.entriesToPages.items[i] = quadrantPage;
                        }
                    } else {
                        self.populatePage(
                            allocator,
                            e.entries.items,
                            e.indices.items,
                            quadrantPage,
                        );
                    }
                }
            }
            if (popZone) |zone| zone.End();
        }

        fn updatePositions(
            self: *QT,
        ) void {
            const iZone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Update Positions") else null;
            defer if (iZone) |zone| zone.End();

            updateEntities: for (self.entriesToPages.items, 0..) |originalParent, i| {
                self.entries.items[i] = getEntryAabb(self.context, self.entryValues[i]);
                const entryAabb = self.entries.items[i];

                while (self.entriesToPages.items[i]) |page| {
                    if (page.aabb.contains(entryAabb) and page.canAccomodate()) continue :updateEntities;
                    self.entriesToPages.items[i] = page.parent;
                }

                if (originalParent != self.entriesToPages.items[i]) {
                    if (self.entriesToPages.items[i]) |newParent| {
                        newParent.entries.append(i);
                    } else {
                        self.orphanedEntries.appendAssumeCapacity(Entry{ .value = self.entryValues[i], .aabb = entryAabb });
                    }
                }
            }
        }

        pub fn updatePositionsAndIntersect(
            self: *QT,
            allocator: Allocator,
            context: anytype,
            comptime intersectionHandler: fn (context: @TypeOf(context), entry: K, []Intersection(K)) void,
        ) void {
            const iZone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: ONLY Intersect") else null;
            defer if (iZone) |zone| zone.End();

            self.updatePositions();
            self.searchAndReportIntersections(allocator, context, intersectionHandler);
        }

        pub fn insert(self: *QT, entry: usize, entryAabb: AABB) ?*Page {
            const zone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Insert") else null;
            defer if (zone) |z| z.End();

            const root = self.getRoot().?;
            return self.insertIntoPage(root, entry, entryAabb);
        }

        pub fn intersecting(
            self: *QT,
            allocator: Allocator,
            aabb: AABB,
        ) []Intersection(K) {
            const zone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Intersecting") else null;
            defer if (zone) |z| z.End();

            const root = self.getRoot();
            std.debug.assert(root != null);
            self.intersectingBuffer.shrinkRetainingCapacity(0);
            if (self.currentPagesIndex == 0) return &.{};
            self.intersectingForPageTopDown(allocator, null, aabb, root.?);
            self.intersectingOrphaned(allocator, aabb);
            return self.intersectingBuffer.items;
        }

        pub fn intersectingBottomUp(
            self: *QT,
            allocator: Allocator,
            index: usize,
            aabb: AABB,
        ) []Intersection(K) {
            const zone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Intersecting (Bottom Up)") else null;
            defer if (zone) |z| z.End();

            self.intersectingBuffer.shrinkRetainingCapacity(0);
            if (self.currentPagesIndex == 0) return &.{};
            if (self.entriesToPages.items[index]) |page| {
                self.intersectingForPageBottomUp(allocator, index, aabb, page);
                self.intersectingForQuadrants(allocator, index, aabb, page);
            } else {
                const root = self.getRoot();
                std.debug.assert(root != null);
                self.intersectingForPageTopDown(allocator, index, aabb, root.?);
            }

            self.intersectingOrphaned(allocator, aabb);

            return self.intersectingBuffer.items;
        }

        pub fn intersectingLine(
            self: *QT,
            allocator: Allocator,
            start: Vector,
            end: Vector,
        ) []Intersection(K) {
            const zone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Intersecting") else null;
            defer if (zone) |z| z.End();

            const root = self.getRoot();
            std.debug.assert(root != null);
            self.intersectingBuffer.shrinkRetainingCapacity(0);
            if (self.currentPagesIndex == 0) return &.{};
            self.intersectingLineForPage(allocator, start, end, root.?);
            self.intersectingLineOrphaned(allocator, start, end);
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

        fn insertIntoPage(self: *QT, page: *Page, entry: usize, entryAabb: AABB) ?*Page {
            const zone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Insert into Page") else null;
            defer if (zone) |z| z.End();

            if (!page.aabb.contains(entryAabb)) return null;
            return self.insertIntoPageSkipBoundaryCheck(page, entry, entryAabb);
        }

        fn insertIntoPageSkipBoundaryCheck(
            self: *QT,
            page: *Page,
            entry: usize,
            entryAabb: AABB,
        ) ?*Page {
            return self.insertIntoQuadrants(page, entry, entryAabb) orelse
                self.insertIntoPageIfCanAccomodate(page, entry, entryAabb);
        }

        fn insertIntoPageIfCanAccomodate(
            self: *QT,
            page: *Page,
            entry: usize,
            entryAabb: AABB,
        ) ?*Page {
            _ = self;
            _ = entryAabb;
            if (!page.canAccomodate()) return null;

            // self.entries.items[entry] = entryAabb;
            page.entries.append(entry);

            return page;
        }

        fn insertIntoQuadrants(self: *QT, page: *Page, entry: usize, entryAabb: AABB) ?*Page {
            const zone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Insert into Quadrants") else null;
            defer if (zone) |z| z.End();

            const subdivisions = page.subdivisions();

            inline for (std.meta.fields(Subdivision(AABB))) |field| {
                if (@field(subdivisions, field.name).contains(entryAabb)) {
                    if (@field(page.quadrants, field.name) == null) {
                        @field(page.quadrants, field.name) = self.addPagePrepared(@field(subdivisions, field.name), page);
                    }
                    return self.insertIntoPageSkipBoundaryCheck(@field(page.quadrants, field.name).?, entry, entryAabb);
                }
            }

            return null;
        }

        fn intersectingOrphaned(self: *QT, allocator: Allocator, aabb: AABB) void {
            const zone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Intersecting Orphaned") else null;
            defer if (zone) |z| z.End();

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

        fn intersectingLineOrphaned(self: *QT, allocator: Allocator, start: Vector, end: Vector) void {
            const zone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Intersecting Orphaned") else null;
            defer if (zone) |z| z.End();

            for (self.orphanedEntries.items) |entry| {
                if (checkCollisionLineRec(start, end, entry.aabb.toRaylib())) {
                    // TODO: make start and end into a aabb and then get intersection information
                    self.intersectingBuffer.append(allocator, .{
                        .entry = entry.value,
                        .axis = V.zero,
                        .depth = 1,
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
            const zone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Intersecting Entry") else null;
            defer if (zone) |z| z.End();

            const entryAabb = self.entries.items[entry];
            if (aabb.intersection(entryAabb)) |intersection| {
                self.intersectingBuffer.append(allocator, .{
                    .entry = self.entryValues[entry],
                    .axis = intersection.axis,
                    .depth = intersection.depth,
                }) catch unreachable;
            }
        }

        fn intersectingLineEntry(
            self: *QT,
            allocator: Allocator,
            start: Vector,
            end: Vector,
            entry: usize,
        ) void {
            const zone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Intersecting Entry") else null;
            defer if (zone) |z| z.End();

            const entryAabb = self.entries.items[entry];
            if (checkCollisionLineRec(start, end, entryAabb.toRaylib())) {
                // TODO: make start and end into a aabb and then get intersection information
                self.intersectingBuffer.append(allocator, .{
                    .entry = self.entryValues[entry],
                    .axis = V.zero,
                    .depth = 1,
                }) catch unreachable;
            }
        }

        fn intersectingForPageTopDown(
            self: *QT,
            allocator: Allocator,
            index: ?usize,
            aabb: AABB,
            page: *Page,
        ) void {
            const zone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Intersecting for Page (Top Down)") else null;
            defer if (zone) |z| z.End();

            if (!page.aabb.intersects(aabb)) return;

            self.intersectingEntriesInPage(aabb, page);
            self.intersectingForQuadrants(allocator, index, aabb, page);
        }

        fn intersectingForPageBottomUp(
            self: *QT,
            allocator: Allocator,
            index: usize,
            aabb: AABB,
            page: *Page,
        ) void {
            const zone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Intersecting for Page (Bottom Up)") else null;
            defer if (zone) |z| z.End();

            self.intersectingEntriesInPage(aabb, page);

            if (page.parent) |parent| {
                self.intersectingForPageBottomUp(allocator, index, aabb, parent);
            }
        }

        fn intersectingEntriesInPage(
            self: *QT,
            aabb: AABB,
            page: *Page,
        ) void {
            const entries = page.entries.slice();

            // if (entries.len >= 10) {
            //     self.intersectingEntriesSweep(aabb, entries);
            // } else {
            self.intersectingEntriesBruteForce(aabb, entries);
            // }
        }

        fn intersectingEntriesBruteForce(
            self: *QT,
            entry: AABB,
            entries: []const usize,
        ) void {
            for (entries) |e| {
                self.intersectingEntry(self.allocator, entry, e);
            }
        }

        fn intersectingEntriesSweep(
            self: *QT,
            entry: AABB,
            entries: []const usize,
        ) void {
            self.sweepLineBuffer.ensureTotalCapacityPrecise(self.allocator, entries.len * 2 + 2) catch unreachable;
            self.sweepLineBuffer.expandToCapacity();
            self.overlappingBuffer.ensureTotalCapacityPrecise(self.allocator, entries.len * 2 + 2) catch unreachable;
            self.overlappingBuffer.expandToCapacity();

            sweepByAABB(
                usize,
                entry,
                entries,
                self.sweepLineBuffer.items,
                self.overlappingBuffer.items,
                self,
                getAabbByIndex,
                onAxisOverlap,
            );
        }

        fn getAabbByIndex(self: *QT, index: usize) AABB {
            return self.entries.items[index];
        }

        fn onAxisOverlap(self: *QT, aabbA: AABB, bs: []bool, n: usize, source: []const usize) void {
            var left = n;

            for (0.., bs) |i, b| {
                if (!b or left <= 0) continue;
                const indexB = source[i];

                self.intersectingEntry(self.allocator, aabbA, indexB);

                left -= 1;
            }
        }

        fn intersectingLineForPage(
            self: *QT,
            allocator: Allocator,
            start: Vector,
            end: Vector,
            page: *Page,
        ) void {
            const zone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Intersecting for Page") else null;
            defer if (zone) |z| z.End();

            if (!checkCollisionLineRec(start, end, page.aabb.toRaylib())) return;

            for (0..page.entries.len) |i| {
                const entry = page.entries.buffer[i];
                self.intersectingLineEntry(allocator, start, end, entry);
            }

            self.intersectingLineForQuadrants(allocator, start, end, page);
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
            index: ?usize,
            aabb: AABB,
            page: *Page,
        ) void {
            const zone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Intersecting for Quadrants") else null;
            defer if (zone) |z| z.End();

            const checkOrderIdx = getQuadrantCheckOrder(aabb, page);

            inline for (0..4) |i| {
                if (checkOrderIdx == i) {
                    inline for (quadrantCheckOrder[i]) |field| {
                        if (@field(page.quadrants, field)) |q| {
                            self.intersectingForPageTopDown(allocator, index, aabb, q);

                            if (q.aabb.contains(aabb)) {
                                return;
                            }
                        }
                    }
                }
            }
        }

        fn intersectingLineForQuadrants(
            self: *QT,
            allocator: Allocator,
            start: Vector,
            end: Vector,
            page: *Page,
        ) void {
            const zone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Intersecting for Quadrants") else null;
            defer if (zone) |z| z.End();

            inline for (quadrantCheckOrder[0]) |field| {
                if (@field(page.quadrants, field)) |q| {
                    self.intersectingLineForPage(allocator, start, end, q);

                    if (q.aabb.containsLine(start, end)) return;
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

        fn addPagePrepared(self: *QT, boundary: AABB, parent: ?*Page) *Page {
            const zone: ?ztracy.ZoneCtx = if (isTracingEnabled) ztracy.ZoneN(@src(), "QT: Add Page") else null;
            defer if (zone) |z| z.End();

            self.pages.items[self.currentPagesIndex] = Page{
                .parent = parent,
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

fn checkCollisionLineRec(start: Vector, end: Vector, rect: rl.Rectangle) bool {
    const startRl = V.toRl(start);
    const endRl = V.toRl(end);

    if (rl.checkCollisionPointRec(startRl, rect)) return true;
    if (rl.checkCollisionPointRec(endRl, rect)) return true;

    const rectLines = [_]struct { rl.Vector2, rl.Vector2 }{
        .{ rl.Vector2.init(rect.x, rect.y), rl.Vector2.init(rect.x + rect.width, rect.y) },
        .{ rl.Vector2.init(rect.x + rect.width, rect.y), rl.Vector2.init(rect.x + rect.width, rect.y + rect.height) },
        .{ rl.Vector2.init(rect.x + rect.width, rect.y + rect.height), rl.Vector2.init(rect.x, rect.y + rect.height) },
        .{ rl.Vector2.init(rect.x, rect.y + rect.height), rl.Vector2.init(rect.x, rect.y) },
    };

    for (rectLines) |rectLine| {
        const rectLineStart, const rectLineEnd = rectLine;
        var collisionPoint: rl.Vector2 = undefined;
        if (rl.checkCollisionLines(startRl, endRl, rectLineStart, rectLineEnd, &collisionPoint)) return true;
    }

    return false;
}
