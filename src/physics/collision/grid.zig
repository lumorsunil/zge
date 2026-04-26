const std = @import("std");
const Allocator = std.mem.Allocator;
const AABB = @import("../shape.zig").AABB;
const checkCollision = @import("aabb.zig").checkCollision;
const Intersection = @import("intersection.zig").Intersection;
const Vector = @import("../../vector.zig").Vector;
const VectorGeneric = @import("../../vector.zig").VectorGeneric;
const V = @import("../../vector.zig").V;

const GridCoordComponent = usize;

pub const GridHash = struct {
    data: std.ArrayList(bool) = .empty,
    gridSize: VectorGeneric(usize),
    cellSize: Vector = V.zero,
    offset: Vector = V.zero,

    pub const GridCoord = VectorGeneric(GridCoordComponent);

    pub const empty = @This(){};

    pub fn deinit(self: *@This(), allocator: Allocator) void {
        self.data.deinit(allocator);
    }

    /// Takes ownership of the passed in slice.
    pub fn populate(
        self: *@This(),
        data: []bool,
        gridSize: VectorGeneric(usize),
        cellSize: Vector,
        offset: Vector,
    ) void {
        self.data = .fromOwnedSlice(data);
        self.gridSize = gridSize;
        self.cellSize = cellSize;
        self.offset = offset;
    }

    pub fn populateF(
        self: *@This(),
        allocator: Allocator,
        context: anytype,
        gridSize: VectorGeneric(usize),
        cellSize: Vector,
        offset: Vector,
        isTileSolid: *const fn (@TypeOf(context), coord: GridCoord) bool,
    ) void {
        self.data.ensureTotalCapacity(allocator, self.gridSize[0] * self.gridSize[1]) catch unreachable;
        self.data.expandToCapacity();
        self.gridSize = gridSize;
        self.cellSize = cellSize;
        self.offset = offset;

        for (0..self.gridSize[0]) |x| {
            for (0..self.gridSize[1]) |y| {
                self.data[x + y * self.gridSize[0]] = isTileSolid(context, .{ x, y });
            }
        }
    }

    pub fn getIntersections(
        self: @This(),
        allocator: Allocator,
        aabb: AABB,
    ) []Intersection(AABB) {
        // Find corners of aabb
        // Turn them into grid coordinates
        // Check all tiles within those coordinates

        var intersections = std.ArrayList(Intersection(AABB)).empty;

        const start = @max(.{ 0, 0 }, self.toGridCoords(aabb.tl));
        const end = @min(self.gridSize, self.toGridCoords(aabb.br));

        // s-xx
        // --xx
        // ----
        // ---e
        //
        // s---
        // ----
        // xx--
        // xx-e

        for (V.x(start)..V.x(end) + 1) |x| {
            for (V.y(start)..V.y(end) + 1) |y| {
                // if grid coord is true -> create intersection
                const coord: GridCoord = .{ x, y };
                if (self.checkIntersection(aabb, coord)) |intersection| {
                    intersections.append(allocator, intersection);
                }
            }
        }

        return intersections.toOwnedSlice(allocator) catch unreachable;
    }

    fn toGridCoords(self: @This(), v: Vector) VectorGeneric(isize) {
        const translated = v + self.offset;
        return V.toInt(isize, @floor(translated / self.cellSize));
    }

    fn checkIntersection(self: @This(), aabb: AABB, coord: GridCoord) ?Intersection(AABB) {
        if (self.data.items(coord[0] + coord[1] * self.gridSize[1])) {
            const tile_aabb = self.coordToAABB(coord);
            var depth: f32 = undefined;
            var axis: Vector = undefined;
            _ = checkCollision(aabb, tile_aabb, &depth, &axis);
            return .{
                .entry = tile_aabb,
                .depth = depth,
                .axis = axis,
            };
        } else {
            return null;
        }
    }

    fn coordToAABB(self: @This(), coord: GridCoord) AABB {
        return .fromTopLeft(V.fromInt(coord * self.gridSize), self.cellSize, true);
    }

    pub const Stub = struct {
        pub fn deinit(_: *@This(), _: Allocator) void {}

        pub fn getIntersections(
            _: @This(),
            _: Allocator,
            _: AABB,
        ) []Intersection(AABB) {}

        pub fn populateF(
            _: *@This(),
            _: Allocator,
            context: anytype,
            _: VectorGeneric(usize),
            _: Vector,
            _: Vector,
            _: *const fn (@TypeOf(context), coord: GridCoord) bool,
        ) void {}
    };
};
