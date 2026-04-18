const std = @import("std");
const ztracy = @import("ztracy");

const AABB = @import("../shape.zig").AABB;
const V = @import("../../vector.zig").V;

// TODO: Add a feature that divides the plane into rectangles with infinite width and a fixed height,
// the height should not be less than the greatest height of any one body, so that each body can only be
// overlapping or contained in at most two of these rectangles that divide the plane.
//
// Each body will be assigned to up to two of these rectangles, depending on their Y-axis boundary.
//
// When sweeping, instead of having just one active list, there will be an active list for each of the
// division rectangles. When a left edge is encountered, place it in the corresponding active list for
// the division rectangles that contain or overlap the body.
// When encountering another left edge, only do collision checks for the active list that corresponds
// to the same division rectangles.
//
// This way, the sweep will only check collisions for rectangles that share division rectangles, thus
// reducing expensive collision checks.

const SMALL_LINE_BUFFER_LEN = 100;
/// Threshold for when to start sweeping on Y-axis when this many overlaps occur on the X-axis.
const SWEEP_Y_THRESHOLD = 3;

const Axis = enum { x, y };

pub const SweepLine = union(enum) {
    min: Unit,
    max: Unit,

    pub const Unit = struct {
        index: usize,
        value: f32,
    };

    pub fn value(self: SweepLine) f32 {
        return switch (self) {
            .min => |min| min.value,
            .max => |max| max.value,
        };
    }
};

pub const SweepLineByAABB = union(enum) {
    min: Unit,
    max: Unit,

    pub const Unit = struct {
        index: usize,
        value: f32,
        isEntry: bool,
    };

    pub fn value(self: SweepLineByAABB) f32 {
        return switch (self) {
            .min => |min| min.value,
            .max => |max| max.value,
        };
    }
};

pub fn sweep(
    comptime T: type,
    source: []const T,
    buffer: []SweepLine,
    overlappingBuffer: []bool,
    context: anytype,
    getAabb: fn (@TypeOf(context), source: T) AABB,
    onAxisOverlap: fn (@TypeOf(context), a: usize, bs: []bool, n: usize, source: []const T) void,
) void {
    const zone = ztracy.ZoneNC(@src(), "sweep", 0xff_00_f0_f0);
    defer zone.End();
    const sortedSweepLines = copySweepLines(T, .x, source, buffer, context, getAabb);
    std.mem.sort(SweepLine, sortedSweepLines, {}, minValue(SweepLine, SweepLine.value));
    const overlappingBufferX = overlappingBuffer[0..source.len];
    const overlappingBufferY = overlappingBuffer[source.len .. source.len * 2];

    for (0..overlappingBufferX.len) |i| {
        overlappingBufferX[i] = false;
    }

    var overlapsX: usize = 0;

    for (sortedSweepLines) |line| {
        switch (line) {
            .min => |min| {
                if (overlapsX > 0) {
                    if (overlapsX >= SWEEP_Y_THRESHOLD) {
                        sweepY(
                            T,
                            min.index,
                            source,
                            overlappingBufferX,
                            overlappingBufferY,
                            context,
                            getAabb,
                            onAxisOverlap,
                        );
                    } else {
                        onAxisOverlap(context, min.index, overlappingBufferX, overlapsX, source);
                    }
                }

                overlappingBufferX[min.index] = true;
                overlapsX += 1;
            },
            .max => |max| {
                overlappingBufferX[max.index] = false;
                overlapsX -= 1;
            },
        }
    }
}

pub fn sweepByAABB(
    comptime T: type,
    entry: AABB,
    source: []const T,
    buffer: []SweepLineByAABB,
    overlappingBuffer: []bool,
    context: anytype,
    getAabb: fn (@TypeOf(context), source: T) AABB,
    onAxisOverlap: fn (@TypeOf(context), entry: AABB, bs: []bool, n: usize, source: []const T) void,
) void {
    const zone = ztracy.ZoneNC(@src(), "sweepByAABB", 0xff_00_f0_f0);
    defer zone.End();

    const sortedSweepLines = copySweepLinesByAabb(T, .x, entry, source, buffer, context, getAabb);
    std.mem.sort(SweepLineByAABB, sortedSweepLines, {}, minValue(SweepLineByAABB, SweepLineByAABB.value));
    const overlappingBufferX = overlappingBuffer[0..source.len];
    // const overlappingBufferY = overlappingBuffer[source.len .. source.len * 2];

    @memset(overlappingBufferX, false);

    var overlapsX: usize = 0;
    var isEntryOverlapping = false;

    for (sortedSweepLines) |line| {
        switch (line) {
            .min => |min| {
                if (min.isEntry) {
                    isEntryOverlapping = true;
                } else {
                    overlappingBufferX[min.index] = true;
                    overlapsX += 1;
                }

                if (overlapsX > 0 and isEntryOverlapping) {
                    onAxisOverlap(context, entry, overlappingBufferX, overlapsX, source);

                    if (min.isEntry) continue;

                    overlappingBufferX[min.index] = false;
                    overlapsX -= 1;
                }
            },
            .max => |max| {
                if (max.isEntry) return;

                if (overlappingBufferX[max.index]) {
                    overlappingBufferX[max.index] = false;
                    overlapsX -= 1;
                }
            },
        }
    }
}

fn sweepY(
    comptime T: type,
    index: usize,
    source: []const T,
    overlappingBufferX: []bool,
    overlappingBufferY: []bool,
    context: anytype,
    getAabb: fn (@TypeOf(context), source: T) AABB,
    onAxisOverlap: fn (context: @TypeOf(context), a: usize, bs: []bool, n: usize, source: []const T) void,
) void {
    const zone = ztracy.ZoneNC(@src(), "sweepY", 0xff_00_f0_f0);
    defer zone.End();

    var smallBuffer: [SMALL_LINE_BUFFER_LEN]SweepLine = undefined;

    var n: usize = 0;
    for (0..overlappingBufferX.len) |j| {
        if (!overlappingBufferX[j]) continue;

        const aabb = getAabb(context, source[j]);
        copySweepLine(.y, n, aabb, &smallBuffer);
        n += 1;

        //onAxisOverlap(context, min.index, j);
    }
    const sortedSmallBuffer = smallBuffer[0..n];
    std.mem.sort(SweepLine, sortedSmallBuffer, {}, minValue(SweepLine, SweepLine.value));

    for (0..overlappingBufferY.len) |i| {
        overlappingBufferY[i] = false;
    }

    var overlapsY: usize = 0;

    for (sortedSmallBuffer) |otherAxisLine| {
        switch (otherAxisLine) {
            .min => |minY| {
                if (overlapsY > 0) {
                    onAxisOverlap(context, index, overlappingBufferY, overlapsY, source);
                }

                overlappingBufferY[minY.index] = true;
                overlapsY += 1;
            },
            .max => |maxY| {
                overlappingBufferY[maxY.index] = false;
                overlapsY -= 1;
            },
        }
    }
}

fn copySweepLines(
    comptime T: type,
    comptime axis: Axis,
    source: []const T,
    buffer: []SweepLine,
    context: anytype,
    getAabb: fn (@TypeOf(context), source: T) AABB,
) []SweepLine {
    const zone = ztracy.ZoneNC(@src(), "copy sweep lines", 0xff_00_f0_f0);
    defer zone.End();

    for (0.., source) |i, s| {
        const aabb = getAabb(context, s);
        copySweepLine(axis, i, aabb, buffer);
    }

    return buffer[0 .. source.len * 2];
}

fn copySweepLinesByAabb(
    comptime T: type,
    comptime axis: Axis,
    entry: AABB,
    source: []const T,
    buffer: []SweepLineByAABB,
    context: anytype,
    getAabb: fn (@TypeOf(context), source: T) AABB,
) []SweepLineByAABB {
    const zone = ztracy.ZoneNC(@src(), "copy sweep lines", 0xff_00_f0_f0);
    defer zone.End();

    for (0.., source) |i, s| {
        const aabb = getAabb(context, s);
        copySweepLineByAabb(axis, i, aabb, false, buffer);
    }

    copySweepLineByAabb(axis, source.len, entry, true, buffer);

    return buffer[0 .. source.len * 2 + 2];
}

fn copySweepLineByAabb(
    comptime axis: Axis,
    index: usize,
    aabb: AABB,
    isEntry: bool,
    buffer: []SweepLineByAABB,
) void {
    const zone = ztracy.ZoneNC(@src(), "copy sweep line", 0xff_00_f0_f0);
    defer zone.End();

    const min = SweepLineByAABB{
        .min = .{
            .index = index,
            .value = if (axis == .x) aabb.left() else aabb.top(),
            .isEntry = isEntry,
        },
    };
    const max = SweepLineByAABB{
        .max = .{
            .index = index,
            .value = if (axis == .x) aabb.right() else aabb.bottom(),
            .isEntry = isEntry,
        },
    };

    const j = index * 2;
    buffer[j] = min;
    buffer[j + 1] = max;
}

fn copySweepLine(
    comptime axis: Axis,
    index: usize,
    aabb: AABB,
    buffer: []SweepLine,
) void {
    const zone = ztracy.ZoneNC(@src(), "copy sweep line", 0xff_00_f0_f0);
    defer zone.End();

    const min = SweepLine{
        .min = .{
            .index = index,
            .value = if (axis == .x) aabb.left() else aabb.top(),
        },
    };
    const max = SweepLine{
        .max = .{
            .index = index,
            .value = if (axis == .x) aabb.right() else aabb.bottom(),
        },
    };

    const j = index * 2;
    buffer[j] = min;
    buffer[j + 1] = max;
}

fn minValue(comptime T: type, getValue: *const fn (T) f32) fn (void, T, T) bool {
    return struct {
        pub fn minValue(_: void, lhs: T, rhs: T) bool {
            return getValue(lhs) < getValue(rhs);
        }
    }.minValue;
}
