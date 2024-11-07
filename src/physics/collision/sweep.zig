const std = @import("std");
const ztracy = @import("ztracy");

const AABB = @import("../shape.zig").AABB;

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

pub fn sweep(
    comptime T: type,
    source: []T,
    buffer: []SweepLine,
    overlappingBuffer: []bool,
    context: anytype,
    onAxisOverlap: fn (@TypeOf(context), a: usize, bs: []bool, n: usize) void,
) void {
    const zone = ztracy.ZoneNC(@src(), "sweep", 0xff_00_f0_f0);
    defer zone.End();
    const sortedSweepLines = copySweepLines(T, .x, source, buffer);
    std.mem.sort(SweepLine, sortedSweepLines, {}, minValue);
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
                            onAxisOverlap,
                        );
                    } else {
                        onAxisOverlap(context, min.index, overlappingBufferX, overlapsX);
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

fn sweepY(
    comptime T: type,
    index: usize,
    source: []T,
    overlappingBufferX: []bool,
    overlappingBufferY: []bool,
    context: anytype,
    onAxisOverlap: fn (context: @TypeOf(context), a: usize, bs: []bool, n: usize) void,
) void {
    const zone = ztracy.ZoneNC(@src(), "sweepY", 0xff_00_f0_f0);
    defer zone.End();

    var smallBuffer: [SMALL_LINE_BUFFER_LEN]SweepLine = undefined;

    var n: usize = 0;
    for (0..overlappingBufferX.len) |j| {
        if (!overlappingBufferX[j]) continue;

        copySweepLine(T, .y, n, source[j], &smallBuffer);
        n += 1;

        //onAxisOverlap(context, min.index, j);
    }
    const sortedSmallBuffer = smallBuffer[0..n];
    std.mem.sort(SweepLine, sortedSmallBuffer, {}, minValue);

    for (0..overlappingBufferY.len) |i| {
        overlappingBufferY[i] = false;
    }

    var overlapsY: usize = 0;

    for (sortedSmallBuffer) |otherAxisLine| {
        switch (otherAxisLine) {
            .min => |minY| {
                if (overlapsY > 0) {
                    onAxisOverlap(context, index, overlappingBufferY, overlapsY);
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

fn copySweepLines(comptime T: type, comptime axis: Axis, source: []T, buffer: []SweepLine) []SweepLine {
    const zone = ztracy.ZoneNC(@src(), "copy sweep lines", 0xff_00_f0_f0);
    defer zone.End();

    for (0.., source) |i, s| {
        copySweepLine(T, axis, i, s, buffer);
    }

    return buffer[0 .. source.len * 2];
}

fn copySweepLine(comptime T: type, comptime axis: Axis, index: usize, source: T, buffer: []SweepLine) void {
    const zone = ztracy.ZoneNC(@src(), "copy sweep line", 0xff_00_f0_f0);
    defer zone.End();

    const min = SweepLine{
        .min = .{
            .index = index,
            .value = if (axis == .x) source.aabb.tl.x else source.aabb.tl.y,
        },
    };
    const max = SweepLine{
        .max = .{
            .index = index,
            .value = if (axis == .x) source.aabb.br.x else source.aabb.br.y,
        },
    };

    const j = index * 2;
    buffer[j] = min;
    buffer[j + 1] = max;
}

fn minValue(_: void, lhs: SweepLine, rhs: SweepLine) bool {
    return lhs.value() < rhs.value();
}
