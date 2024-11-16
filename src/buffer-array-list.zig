const std = @import("std");

pub fn SliceArrayList(comptime T: type) type {
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
