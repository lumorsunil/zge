const std = @import("std");
const Allocator = std.mem.Allocator;
const ArrayList = std.ArrayList;

pub const SizeOption = union(enum) {
    default,
    custom: usize,
};

pub fn VectorArrayList(
    comptime E: type,
    comptime default_value: E,
    comptime size_opt: SizeOption,
) type {
    return struct {
        list: ArrayList(Vector),

        const VECTOR_SIZE = switch (size_opt) {
            .default => 2048,
            .custom => |n| n,
        };
        const VECTOR_ELEMENTS = @divExact(VECTOR_SIZE, @bitSizeOf(E));
        const INITIAL_ELEMENTS_CAPACITY = 1024;
        const INITIAL_CAPACITY = elementsToNumberOfVectors(INITIAL_ELEMENTS_CAPACITY);

        pub const Vector = @Vector(VECTOR_ELEMENTS, E);

        fn elementsToNumberOfVectors(elements: usize) usize {
            return elementToVectorIndex(elements) + 1;
        }

        fn elementToVectorIndex(element: usize) usize {
            return element / VECTOR_ELEMENTS;
        }

        fn elementToLocalIndex(element: usize) usize {
            return element % VECTOR_ELEMENTS;
        }

        pub const Accessor = struct {
            vectorIndex: usize,
            elementIndex: usize,

            pub fn from(element: usize) Accessor {
                return Accessor{
                    .vectorIndex = elementToVectorIndex(element),
                    .elementIndex = elementToLocalIndex(element),
                };
            }
        };

        pub const empty = @This(){ .list = .empty };

        pub fn init(allocator: Allocator) @This() {
            return .{
                .list = ArrayList(Vector).initCapacity(allocator, INITIAL_CAPACITY) catch unreachable,
            };
        }

        pub fn initCapacity(allocator: Allocator, capacity: usize) @This() {
            const vectorCapacity = elementsToNumberOfVectors(capacity);

            return .{
                .list = ArrayList(Vector).initCapacity(allocator, vectorCapacity) catch unreachable,
            };
        }

        pub fn deinit(self: *@This(), allocator: Allocator) void {
            self.list.deinit(allocator);
        }

        pub fn ensureCapacity(
            self: *@This(),
            allocator: Allocator,
            capacity: usize,
            isPointersInvalidated: ?*bool,
        ) void {
            const prevPtr = self.list.items.ptr;
            self.list.ensureTotalCapacity(allocator, elementsToNumberOfVectors(capacity)) catch unreachable;
            self.list.expandToCapacity();
            const newPtr = self.list.items.ptr;

            if (prevPtr != newPtr and isPointersInvalidated != null) {
                isPointersInvalidated.?.* = true;
            }
        }

        /// Slow operation
        pub fn getAccessor(
            self: *@This(),
            element: usize,
        ) ?Accessor {
            if (self.hasCapacity(element + 1)) {
                self.list.expandToCapacity();
                return Accessor.from(element);
            } else {
                return null;
            }
        }

        pub fn hasCapacity(self: @This(), capacity: usize) bool {
            return elementsToNumberOfVectors(capacity) <= self.list.capacity;
        }

        /// Slow operation
        pub fn get(self: *@This(), element: usize) E {
            if (self.getAccessor(element)) |accessor| {
                return self.getA(accessor);
            } else {
                return default_value;
            }
        }

        pub fn getA(self: *@This(), accessor: Accessor) E {
            const vector_type = @typeInfo(Vector).vector;
            const array: *[vector_type.len]vector_type.child = @ptrCast(&self.list.items[accessor.vectorIndex]);
            return array[accessor.elementIndex];
            // return self.list.items[accessor.vectorIndex][accessor.elementIndex];
        }

        /// Slow operation
        pub fn getP(self: *@This(), allocator: Allocator, element: usize) *E {
            self.ensureCapacity(allocator, element + 1, null);
            return self.getPA(self.getAccessor(element).?);
        }

        pub fn getPA(self: *@This(), accessor: Accessor) *E {
            const vector_type = @typeInfo(Vector).vector;
            const array: *[vector_type.len]vector_type.child = @ptrCast(&self.list.items[accessor.vectorIndex]);
            return &array[accessor.elementIndex];
            // return &self.list.items[accessor.vectorIndex][accessor.elementIndex];
        }

        /// Slow operation, only intended for initializing the vector and should not be called frequently
        pub fn set(
            self: *@This(),
            allocator: Allocator,
            element: usize,
            value: E,
            isPointersInvalidated: ?*bool,
        ) void {
            self.ensureCapacity(allocator, element + 1, isPointersInvalidated);
            self.setA(self.getAccessor(element).?, value);
        }

        pub fn setA(self: *@This(), accessor: Accessor, value: E) void {
            const vector_type = @typeInfo(Vector).vector;
            const array: *[vector_type.len]vector_type.child = @ptrCast(&self.list.items[accessor.vectorIndex]);
            array[accessor.elementIndex] = value;
            // self.list.items[accessor.vectorIndex][accessor.elementIndex] = value;
        }

        pub fn iterate(
            self: *@This(),
            other: *const @This(),
            f: fn (a: *Vector, b: *Vector) void,
        ) void {
            std.debug.assert(self.list.items.len == other.list.items.len);

            for (0..self.list.items.len) |i| {
                f(&self.list.items[i], &other.list.items[i]);
            }
        }

        pub fn iterateC(
            self: *@This(),
            comptime T: type,
            other: *const @This(),
            context: T,
            f: fn (context: T, a: *Vector, b: *Vector) void,
        ) void {
            std.debug.assert(self.list.items.len == other.list.items.len);

            for (0..self.list.items.len) |i| {
                f(context, &self.list.items[i], &other.list.items[i]);
            }
        }

        pub fn iterateScalar(
            self: *@This(),
            s: E,
            f: fn (a: *Vector, b: *Vector) void,
        ) void {
            var splat = @as(Vector, @splat(s));

            for (0..self.list.items.len) |i| {
                f(&self.list.items[i], &splat);
            }
        }

        pub fn iterateScalarC(
            self: *@This(),
            comptime T: type,
            s: E,
            context: T,
            f: fn (context: T, a: *Vector, b: *Vector) void,
        ) void {
            const splat = @as(Vector, @splat(s));

            for (0..self.list.items.len) |i| {
                f(context, &self.list.items[i], &splat);
            }
        }

        fn _add(a: *Vector, b: *Vector) void {
            a.* += b.*;
        }

        fn _sub(a: *Vector, b: *Vector) void {
            a.* -= b.*;
        }

        fn _mul(a: *Vector, b: *Vector) void {
            a.* *= b.*;
        }

        fn _set(a: *Vector, b: *Vector) void {
            a.* = b.*;
        }

        /// Vector addition
        pub fn add(self: *@This(), other: *const @This()) void {
            self.iterate(other, _add);
        }

        /// Vector addition
        pub fn addScalar(self: *@This(), s: E) void {
            self.iterateScalar(s, _add);
        }

        /// Vector subtraction
        pub fn sub(self: *@This(), other: *const @This()) void {
            self.iterate(other, _sub);
        }
        /// Vector scale
        pub fn mul(self: *@This(), other: *const @This()) void {
            self.iterate(other, _mul);
        }

        /// Vector scale
        pub fn scale(self: *@This(), s: E) void {
            self.iterateScalar(s, _mul);
        }

        /// Vector set to scalar
        pub fn setScalar(self: *@This(), s: E) void {
            self.iterateScalar(s, _set);
        }

        pub fn totalElements(self: *const @This()) usize {
            return self.list.items.len * VECTOR_ELEMENTS;
        }

        pub fn scalar(s: E) Vector {
            return @splat(s);
        }
    };
}
