const std = @import("std");
const Allocator = std.mem.Allocator;
const ArrayList = std.ArrayList;

const VECTOR_SIZE = 2048;
const VECTOR_ELEMENTS = @divExact(VECTOR_SIZE, @bitSizeOf(f32));
const INITIAL_ELEMENTS_CAPACITY = 1024;
const INITIAL_CAPACITY = elementsToNumberOfVectors(INITIAL_ELEMENTS_CAPACITY);

fn elementsToNumberOfVectors(elements: usize) usize {
    return elementToVectorIndex(elements) + 1;
}

fn elementToVectorIndex(element: usize) usize {
    return element / VECTOR_ELEMENTS;
}

fn elementToLocalIndex(element: usize) usize {
    return element % VECTOR_ELEMENTS;
}

pub const Vector = @Vector(VECTOR_ELEMENTS, f32);

pub const VectorArrayList = struct {
    list: ArrayList(Vector),
    allocator: Allocator,

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

    pub fn init(allocator: Allocator) VectorArrayList {
        return VectorArrayList{
            .list = ArrayList(Vector).initCapacity(allocator, INITIAL_CAPACITY) catch unreachable,
            .allocator = allocator,
        };
    }

    pub fn initCapacity(allocator: Allocator, capacity: usize) VectorArrayList {
        const vectorCapacity = elementsToNumberOfVectors(capacity);

        return VectorArrayList{
            .list = ArrayList(Vector).initCapacity(allocator, vectorCapacity) catch unreachable,
            .allocator = allocator,
        };
    }

    pub fn deinit(self: *VectorArrayList) void {
        self.list.deinit(self.allocator);
    }

    pub fn ensureCapacity(self: *VectorArrayList, capacity: usize, isPointersInvalidated: ?*bool) void {
        const prevPtr = self.list.items.ptr;
        self.list.ensureTotalCapacity(self.allocator, elementsToNumberOfVectors(capacity)) catch unreachable;
        self.list.expandToCapacity();
        const newPtr = self.list.items.ptr;

        if (prevPtr != newPtr and isPointersInvalidated != null) {
            isPointersInvalidated.?.* = true;
        }
    }

    /// Slow operation
    pub fn getAccessor(self: *VectorArrayList, element: usize, isPointersInvalidated: ?*bool) Accessor {
        self.ensureCapacity(element, isPointersInvalidated);
        return Accessor.from(element);
    }

    /// Slow operation
    pub fn get(self: *VectorArrayList, element: usize) f32 {
        return self.getA(self.getAccessor(element, null));
    }

    pub fn getA(self: *VectorArrayList, accessor: Accessor) f32 {
        return self.list.items[accessor.vectorIndex][accessor.elementIndex];
    }

    /// Slow operation
    pub fn getP(self: *VectorArrayList, element: usize) *f32 {
        return self.getPA(self.getAccessor(element, null));
    }

    pub fn getPA(self: *VectorArrayList, accessor: Accessor) *f32 {
        return &self.list.items[accessor.vectorIndex][accessor.elementIndex];
    }

    /// Slow operation, only intended for initializing the vector and should not be called frequently
    pub fn set(self: *VectorArrayList, element: usize, value: f32, isPointersInvalidated: ?*bool) void {
        self.setA(self.getAccessor(element, isPointersInvalidated), value);
    }

    pub fn setA(self: *VectorArrayList, accessor: Accessor, value: f32) void {
        self.list.items[accessor.vectorIndex][accessor.elementIndex] = value;
    }

    pub fn iterate(
        self: *VectorArrayList,
        other: *const VectorArrayList,
        f: fn (a: *Vector, b: *Vector) void,
    ) void {
        std.debug.assert(self.list.items.len == other.list.items.len);

        for (0..self.list.items.len) |i| {
            f(&self.list.items[i], &other.list.items[i]);
        }
    }

    pub fn iterateC(
        self: *VectorArrayList,
        comptime T: type,
        other: *const VectorArrayList,
        context: T,
        f: fn (context: T, a: *Vector, b: *Vector) void,
    ) void {
        std.debug.assert(self.list.items.len == other.list.items.len);

        for (0..self.list.items.len) |i| {
            f(context, &self.list.items[i], &other.list.items[i]);
        }
    }

    pub fn iterateScalar(
        self: *VectorArrayList,
        s: f32,
        f: fn (a: *Vector, b: *Vector) void,
    ) void {
        var splat = @as(Vector, @splat(s));

        for (0..self.list.items.len) |i| {
            f(&self.list.items[i], &splat);
        }
    }

    pub fn iterateScalarC(
        self: *VectorArrayList,
        comptime T: type,
        s: f32,
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
    pub fn add(self: *VectorArrayList, other: *const VectorArrayList) void {
        self.iterate(other, _add);
    }

    /// Vector addition
    pub fn addScalar(self: *VectorArrayList, s: f32) void {
        self.iterateScalar(s, _add);
    }

    /// Vector subtraction
    pub fn sub(self: *VectorArrayList, other: *const VectorArrayList) void {
        self.iterate(other, _sub);
    }
    /// Vector scale
    pub fn mul(self: *VectorArrayList, other: *const VectorArrayList) void {
        self.iterate(other, _mul);
    }

    /// Vector scale
    pub fn scale(self: *VectorArrayList, s: f32) void {
        self.iterateScalar(s, _mul);
    }

    /// Vector set to scalar
    pub fn setScalar(self: *VectorArrayList, s: f32) void {
        self.iterateScalar(s, _set);
    }

    pub fn totalElements(self: *const VectorArrayList) usize {
        return self.list.items.len * VECTOR_ELEMENTS;
    }
};
