const std = @import("std");
const Allocator = std.mem.Allocator;
const ecs = @import("ecs");

pub const EntityGroup = struct {
    entities: std.array_hash_map.Auto(ecs.Entity, bool),

    pub const empty: @This() = .{ .entities = .empty };

    pub fn deinit(self: *EntityGroup, allocator: Allocator) void {
        self.entities.deinit(allocator);
    }
};

pub const CollisionEnabledFor = struct {
    groups: std.hash_map.StringHashMapUnmanaged(EntityGroup),
    collisionsEnabledFor: std.array_hash_map.Auto(ecs.Entity, std.ArrayList(*EntityGroup)),
    collisionsEnabledForGroups: std.array_hash_map.String(std.ArrayList(*[]const u8)),

    pub const empty = @This(){
        .groups = .empty,
        .collisionsEnabledFor = .empty,
        .collisionsEnabledForGroups = .empty,
    };

    pub fn deinit(self: *CollisionEnabledFor, allocator: Allocator) void {
        var it = self.groups.valueIterator();
        while (it.next()) |group| {
            group.deinit(allocator);
        }
        self.groups.deinit(allocator);
        for (self.collisionsEnabledFor.values()) |*item| {
            item.deinit(allocator);
        }
        self.collisionsEnabledFor.deinit(allocator);
        for (self.collisionsEnabledForGroups.values()) |*item| {
            item.deinit(allocator);
        }
        self.collisionsEnabledForGroups.deinit(allocator);
    }

    pub fn enableCollisionsFor(
        self: *CollisionEnabledFor,
        allocator: Allocator,
        entity: ecs.Entity,
        groupKey: []const u8,
    ) void {
        const group = self.groups.getPtr(groupKey) orelse {
            std.log.warn("Group not found: {s}", .{groupKey});
            return;
        };

        const cef = self.collisionsEnabledFor.getOrPut(allocator, entity) catch unreachable;

        if (!cef.found_existing) {
            cef.value_ptr.* = .empty;
        }

        if (std.mem.indexOfScalar(*EntityGroup, cef.value_ptr.items, group) == null) {
            cef.value_ptr.append(allocator, group) catch unreachable;
        }
    }

    pub fn enableCollisionsForGroup(
        self: *CollisionEnabledFor,
        allocator: Allocator,
        groupAKey: []const u8,
        groupBKey: []const u8,
    ) void {
        const groupA = self.groups.getPtr(groupAKey) orelse {
            std.log.warn("Group not found: {s}", .{groupAKey});
            return;
        };
        const groupBKeyPtr = self.groups.getKeyPtr(groupBKey) orelse {
            std.log.warn("Group not found: {s}", .{groupBKey});
            return;
        };

        for (groupA.entities.keys()) |entity| {
            self.enableCollisionsFor(allocator, entity, groupBKey);
        }

        const cef = self.collisionsEnabledForGroups.getOrPut(allocator, groupAKey) catch unreachable;

        if (!cef.found_existing) {
            cef.value_ptr.* = .empty;
        }

        if (std.mem.indexOfScalar(*[]const u8, cef.value_ptr.items, groupBKeyPtr) == null) {
            cef.value_ptr.append(allocator, groupBKeyPtr) catch unreachable;
        }
    }

    pub fn isCollisionEnabledFor(
        self: CollisionEnabledFor,
        entityA: ecs.Entity,
        entityB: ecs.Entity,
    ) bool {
        const groupsA = self.collisionsEnabledFor.get(entityA);
        if (groupsA) |g| for (g.items) |eg| if (eg.entities.contains(entityB)) return true;
        const groupsB = self.collisionsEnabledFor.get(entityB);
        if (groupsB) |g| for (g.items) |eg| if (eg.entities.contains(entityA)) return true;

        return false;
    }

    pub fn createGroup(self: *CollisionEnabledFor, allocator: Allocator, groupKey: []const u8) void {
        const g = self.groups.getOrPut(allocator, groupKey) catch unreachable;

        if (g.found_existing) {
            std.log.warn("Group already created: {s}", .{groupKey});
        } else {
            g.value_ptr.* = .empty;
        }
    }

    pub fn addToGroup(
        self: *CollisionEnabledFor,
        allocator: Allocator,
        entity: ecs.Entity,
        groupKey: []const u8,
    ) void {
        const group = self.groups.getPtr(groupKey) orelse {
            std.log.err("Group not found: {s}", .{groupKey});
            return;
        };
        const addedToGroup = group.entities.getOrPut(allocator, entity) catch unreachable;
        if (addedToGroup.found_existing) {
            std.log.warn("Entity {} already added to group: {s}", .{ entity, groupKey });
        }

        const cef = self.collisionsEnabledForGroups.getPtr(groupKey) orelse return;

        for (cef.items) |g| {
            self.enableCollisionsFor(allocator, entity, g.*);
        }
    }

    pub fn removeFromGroup(
        self: *CollisionEnabledFor,
        allocator: Allocator,
        entity: ecs.Entity,
        groupKey: []const u8,
    ) void {
        const group = self.groups.getPtr(groupKey) orelse {
            std.log.err("Group not found: {s}", .{groupKey});
            return;
        };
        _ = group.entities.swapRemove(entity);
        var r = self.collisionsEnabledFor.fetchSwapRemove(entity) orelse return;
        r.value.deinit(allocator);
    }

    pub fn removeFromAllGroups(
        self: *CollisionEnabledFor,
        allocator: Allocator,
        entity: ecs.Entity,
    ) void {
        var it = self.groups.valueIterator();
        while (it.next()) |group| {
            _ = group.entities.swapRemove(entity);
        }
        var r = self.collisionsEnabledFor.fetchSwapRemove(entity) orelse return;
        r.value.deinit(allocator);
    }
};
