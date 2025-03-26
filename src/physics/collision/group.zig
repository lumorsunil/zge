const std = @import("std");
const Allocator = std.mem.Allocator;
const ecs = @import("ecs");

pub const EntityGroup = struct {
    entities: std.AutoArrayHashMap(ecs.Entity, bool),

    pub fn init(allocator: Allocator) EntityGroup {
        return EntityGroup{
            .entities = std.AutoArrayHashMap(ecs.Entity, bool).init(allocator),
        };
    }

    pub fn deinit(self: *EntityGroup) void {
        self.entities.deinit();
    }
};

pub const CollisionEnabledFor = struct {
    groups: std.StringHashMap(EntityGroup),
    collisionsEnabledFor: std.AutoArrayHashMap(ecs.Entity, std.ArrayList(*EntityGroup)),
    collisionsEnabledForGroups: std.StringArrayHashMap(std.ArrayList(*[]const u8)),

    pub fn init(allocator: Allocator) CollisionEnabledFor {
        return CollisionEnabledFor{
            .groups = std.StringHashMap(EntityGroup).init(allocator),
            .collisionsEnabledFor = std.AutoArrayHashMap(ecs.Entity, std.ArrayList(*EntityGroup)).init(allocator),
            .collisionsEnabledForGroups = std.StringArrayHashMap(std.ArrayList(*[]const u8)).init(allocator),
        };
    }

    pub fn deinit(self: *CollisionEnabledFor) void {
        var it = self.groups.valueIterator();
        while (it.next()) |group| {
            group.deinit();
        }
        self.groups.deinit();
        for (self.collisionsEnabledFor.values()) |item| {
            item.deinit();
        }
        self.collisionsEnabledFor.deinit();
        for (self.collisionsEnabledForGroups.values()) |item| {
            item.deinit();
        }
        self.collisionsEnabledForGroups.deinit();
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

        const cef = self.collisionsEnabledFor.getOrPut(entity) catch unreachable;

        if (!cef.found_existing) {
            cef.value_ptr.* = std.ArrayList(*EntityGroup).init(allocator);
        }

        if (std.mem.indexOfScalar(*EntityGroup, cef.value_ptr.items, group) == null) {
            cef.value_ptr.append(group) catch unreachable;
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

        const cef = self.collisionsEnabledForGroups.getOrPut(groupAKey) catch unreachable;

        if (!cef.found_existing) {
            cef.value_ptr.* = std.ArrayList(*[]const u8).init(allocator);
        }

        if (std.mem.indexOfScalar(*[]const u8, cef.value_ptr.items, groupBKeyPtr) == null) {
            cef.value_ptr.append(groupBKeyPtr) catch unreachable;
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
        const g = self.groups.getOrPut(groupKey) catch unreachable;

        if (g.found_existing) {
            std.log.warn("Group already created: {s}", .{groupKey});
        } else {
            g.value_ptr.* = EntityGroup.init(allocator);
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
        const addedToGroup = group.entities.getOrPut(entity) catch unreachable;
        if (addedToGroup.found_existing) {
            std.log.warn("Entity {d} already added to group: {s}", .{ entity, groupKey });
        }

        const cef = self.collisionsEnabledForGroups.getPtr(groupKey) orelse return;

        for (cef.items) |g| {
            self.enableCollisionsFor(allocator, entity, g.*);
        }
    }

    pub fn removeFromGroup(
        self: *CollisionEnabledFor,
        entity: ecs.Entity,
        groupKey: []const u8,
    ) void {
        const group = self.groups.getPtr(groupKey) orelse {
            std.log.err("Group not found: {s}", .{groupKey});
            return;
        };
        _ = group.entities.swapRemove(entity);
        const r = self.collisionsEnabledFor.fetchSwapRemove(entity) orelse return;
        r.value.deinit();
    }

    pub fn removeFromAllGroups(
        self: *CollisionEnabledFor,
        entity: ecs.Entity,
    ) void {
        var it = self.groups.valueIterator();
        while (it.next()) |group| {
            _ = group.entities.swapRemove(entity);
        }
        const r = self.collisionsEnabledFor.fetchSwapRemove(entity) orelse return;
        r.value.deinit();
    }
};
