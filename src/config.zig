const V = @import("vector.zig").V;
const CollisionContainer_ = @import("physics/collision/container.zig").CollisionContainer;
const PhysicsSystem_ = @import("physics.zig").PhysicsSystem;
const DrawSystem_ = @import("draw.zig").DrawSystem;

pub const size = V.init(1024, 1024);
pub const sizeHalf = size / V.scalar(2);

pub const MAX_ENTITIES = 10000;

pub const isTracingEnabled = true;

pub const ZGEConfig = struct {
    physics: ZGEPhysicsConfig = .{},

    pub fn PhysicsSystem(comptime self: @This()) type {
        return PhysicsSystem_(self);
    }

    pub fn CollisionContainer(comptime self: @This()) type {
        return CollisionContainer_(self);
    }

    pub fn DrawSystem(comptime self: @This()) type {
        return DrawSystem_(self);
    }

    pub fn getDynamicBodiesAlgorithm(comptime self: @This()) CollisionContainerAlgorithm {
        return self.physics.collision.dynamic_bodies_algorithm;
    }

    pub fn isGridHashEnabled(comptime self: @This()) bool {
        return self.physics.collision.enable_grid_hash;
    }
};

pub const ZGEPhysicsConfig = struct {
    collision: ZGECollisionConfig = .{},
};

pub const ZGECollisionConfig = struct {
    dynamic_bodies_algorithm: CollisionContainerAlgorithm = .quadTree,
    enable_grid_hash: bool = false,
};

pub const CollisionContainerAlgorithm = enum {
    rTree,
    quadTree,
};
