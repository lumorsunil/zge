const zlm = @import("zlm");

const RigidBody = @import("../rigid-body-flat.zig").RigidBodyFlat;

pub const CollisionResult = union(enum) {
    collision: Collision,
    noCollision,
};

pub const Collision = struct {
    bodyA: *RigidBody,
    bodyB: *RigidBody,
    normal: zlm.Vec2,
    depth: f32,
    contact1: zlm.Vec2,
    contact2: zlm.Vec2,
    contactCount: u2,
};
