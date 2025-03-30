const Vector = @import("../../vector.zig").Vector;
const V = @import("../../vector.zig").V;

const RigidBody = @import("../rigid-body-flat.zig").RigidBodyFlat;

pub const CollisionResult = union(enum) {
    collision: Collision,
    noCollision,
};

pub const Collision = struct {
    bodyA: *RigidBody,
    bodyB: *RigidBody,
    normal: Vector,
    depth: f32,
    velocityA: Vector = V.zero,
    velocityB: Vector = V.zero,
    contact1: Vector,
    contact2: Vector,
    contactCount: u2,
};
