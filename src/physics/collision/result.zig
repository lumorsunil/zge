const Vector = @import("../../vector.zig").Vector;
const V = @import("../../vector.zig").V;

const RigidBody = @import("../rigid-body-flat.zig").RigidBodyFlat;
const AABB = @import("../shape.zig").AABB;

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

    pub fn collisionSpeed(self: Collision) f32 {
        const dotA = V.dot(self.velocityA, self.normal);
        const dotB = V.dot(self.velocityB, self.normal);

        return @abs(dotA - dotB);
    }
};

pub const CollisionStatic = struct {
    body: *RigidBody,
    normal: Vector,
    depth: f32,
    velocity: Vector = V.zero,
    contact1: Vector,
    contact2: Vector,
    contactCount: u2,

    pub fn collisionSpeed(self: CollisionStatic) f32 {
        return V.dot(self.velocity, self.normal);
    }
};
