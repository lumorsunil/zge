const RigidBodyStaticParams = @import("rigid-body-static.zig").RigidBodyStaticParams;
const RigidBodyDynamicParams = @import("rigid-body-dynamic.zig").RigidBodyDynamicParams;

pub const RigidBody = struct {
    static: *RigidBodyStaticParams,
    dynamic: *RigidBodyDynamicParams,
};
