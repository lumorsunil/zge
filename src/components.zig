const rl = @import("raylib");

const AABB = @import("physics/shape.zig").AABB;

const Vector = @import("vector.zig").Vector;

pub const TextureComponent = struct {
    texture: *const rl.Texture2D,
    sourceRectangle: ?AABB,
    origin: Vector,
    horizontalFlip: bool,

    pub fn init(texture: *const rl.Texture2D, sourceRectangle: ?AABB, origin: Vector) TextureComponent {
        return TextureComponent{
            .texture = texture,
            .sourceRectangle = sourceRectangle,
            .origin = origin,
            .horizontalFlip = false,
        };
    }
};

pub const Invisible = struct {
    isInvisible: bool = true,
};
