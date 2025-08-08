const rl = @import("raylib");

const AABB = @import("physics/shape.zig").AABB;

const Vector = @import("vector.zig").Vector;

pub const TextureComponent = struct {
    texture: rl.Texture2D,
    sourceRectangle: ?AABB,
    origin: Vector,
    horizontalFlip: bool,

    pub fn init(texture: rl.Texture2D, sourceRectangle: ?AABB, origin: Vector) TextureComponent {
        return TextureComponent{
            .texture = texture,
            .sourceRectangle = sourceRectangle,
            .origin = origin,
            .horizontalFlip = false,
        };
    }
};

var drawLayerSubZCounter: u32 = 0;

pub const DrawLayerComponent = struct {
    z: u32,
    subZ: u32,

    pub fn init(z: u32) DrawLayerComponent {
        drawLayerSubZCounter +%= 1;
        return DrawLayerComponent{
            .z = z,
            .subZ = drawLayerSubZCounter,
        };
    }
};

pub const Invisible = struct {
    isInvisible: bool = true,
};
