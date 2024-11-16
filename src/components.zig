const rl = @import("raylib");

pub const TextureComponent = struct {
    texture: *const rl.Texture2D,

    pub fn init(texture: *const rl.Texture2D) TextureComponent {
        return TextureComponent{ .texture = texture };
    }
};
