const std = @import("std");
const rl = @import("raylib");
const zlm = @import("zlm");
const ecs = @import("ecs");

const RigidBody = @import("physics/rigid-body.zig").RigidBodyFlat;
const Camera = @import("camera.zig").Camera;

const cfg = @import("config.zig");

const screenPosition = @import("screen.zig").screenPosition;
const screenPositionV = @import("screen.zig").screenPositionV;

pub const DrawSystem = struct {
    view: ecs.BasicView(RigidBody),

    pub fn init(reg: *ecs.Registry) DrawSystem {
        return DrawSystem{
            .view = reg.basicView(RigidBody),
        };
    }

    pub fn draw(self: *const DrawSystem, camera: Camera) void {
        for (self.view.raw()) |body| {
            DrawSystem.drawShape(body, camera);
        }
    }

    pub fn drawTexture(texture: *const rl.Texture2D, position: rl.Vector2, rotation: f32, scale: f32, camera: Camera) void {
        const s = scale * camera.s;
        const r = rotation + camera.angle();
        const w = @as(f32, @floatFromInt(texture.width));
        const h = @as(f32, @floatFromInt(texture.height));
        const p = rl.Vector2.init(position.x - (w / 2) * scale, position.y - (h / 2) * scale);

        const screenP = camera.v(screenPosition(p.x * camera.s, p.y * camera.s));
        const screenS = rl.Vector2.init(w * s, h * s);

        const source = rl.Rectangle.init(0, 0, w, h);
        const dest = rl.Rectangle.init(screenP.x, screenP.y, screenS.x, screenS.y);
        const origin = rl.Vector2.init(-dest.width / 2, -dest.height / 2);

        rl.drawTexturePro(texture.*, source, dest, origin, r, rl.Color.white);
    }

    pub fn drawShape(rb: RigidBody, camera: Camera) void {
        switch (rb.s.shape) {
            .circle => |circle| drawCircle(circle, rb, camera),
            .rectangle => |rectangle| drawRectangle(rectangle, rb, camera),
        }
    }

    pub fn drawCircle(circle: RigidBody.Shape.Circle, rb: RigidBody, camera: Camera) void {
        const s = camera.s;

        const screenP = camera.v(screenPosition(rb.d.p.x, rb.d.p.y));

        rl.drawCircleLinesV(screenP, circle.radius * s, rl.Color.white);
    }

    pub fn drawRectangle(rect: RigidBody.Shape.Rectangle, rb: RigidBody, camera: Camera) void {
        const s = camera.s;
        const p = rb.d.p.sub(rect.size.scale(1 / 2));

        const screenP = camera.v(screenPosition(p.x, p.y));

        var transformedVertices: [5]rl.Vector2 = undefined;

        for (0..4) |i| {
            const v = rect.vertices[i].rotate(rb.d.r).scale(s).add(zlm.vec2(screenP.x, screenP.y));
            transformedVertices[i] = rl.Vector2.init(v.x, v.y);
        }
        transformedVertices[4] = transformedVertices[0];

        rl.drawLineStrip(&transformedVertices, rl.Color.white);
    }

    fn drawDebugBorder(camera: Camera) void {
        const lw = 4;
        const lo = lw / 2;
        const tl = screenPositionV(camera.vxy(-lo, -lo));
        const br = screenPositionV(camera.vxy(cfg.size.w + lo, cfg.size.h + lo));

        const bl = rl.Vector2.init(tl.x, br.y);
        const tr = rl.Vector2.init(br.x, tl.y);

        rl.drawLineEx(tl, bl, lw, rl.Color.red);
        rl.drawLineEx(tl, tr, lw, rl.Color.red);
        rl.drawLineEx(bl, br, lw, rl.Color.red);
        rl.drawLineEx(tr, br, lw, rl.Color.red);
    }
};
