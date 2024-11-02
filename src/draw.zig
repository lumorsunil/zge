const std = @import("std");
const ArrayList = std.ArrayList;
const Allocator = std.mem.Allocator;
const rl = @import("raylib");
const zlm = @import("zlm");
const ecs = @import("ecs");
const util = @import("util.zig");

const RigidBody = @import("physics/rigid-body-flat.zig").RigidBodyFlat;
const AABB = @import("physics/shape.zig").AABB;
const Circle = @import("physics/shape.zig").Circle;
const Rectangle = @import("physics/shape.zig").Rectangle;
const CollisionContainer = @import("physics/collision/container.zig").CollisionContainer;
const Camera = @import("camera.zig").Camera;

const cfg = @import("config.zig");

const screenPosition = @import("screen.zig").screenPosition;
const screenPositionV = @import("screen.zig").screenPositionV;

pub const DrawSystem = struct {
    view: ecs.BasicView(RigidBody),
    ccView: ecs.BasicView(CollisionContainer),
    drawOrderList: ArrayList(ecs.Entity),
    allocator: Allocator,

    pub fn init(allocator: Allocator, reg: *ecs.Registry) !DrawSystem {
        return DrawSystem{
            .view = reg.basicView(RigidBody),
            .ccView = reg.basicView(CollisionContainer),
            .drawOrderList = try ArrayList(ecs.Entity).initCapacity(allocator, 100),
            .allocator = allocator,
        };
    }

    pub fn deinit(self: DrawSystem) void {
        self.drawOrderList.deinit();
    }

    pub fn draw(self: *DrawSystem, camera: Camera) void {
        for (self.drawOrder()) |entity| {
            const body = self.view.getConst(entity);

            DrawSystem.drawShape(body, camera);
        }

        self.drawCollisionContainer(camera);
    }

    const ccEntryColor = rl.Color.lime;

    const ccLevelColors = [_]rl.Color{
        rl.Color.green,
        rl.Color.blue,
        rl.Color.red,
        rl.Color.yellow,
        rl.Color.magenta,
        rl.Color.orange,
        rl.Color.black,
    };

    fn drawCollisionContainer(self: *DrawSystem, camera: Camera) void {
        const cc = self.ccView.raw()[0];

        self.drawPage(cc.tree.root, 1, cc.tree.height, camera);
    }

    fn drawPage(self: *DrawSystem, page: *CollisionContainer.RTree.Page, level: usize, height: usize, camera: Camera) void {
        const color = ccLevelColors[height - level];

        self.drawAabb(page.aabb, color, camera);

        for (page.children.items) |child| {
            self.drawNode(child, level + 1, height, camera);
        }
    }

    fn drawNode(self: *DrawSystem, node: CollisionContainer.RTree.Node, level: usize, height: usize, camera: Camera) void {
        switch (node) {
            .page => |page| self.drawPage(page, level, height, camera),
            .entry => |entry| self.drawEntry(entry.entry, camera),
        }
    }

    fn drawEntry(self: *DrawSystem, entry: CollisionContainer.RTreeEntry, camera: Camera) void {
        self.drawAabb(CollisionContainer.entryAabb(entry), ccEntryColor, camera);
    }

    fn drawAabb(self: *DrawSystem, aabb: AABB, color: rl.Color, camera: Camera) void {
        _ = self;
        const p = screenPositionV(camera.vxy(aabb.tl.x, aabb.tl.y));
        rl.drawRectangleLinesEx(
            rl.Rectangle.init(
                p.x,
                p.y,
                aabb.width(),
                aabb.height(),
            ),
            1,
            color,
        );
    }

    fn minYComparer(self: *DrawSystem, a: ecs.Entity, b: ecs.Entity) bool {
        const bodyA = self.view.getConst(a);
        const bodyB = self.view.getConst(b);

        return bodyA.d.p.y.* < bodyB.d.p.y.*;
    }

    fn drawOrder(self: *DrawSystem) []const ecs.Entity {
        self.drawOrderList.clearRetainingCapacity();
        self.drawOrderList.ensureTotalCapacity(self.view.len()) catch unreachable;
        self.drawOrderList.expandToCapacity();

        util.sortTo(ecs.Entity, self.view.data(), self.drawOrderList.items[0..self.view.len()], self, minYComparer);

        return self.drawOrderList.items[0..self.view.len()];
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

    pub fn drawCircle(circle: Circle, rb: RigidBody, camera: Camera) void {
        const s = camera.s;

        const screenP = camera.v(screenPosition(rb.d.p.x.*, rb.d.p.y.*));

        rl.drawCircleLinesV(screenP, circle.radius * s, rl.Color.white);
    }

    pub fn drawRectangle(rect: Rectangle, rb: RigidBody, camera: Camera) void {
        const s = camera.s;
        const halfRect = rect.size.scale(1 / 2);
        const p = zlm.vec2(rb.d.p.x.*, rb.d.p.y.*).sub(halfRect);

        const screenP = camera.v(screenPosition(p.x, p.y));

        var transformedVertices: [5]rl.Vector2 = undefined;

        for (0..4) |i| {
            const v = rect.vertices[i].rotate(rb.d.r.*).scale(s).add(zlm.vec2(screenP.x, screenP.y));
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
