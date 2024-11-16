const std = @import("std");
const ArrayList = std.ArrayList;
const Allocator = std.mem.Allocator;
const rl = @import("raylib");
const zlm = @import("zlm");
const ecs = @import("ecs");
const util = @import("util.zig");
const vector = @import("vector.zig");

const RigidBody = @import("physics/rigid-body-flat.zig").RigidBodyFlat;
const AABB = @import("physics/shape.zig").AABB;
const Circle = @import("physics/shape.zig").Circle;
const Rectangle = @import("physics/shape.zig").Rectangle;
const CollisionContainer = @import("physics/collision/container.zig").CollisionContainer;
const ccAlgorithm = @import("physics/collision/container.zig").ccAlgorithm;
const Camera = @import("camera.zig").Camera;
const Screen = @import("screen.zig").Screen;
const TextureComponent = @import("components.zig").TextureComponent;

const z2r = @import("vector.zig").z2r;
const z2rect = @import("vector.zig").z2rect;

pub const DrawSystem = struct {
    screen: *const Screen,
    reg: *ecs.Registry,
    view: ecs.BasicView(RigidBody),
    textureView: ecs.MultiView(2, 0),
    shapeView: ecs.MultiView(1, 1),
    ccView: ecs.BasicView(CollisionContainer),
    drawOrderList: ArrayList(ecs.Entity),
    allocator: Allocator,

    var showQTGrid = false;
    var showCollisionBoxes = false;

    pub fn init(allocator: Allocator, reg: *ecs.Registry, screen: *const Screen) DrawSystem {
        return DrawSystem{
            .screen = screen,
            .reg = reg,
            .view = reg.basicView(RigidBody),
            .textureView = reg.view(.{ TextureComponent, RigidBody }, .{}),
            .shapeView = reg.view(.{RigidBody}, .{TextureComponent}),
            .ccView = reg.basicView(CollisionContainer),
            .drawOrderList = ArrayList(ecs.Entity).initCapacity(allocator, 100) catch unreachable,
            .allocator = allocator,
        };
    }

    pub fn deinit(self: *DrawSystem) void {
        self.unbind();
        self.drawOrderList.deinit();
    }

    pub fn bind(self: *DrawSystem) void {
        self.reg.onConstruct(RigidBody).connectBound(self, DrawSystem.ensureDrawOrderCapacity);
    }

    fn unbind(self: *DrawSystem) void {
        self.reg.onConstruct(RigidBody).disconnectBound(self);
    }

    pub fn draw(self: *DrawSystem, camera: Camera) void {
        if (rl.isKeyPressed(rl.KeyboardKey.key_q)) {
            showQTGrid = !showQTGrid;
        }
        if (rl.isKeyPressed(rl.KeyboardKey.key_c)) {
            showCollisionBoxes = !showCollisionBoxes;
        }

        for (self.drawOrder()) |entity| {
            const body = self.view.get(entity);
            const maybeTexture = self.reg.tryGetConst(TextureComponent, entity);

            if (maybeTexture) |texture| {
                self.drawTexture(
                    texture.texture,
                    body.d.clonePos(),
                    body.d.r.*,
                    body.d.s,
                    camera,
                );
            }

            if (showCollisionBoxes) {
                self.drawShape(body, camera);
            }
        }

        if (showQTGrid) {
            self.drawCollisionContainer(camera);
        }
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

        if (ccAlgorithm == .rTree) {
            self.drawPage(cc.tree.root, 1, cc.tree.height, camera);
        } else {
            self.drawPageQT(cc.tree.getRoot(), 0, camera);
        }
    }

    fn drawPageQT(self: *DrawSystem, maybePage: ?*CollisionContainer.QT.Page, level: usize, camera: Camera) void {
        if (maybePage == null) return;
        const page = maybePage.?;

        const color = ccLevelColors[@min(level, ccLevelColors.len - 1)];

        self.drawPageQT(page.quadrants.tl, level + 1, camera);
        self.drawPageQT(page.quadrants.tr, level + 1, camera);
        self.drawPageQT(page.quadrants.bl, level + 1, camera);
        self.drawPageQT(page.quadrants.br, level + 1, camera);

        self.drawAabb(page.aabb, color, 2, camera);
    }

    fn drawPage(self: *DrawSystem, page: *CollisionContainer.RTree.Page, level: usize, height: usize, camera: Camera) void {
        std.debug.assert(level <= height);
        const color = ccLevelColors[@min(height - level, ccLevelColors.len - 1)];

        self.drawAabb(page.aabb, color, @as(f32, @floatFromInt(page.calculateHeight())) - 1, camera);

        for (page.children.items) |child| {
            self.drawNode(child.*, level + 1, height, camera);
        }
    }

    fn drawNode(self: *DrawSystem, node: CollisionContainer.RTree.Node, level: usize, height: usize, camera: Camera) void {
        switch (node) {
            .page => |page| self.drawPage(page, level, height, camera),
            .entry => |entry| self.drawEntry(entry.entry, camera),
        }
    }

    fn drawEntry(self: *DrawSystem, entry: CollisionContainer.RTreeEntry, camera: Camera) void {
        self.drawAabb(CollisionContainer.entryAabb(entry), ccEntryColor, 1, camera);
    }

    fn drawAabb(self: *DrawSystem, aabb: AABB, color: rl.Color, thickness: f32, camera: Camera) void {
        const p = self.screen.screenPositionV(camera.transformV(aabb.tl));
        rl.drawRectangleLinesEx(
            rl.Rectangle.init(
                p.x,
                p.y,
                aabb.width(),
                aabb.height(),
            ),
            thickness,
            color,
        );
    }

    fn minYComparer(self: *DrawSystem, a: ecs.Entity, b: ecs.Entity) bool {
        const bodyA = self.view.getConst(a);
        const bodyB = self.view.getConst(b);

        return bodyA.d.p.y.* < bodyB.d.p.y.*;
    }

    fn ensureDrawOrderCapacity(self: *DrawSystem, reg: *ecs.Registry, entity: ecs.Entity) void {
        _ = entity;
        _ = reg;

        self.drawOrderList.clearRetainingCapacity();
        self.drawOrderList.ensureTotalCapacity(self.view.len()) catch unreachable;
        self.drawOrderList.expandToCapacity();
    }

    fn drawOrder(self: *DrawSystem) []const ecs.Entity {
        util.sortTo(ecs.Entity, self.view.data(), self.drawOrderList.items[0..self.view.len()], self, minYComparer);

        return self.drawOrderList.items[0..self.view.len()];
    }

    pub fn drawTexture(
        self: DrawSystem,
        texture: *const rl.Texture2D,
        position: zlm.Vec2,
        rotation: f32,
        scale: f32,
        camera: Camera,
    ) void {
        const s = scale * camera.s;
        const r = rotation + camera.angle();
        const w = @as(f32, @floatFromInt(texture.width));
        const h = @as(f32, @floatFromInt(texture.height));
        const textureSize = zlm.vec2(w, h);
        const p = position.sub(textureSize.scale(0.5 * scale));

        const screenP = camera.transformV(self.screen.screenPositionV(p));
        const screenS = textureSize.scale(s);

        const source = z2rect(zlm.Vec2.zero, textureSize);
        const dest = z2rect(screenP, screenS);
        const origin = zlm.Vec2.zero;

        rl.drawTexturePro(texture.*, source, dest, z2r(origin), r, rl.Color.white);
    }

    pub fn drawShape(self: DrawSystem, rb: *RigidBody, camera: Camera) void {
        switch (rb.s.shape) {
            .circle => |circle| self.drawCircle(circle, rb, camera),
            .rectangle => |rectangle| self.drawRectangle(rectangle, rb, camera),
        }
    }

    pub fn drawCircle(self: DrawSystem, circle: Circle, rb: *RigidBody, camera: Camera) void {
        const s = camera.s;

        const screenP = camera.transformV(self.screen.screenPosition(rb.d.p.x.*, rb.d.p.y.*));

        rl.drawCircleLinesV(z2r(screenP), circle.radius * s, rl.Color.white);
    }

    pub fn drawRectangle(self: DrawSystem, _: Rectangle, rb: *RigidBody, camera: Camera) void {
        var transformedVertices: [5]rl.Vector2 = .{
            vector.z2r(self.screen.screenPositionV(camera.transformV(rb.aabb.tl))),
            vector.z2r(self.screen.screenPositionV(camera.transformV(rb.aabb.tr()))),
            vector.z2r(self.screen.screenPositionV(camera.transformV(rb.aabb.br))),
            vector.z2r(self.screen.screenPositionV(camera.transformV(rb.aabb.bl()))),
            vector.z2r(self.screen.screenPositionV(camera.transformV(rb.aabb.tl))),
        };

        const color = if (ccAlgorithm == .quadTree)
            if (self.ccView.raw()[0].tree.isEntryInTree(rb)) rl.Color.white else rl.Color.red
        else
            rl.Color.white;

        rl.drawLineStrip(&transformedVertices, color);
    }

    fn drawDebugBorder(self: DrawSystem, camera: Camera) void {
        const lw = 4;
        const lo = lw / 2;
        const tl = self.screen.screenPositionV(camera.transformV(-lo, -lo));
        const br = self.screen.screenPositionV(camera.transformV(self.screen.size.w + lo, self.screen.size.h + lo));

        const bl = rl.Vector2.init(tl.x, br.y);
        const tr = rl.Vector2.init(br.x, tl.y);

        rl.drawLineEx(tl, bl, lw, rl.Color.red);
        rl.drawLineEx(tl, tr, lw, rl.Color.red);
        rl.drawLineEx(bl, br, lw, rl.Color.red);
        rl.drawLineEx(tr, br, lw, rl.Color.red);
    }
};
