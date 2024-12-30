const std = @import("std");
const ArrayList = std.ArrayList;
const Allocator = std.mem.Allocator;
const rl = @import("raylib");
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
const Invisible = @import("components.zig").Invisible;
const DrawLayerComponent = @import("components.zig").DrawLayerComponent;

const V = @import("vector.zig").V;
const Vector = @import("vector.zig").Vector;

pub const DrawSystem = struct {
    screen: *const Screen,
    camera: *Camera,
    reg: *ecs.Registry,
    bodyView: ecs.BasicView(RigidBody),
    layerView: ecs.BasicView(DrawLayerComponent),
    textureView: ecs.MultiView(3, 0),
    topLayerView: ecs.MultiView(1, 1),
    shapeView: ecs.MultiView(1, 1),
    ccView: ecs.BasicView(CollisionContainer),
    drawOrderList: ArrayList(ecs.Entity),
    allocator: Allocator,

    var showQTGrid = false;
    var showCollisionBoxes = false;
    var drawLayersSlowly = false;
    var layersBeingDrawn: usize = 0;
    const layerDrawSpeed = 2;
    var nextLayerDrawnAt: f64 = layerDrawSpeed;

    pub fn init(allocator: Allocator, reg: *ecs.Registry, screen: *const Screen, camera: *Camera) DrawSystem {
        return DrawSystem{
            .screen = screen,
            .camera = camera,
            .reg = reg,
            .bodyView = reg.basicView(RigidBody),
            .layerView = reg.basicView(DrawLayerComponent),
            .textureView = reg.view(.{ TextureComponent, DrawLayerComponent, RigidBody }, .{}),
            .topLayerView = reg.view(.{RigidBody}, .{DrawLayerComponent}),
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

    pub fn draw(self: *DrawSystem) void {
        if (rl.isKeyPressed(rl.KeyboardKey.key_q)) {
            showQTGrid = !showQTGrid;
        }
        if (rl.isKeyPressed(rl.KeyboardKey.key_c)) {
            showCollisionBoxes = !showCollisionBoxes;
        }

        if (drawLayersSlowly) {
            var maxLayer: usize = 0;

            for (self.layerView.raw()) |layer| {
                if (maxLayer < layer.z) {
                    maxLayer = layer.z;
                }
            }

            if (nextLayerDrawnAt <= rl.getTime()) {
                layersBeingDrawn += 1;

                if (layersBeingDrawn > maxLayer) {
                    layersBeingDrawn = 0;
                }

                nextLayerDrawnAt += layerDrawSpeed;
            }
        }

        for (self.drawOrder()) |entity| {
            if (drawLayersSlowly) {
                const layer = self.reg.get(DrawLayerComponent, entity);

                if (layer.z > layersBeingDrawn) {
                    continue;
                }
            }

            const body = self.bodyView.get(entity);
            const maybeTexture = self.reg.tryGetConst(TextureComponent, entity);

            if (maybeTexture) |texture| {
                self.drawTextureComponent(texture, body);
            }

            if (showCollisionBoxes) {
                self.drawShape(body);
            }
        }

        var it = self.topLayerView.entityIterator();

        while (it.next()) |entity| {
            const body = self.bodyView.get(entity);
            const maybeTexture = self.reg.tryGetConst(TextureComponent, entity);

            if (maybeTexture) |texture| {
                self.drawTextureComponent(texture, body);
            }

            if (showCollisionBoxes) {
                self.drawShape(body);
            }
        }

        if (showQTGrid) {
            self.drawCollisionContainer();
        }

        if (drawLayersSlowly) {
            const screen = self.reg.singletons().get(Screen);
            const pos = V.toRl(screen.sizeHalf);
            var buffer: [64:0]u8 = undefined;
            const label = std.fmt.bufPrintZ(&buffer, "Drawing Layer: {d:.0}", .{layersBeingDrawn}) catch unreachable;
            rl.drawTextPro(rl.getFontDefault(), label, pos, V.toRl(.{ 160, 0 }), 0, 48, 3, rl.Color.white);
        }
    }

    fn screenCoords(self: *DrawSystem, v: Vector) Vector {
        return self.screen.screenPositionV(self.camera.transformV(v));
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

    fn drawCollisionContainer(self: *DrawSystem) void {
        const cc = self.ccView.raw()[0];

        if (ccAlgorithm == .rTree) {
            self.drawPage(cc.tree.root, 1, cc.tree.height);
        } else {
            self.drawPageQT(cc.tree.getRoot(), 0);
        }
    }

    fn drawPageQT(self: *DrawSystem, maybePage: ?*CollisionContainer.QT.Page, level: usize) void {
        if (maybePage == null) return;
        const page = maybePage.?;

        const color = ccLevelColors[@min(level, ccLevelColors.len - 1)];

        self.drawPageQT(page.quadrants.tl, level + 1);
        self.drawPageQT(page.quadrants.tr, level + 1);
        self.drawPageQT(page.quadrants.bl, level + 1);
        self.drawPageQT(page.quadrants.br, level + 1);

        self.drawAabb(page.aabb, color, 2);
    }

    fn drawPage(self: *DrawSystem, page: *CollisionContainer.RTree.Page, level: usize, height: usize) void {
        std.debug.assert(level <= height);
        const color = ccLevelColors[@min(height - level, ccLevelColors.len - 1)];

        self.drawAabb(page.aabb, color, @as(f32, @floatFromInt(page.calculateHeight())) - 1);

        for (page.children.items) |child| {
            self.drawNode(child.*, level + 1, height);
        }
    }

    fn drawNode(self: *DrawSystem, node: CollisionContainer.RTree.Node, level: usize, height: usize) void {
        switch (node) {
            .page => |page| self.drawPage(page, level, height),
            .entry => |entry| self.drawEntry(entry.entry),
        }
    }

    fn drawEntry(self: *DrawSystem, entry: CollisionContainer.RTreeEntry) void {
        self.drawAabb(CollisionContainer.entryAabb(entry), ccEntryColor, 1);
    }

    pub fn drawAabb(self: *DrawSystem, aabb: AABB, color: rl.Color, thickness: f32) void {
        const p = self.screenCoords(aabb.tl);
        rl.drawRectangleLinesEx(
            rl.Rectangle.init(
                V.x(p),
                V.y(p),
                aabb.width(),
                aabb.height(),
            ),
            thickness,
            color,
        );
    }

    fn minYComparer(self: *DrawSystem, a: ecs.Entity, b: ecs.Entity) bool {
        const layerA = self.textureView.getConst(DrawLayerComponent, a);
        const layerB = self.textureView.getConst(DrawLayerComponent, b);

        return layerA.z < layerB.z;
    }

    fn ensureDrawOrderCapacity(self: *DrawSystem, reg: *ecs.Registry, entity: ecs.Entity) void {
        _ = entity;
        _ = reg;

        self.drawOrderList.clearRetainingCapacity();
        self.drawOrderList.ensureTotalCapacity(self.layerView.len()) catch unreachable;
        self.drawOrderList.expandToCapacity();
    }

    fn drawOrderFilterFn(self: *DrawSystem, entity: ecs.Entity) bool {
        const invisible = self.reg.tryGet(Invisible, entity);

        return invisible == null;
    }

    fn drawOrder(self: *DrawSystem) []const ecs.Entity {
        const filtered = util.filterTo(ecs.Entity, self.layerView.data(), self.drawOrderList.items, self, drawOrderFilterFn);
        std.mem.sort(ecs.Entity, filtered, self, minYComparer);

        return filtered;
    }

    pub fn drawTexture(
        self: DrawSystem,
        texture: *const rl.Texture2D,
        position: Vector,
        origin: Vector,
        rotation: f32,
        scale: f32,
        horizontalFlip: bool,
    ) void {
        const s = scale * self.camera.s;
        const r = rotation + self.camera.angle();
        const textureSize = V.fromInt(c_int, texture.width, texture.height);
        const p = position - textureSize * V.scalar(0.5 * scale);

        const screenP = self.camera.transformV(self.screen.screenPositionV(p));
        const screenS = textureSize * V.scalar(s);

        const sourceSize = if (horizontalFlip) textureSize * V.init(-1, 1) else textureSize;
        const source = V.rect(V.zero, sourceSize);
        const dest = V.rect(screenP, screenS);

        var origin_ = V.toRl(origin * V.scalar(scale));

        if (horizontalFlip) {
            origin_.x = -origin_.x;
        }

        rl.drawTexturePro(texture.*, source, dest, origin_, r, rl.Color.white);
    }

    pub fn drawTextureComponent(
        self: DrawSystem,
        texture: TextureComponent,
        body: *RigidBody,
    ) void {
        if (texture.sourceRectangle) |sourceRectangle| {
            self.drawTextureSR(
                texture.texture,
                sourceRectangle,
                texture.origin,
                body.d.clonePos(),
                body.d.r.*,
                body.d.s,
                texture.horizontalFlip,
            );
        } else {
            self.drawTexture(
                texture.texture,
                body.d.clonePos(),
                texture.origin,
                body.d.r.*,
                body.d.s,
                texture.horizontalFlip,
            );
        }
    }

    /// Draw texture with source rectangle
    pub fn drawTextureSR(
        self: DrawSystem,
        texture: *const rl.Texture2D,
        source: AABB,
        origin: Vector,
        position: Vector,
        rotation: f32,
        scale: f32,
        horizontalFlip: bool,
    ) void {
        const s = scale * self.camera.s;
        const r = rotation + self.camera.angle();
        const textureSize = source.size();
        const p = position - textureSize * V.scalar(0.5 * scale);

        const screenP = self.camera.transformV(self.screen.screenPositionV(p));
        const screenS = textureSize * V.scalar(s);

        const sourceSize = if (horizontalFlip) textureSize * V.init(-1, 1) else source.size();
        const sourceR = V.rect(source.tl, sourceSize);
        const dest = V.rect(screenP, screenS);

        var origin_ = V.toRl(origin * V.scalar(scale));

        if (horizontalFlip) {
            origin_.x = -origin_.x;
        }

        rl.drawTexturePro(texture.*, sourceR, dest, origin_, r, rl.Color.white);
    }

    pub fn drawShape(self: *DrawSystem, rb: *RigidBody) void {
        switch (rb.s.shape) {
            .circle => |circle| self.drawCircle(circle, rb),
            .rectangle => |rectangle| self.drawRectangle(rectangle, rb),
        }
    }

    pub fn drawCircle(self: *DrawSystem, circle: Circle, rb: *RigidBody) void {
        const s = self.camera.s;
        const center = rb.aabb.center();
        const screenP = self.screenCoords(center);

        rl.drawCircleLinesV(V.toRl(screenP), circle.radius * s, rl.Color.white);
    }

    pub fn drawRectangle(self: *DrawSystem, _: Rectangle, rb: *RigidBody) void {
        var transformedVertices: [5]rl.Vector2 = .{
            V.toRl(self.screenCoords(rb.aabb.tl)),
            V.toRl(self.screenCoords(rb.aabb.tr())),
            V.toRl(self.screenCoords(rb.aabb.br)),
            V.toRl(self.screenCoords(rb.aabb.bl())),
            V.toRl(self.screenCoords(rb.aabb.tl)),
        };

        const color = if (ccAlgorithm == .quadTree)
            if (self.ccView.raw()[0].tree.isEntryInTree(rb)) rl.Color.white else rl.Color.red
        else
            rl.Color.white;

        rl.drawLineStrip(&transformedVertices, color);
    }

    fn drawDebugBorder(self: DrawSystem) void {
        const lw = 4;
        const lo = V.all(lw / 2);

        const tl = V.toRl(self.screenCoords(-lo));
        const br = V.toRl(self.screenCoords(self.screen.size + lo));

        const bl = rl.Vector2.init(V.x(tl), V.y(br));
        const tr = rl.Vector2.init(V.x(br), V.y(tl));

        rl.drawLineEx(tl, bl, lw, rl.Color.red);
        rl.drawLineEx(tl, tr, lw, rl.Color.red);
        rl.drawLineEx(bl, br, lw, rl.Color.red);
        rl.drawLineEx(tr, br, lw, rl.Color.red);
    }
};
