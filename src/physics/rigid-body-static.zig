const std = @import("std");
const zlm = @import("zlm");

const Shape = @import("shape.zig").Shape;
const AABB = @import("shape.zig").AABB;
const Result = @import("result.zig").Result;

/// 1cm x 1cm
pub const MIN_AREA: f32 = 0.01 * 0.01;
/// 1024m x 1024m
pub const MAX_AREA: f32 = 1024 * 1024;

/// Density of air = 0.001225 g/cm^3
pub const MIN_DENSITY: f32 = 0.001225;
/// Density of osmium = 22.6 g/cm^3
pub const MAX_DENSITY: f32 = 22.6;

pub const MIN_RESTITUTION: f32 = 0;
pub const MAX_RESTITUTION: f32 = 1;

pub const Densities = struct {
    pub const Wood = 0.5;
    pub const Water = 1;
    pub const Human = 1.1;

    pub const Element = struct {
        pub const Hydrogen = 0.0899;
        pub const Helium = 0.1785;
        pub const Neon = 0.9;
        pub const Nitrogen = 1.2506;
        pub const Oxygen = 1.429;
        pub const Fluorine = 1.696;
        pub const Argon = 1.7824;
        pub const Chlorine = 3.214;
        pub const Krypton = 3.75;
        pub const Xenon = 5.9;
        pub const Radon = 9.73;
        pub const Lithium = 0.534;
        pub const Potassium = 0.862;
        pub const Sodium = 0.971;
        pub const Calcium = 1.55;
        pub const Rubidium = 1.63;
        pub const Magnesium = 1.738;
        pub const Phosphorus = 1.82;
        pub const Beryllium = 1.848;
        pub const Cesium = 1.873;
        pub const Sulfur = 2.07;
        pub const Carbon = 2.26;
        pub const Silicon = 2.33;
        pub const Boron = 2.34;
        pub const Strontium = 2.54;
        pub const Aluminum = 2.702;
        pub const Scandium = 2.99;
        pub const Bromine = 3.119;
        pub const Barium = 3.59;
        pub const Yttrium = 4.47;
        pub const Titanium = 4.54;
        pub const Selenium = 4.79;
        pub const Iodine = 4.93;
        pub const Europium = 5.24;
        pub const Germanium = 5.323;
        pub const Radium = 5.5;
        pub const Arsenic = 5.72;
        pub const Gallium = 5.907;
        pub const Vanadium = 6.11;
        pub const Lanthanum = 6.15;
        pub const Tellurium = 6.24;
        pub const Zirconium = 6.51;
        pub const Antimony = 6.684;
        pub const Praseodymium = 6.77;
        pub const Cerium = 6.77;
        pub const Ytterbium = 6.9;
        pub const Neodymium = 7.01;
        pub const Zinc = 7.13;
        pub const Chromium = 7.19;
        pub const Promethium = 7.3;
        pub const Indium = 7.31;
        pub const Tin = 7.31;
        pub const Manganese = 7.43;
        pub const Samarium = 7.52;
        pub const Iron = 7.874;
        pub const Gadolinium = 7.895;
        pub const Terbium = 8.23;
        pub const Dysprosium = 8.55;
        pub const Niobium = 8.57;
        pub const Cadmium = 8.65;
        pub const Holmium = 8.8;
        pub const Cobalt = 8.9;
        pub const Nickel = 8.9;
        pub const Copper = 8.96;
        pub const Erbium = 9.07;
        pub const Polonium = 9.3;
        pub const Thulium = 9.32;
        pub const Bismuth = 9.75;
        pub const Lutetium = 9.84;
        pub const Actinium = 10.07;
        pub const Molybdenum = 10.22;
        pub const Silver = 10.5;
        pub const Lead = 11.35;
        pub const Technetium = 11.5;
        pub const Thorium = 11.724;
        pub const Thallium = 11.85;
        pub const Palladium = 12.02;
        pub const Ruthenium = 12.37;
        pub const Rhodium = 12.41;
        pub const Hafnium = 13.31;
        pub const Curium = 13.5;
        pub const Mercury = 13.546;
        pub const Americium = 13.67;
        pub const Berkelium = 14.78;
        pub const Californium = 15.1;
        pub const Protactinium = 15.4;
        pub const Tantalum = 16.65;
        pub const Uranium = 18.95;
        pub const Gold = 19.32;
        pub const Tungsten = 19.35;
        pub const Plutonium = 19.84;
        pub const Neptunium = 20.2;
        pub const Rhenium = 21.04;
        pub const Platinum = 21.45;
        pub const Iridium = 22.4;
        pub const Osmium = 22.6;
    };
};

pub const RigidBodyStaticParams = struct {
    density: f32,
    restitution: f32,

    isStatic: bool,

    shape: Shape,

    pub fn init(shape: Shape, density: f32, restitution: f32, isStatic: bool) Result(RigidBodyStaticParams) {
        const a = shape.area();

        if (a < MIN_AREA) {
            return .{ .err = std.fmt.comptimePrint("Area cannot be less than {d:.4}", .{MIN_AREA}) };
        } else if (a > MAX_AREA) {
            return .{ .err = std.fmt.comptimePrint("Area cannot be greater than {d:.2}", .{MAX_AREA}) };
        }

        if (density < MIN_DENSITY) {
            return .{ .err = std.fmt.comptimePrint("Density cannot be less than {d:.6}", .{MIN_DENSITY}) };
        } else if (density > MAX_DENSITY) {
            return .{ .err = std.fmt.comptimePrint("Density cannot be greater than {d:.1}", .{MAX_DENSITY}) };
        }

        if (restitution < MIN_RESTITUTION) {
            return .{ .err = std.fmt.comptimePrint("Restitution cannot be less than {d:.0}", .{MIN_RESTITUTION}) };
        } else if (restitution > MAX_RESTITUTION) {
            return .{ .err = std.fmt.comptimePrint("Restitution cannot be greater than {d:.0}", .{MAX_RESTITUTION}) };
        }

        return .{ .success = RigidBodyStaticParams{
            .density = density,
            .restitution = restitution,
            .isStatic = isStatic,
            .shape = shape,
        } };
    }

    pub fn aabb(self: RigidBodyStaticParams, rotation: f32) AABB {
        return self.shape.aabb(rotation);
    }

    pub fn radius(self: RigidBodyStaticParams) f32 {
        return self.shape.radius();
    }

    pub fn area(self: RigidBodyStaticParams) f32 {
        return self.shape.area();
    }

    pub fn volume(self: RigidBodyStaticParams) f32 {
        return self.shape.area();
    }

    pub fn mass(self: RigidBodyStaticParams) f32 {
        const v = self.volume();
        if (v == 0) return 0;
        const d = self.density;
        return d / v;
    }

    pub fn vertices(self: *const RigidBodyStaticParams) []const zlm.Vec2 {
        return self.shape.vertices();
    }

    pub fn transformedVertices(self: *const RigidBodyStaticParams) []const zlm.Vec2 {
        return self.shape.transformedVertices();
    }

    pub fn updateTransform(self: *RigidBodyStaticParams, translation: zlm.Vec2, rotation: f32, scale: f32) void {
        self.shape.updateTransform(translation, rotation, scale);
    }
};
