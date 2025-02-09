const std = @import("std");

const Quadtree = @import("Quadtree.zig").Quadtree;
const RTree = @import("RTree.zig").RTree;

fn getNorm(comptime R: u3, f: @Vector(R, f32)) f32 {
    return std.math.sqrt(@reduce(.Add, f * f));
}

fn Context(comptime R: u3) type {
    return struct {
        const Self = @This();

        repulsion: f32 = 100.0,

        pub fn getForce(
            self: Self,
            a_position: @Vector(R, f32),
            a_mass: f32,
            b_position: @Vector(R, f32),
            b_mass: f32,
        ) @Vector(R, f32) {
            const delta = b_position - a_position;

            const dist = getNorm(R, delta);
            if (dist == 0) {
                return @splat(0);
            }

            const unit = delta / @as(@Vector(R, f32), @splat(dist));

            var f: f32 = -1 * (self.repulsion) / dist;
            f *= a_mass * b_mass / 500;

            return unit * @as(@Vector(R, f32), @splat(f));
        }
    };
}

test "create RTree(3)" {
    const s: f32 = 8192;

    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer std.debug.assert(gpa.deinit() == .ok);

    const allocator = gpa.allocator();

    var prng = std.Random.Xoshiro256.init(0);
    const random = prng.random();

    const Tree = RTree(3);

    var rtree = Tree.init(allocator, .{ .s = s });
    defer rtree.deinit();

    var bodies = std.ArrayList(Tree.Body).init(allocator);
    defer bodies.deinit();

    var total_mass: f32 = 0;

    const count = 10000;
    for (0..count) |_| {
        const x = (random.float(f32) - 0.5) * s;
        const y = (random.float(f32) - 0.5) * s;
        const z = (random.float(f32) - 0.5) * s;

        const mass: f32 = @floatFromInt(random.uintLessThan(u32, 256));

        try rtree.insert(.{ x, y, z }, mass);
        try bodies.append(.{ .position = .{ x, y, z }, .mass = mass });

        total_mass += mass;
        try std.testing.expectEqual(rtree.tree.items[0].body.mass, total_mass);
    }

    for (bodies.items) |body| {
        try rtree.remove(body.position, body.mass);
        total_mass -= body.mass;
        try std.testing.expectEqual(rtree.getTotalMass(), total_mass);
    }
}

test "compare RTree(2) with Quadtree" {
    const s: f32 = 8192;

    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer std.debug.assert(gpa.deinit() == .ok);

    const allocator = gpa.allocator();

    var prng = std.Random.Xoshiro256.init(0);
    const random = prng.random();

    var quadtree = Quadtree.init(allocator, .{ .s = s });
    defer quadtree.deinit();

    const Tree = RTree(2);

    var rtree = Tree.init(allocator, .{ .s = s });
    defer rtree.deinit();

    var bodies = std.ArrayList(Tree.Body).init(allocator);
    defer bodies.deinit();

    var total_mass: f32 = 0;

    const count = 10000;
    for (0..count) |_| {
        const x = (random.float(f32) - 0.5) * s;
        const y = (random.float(f32) - 0.5) * s;
        const mass: f32 = @floatFromInt(random.uintLessThan(u32, 256));

        try quadtree.insert(.{ x, y }, mass);
        try rtree.insert(.{ x, y }, mass);
        try bodies.append(.{ .position = .{ x, y }, .mass = mass });

        total_mass += mass;
        try std.testing.expectEqual(quadtree.getTotalMass(), total_mass);
        try std.testing.expectEqual(rtree.getTotalMass(), total_mass);
    }

    const ctx = Context(2){};

    for (0..10000) |_| {
        const x = (random.float(f32) - 0.5) * s;
        const y = (random.float(f32) - 0.5) * s;
        const mass: f32 = @floatFromInt(1 + random.uintLessThan(u32, 256));

        try std.testing.expectEqual(
            quadtree.getForce(ctx, .{ x, y }, mass),
            rtree.getForce(ctx, .{ x, y }, mass),
        );
    }

    for (bodies.items) |body| {
        try quadtree.remove(body.position, body.mass);
        try rtree.remove(body.position, body.mass);
        total_mass -= body.mass;
        try std.testing.expectEqual(quadtree.getTotalMass(), total_mass);
        try std.testing.expectEqual(rtree.getTotalMass(), total_mass);
    }
}

test "compare RTree(3) with Quadtree" {
    const s: f32 = 8192;

    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer std.debug.assert(gpa.deinit() == .ok);

    const allocator = gpa.allocator();

    var prng = std.Random.Xoshiro256.init(0);
    const random = prng.random();

    var quadtree = Quadtree.init(allocator, .{ .s = s });
    defer quadtree.deinit();

    const Tree = RTree(3);

    var rtree = Tree.init(allocator, .{ .s = s });
    defer rtree.deinit();

    var bodies = std.ArrayList(@Vector(3, f32)).init(allocator);
    defer bodies.deinit();

    var total_mass: f32 = 0;

    const count = 10000;
    for (0..count) |_| {
        const x = (random.float(f32) - 0.5) * s;
        const y = (random.float(f32) - 0.5) * s;
        const mass: f32 = @floatFromInt(random.uintLessThan(u32, 256));

        try quadtree.insert(.{ x, y }, mass);
        try rtree.insert(.{ x, y, 0 }, mass);
        try bodies.append(.{ x, y, mass });

        total_mass += mass;
        try std.testing.expectEqual(quadtree.getTotalMass(), total_mass);
        try std.testing.expectEqual(rtree.getTotalMass(), total_mass);
    }

    const ctx2 = Context(2){};
    const ctx3 = Context(3){};

    for (0..10000) |_| {
        const x = (random.float(f32) - 0.5) * s;
        const y = (random.float(f32) - 0.5) * s;
        const mass: f32 = @floatFromInt(1 + random.uintLessThan(u32, 256));

        const a = quadtree.getForce(ctx2, .{ x, y }, mass);
        const b = rtree.getForce(ctx3, .{ x, y, 0 }, mass);
        try std.testing.expectEqual(a[0], b[0]);
        try std.testing.expectEqual(a[1], b[1]);
        try std.testing.expectEqual(b[2], 0);
    }

    for (bodies.items) |body| {
        try quadtree.remove(.{ body[0], body[1] }, body[2]);
        try rtree.remove(.{ body[0], body[1], 0 }, body[2]);
        total_mass -= body[2];
        try std.testing.expectEqual(quadtree.getTotalMass(), total_mass);
        try std.testing.expectEqual(rtree.getTotalMass(), total_mass);
    }
}
