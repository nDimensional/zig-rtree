const std = @import("std");
const utils = @import("utils.zig");

pub const Force = @import("Force.zig");

pub const Options = struct {
    /// threshold for large-body approximation
    threshold: f32 = 0.5,
    force: Force = .{},
};

pub fn RTree(comptime R: u3) type {
    return struct {
        const Self = @This();

        pub const Error = std.mem.Allocator.Error || error{ Empty, OutOfBounds };

        pub const fanout = 1 << R;
        pub const N = @Type(.{ .Int = .{ .signedness = .unsigned, .bits = R } });

        comptime {
            std.debug.assert(fanout == std.math.maxInt(N) + 1);
        }

        pub const Vector = @Vector(R, f32);
        pub const Point = @Vector(R, f32);

        pub const Cell = @Vector(R, bool);

        pub fn cellFromIndex(n: N) Cell {
            var q: Cell = @splat(false);
            inline for (0..R) |i| {
                const needle = (1 << (R - 1 - i));
                const bit = n & needle;
                q[i] = bit != 0;
            }

            return q;
        }

        pub fn indexFromCell(q: Cell) N {
            var n: N = 0;
            inline for (0..R) |i| {
                if (q[i]) {
                    const needle = @as(N, 1) << (R - 1 - i);
                    n |= needle;
                }
            }

            return n;
        }

        pub const Area = packed struct {
            s: f32 = 0,
            c: Point = @splat(0),

            pub fn locate(area: Area, point: Point) Cell {
                return area.c <= point;
            }

            pub fn divide(area: Area, quadrant: Cell) Area {
                const s = area.s / 2;
                const d = s / 2;

                var delta: Vector = @splat(0);
                inline for (0..R) |i|
                    delta[i] = if (quadrant[i]) d else -d;

                return .{ .s = s, .c = area.c + delta };
            }

            pub fn contains(area: Area, point: Point) bool {
                const s: Vector = @splat(area.s / 2);
                const min = area.c - s;
                const max = area.c + s;
                return @reduce(.And, min <= point) and @reduce(.And, point <= max);
            }
        };

        pub const Body = packed struct {
            position: Point = @splat(0),
            mass: f32 = 0,
        };

        pub const Node = struct {
            pub const NULL = std.math.maxInt(u32);

            body: Body,
            children: [fanout]u32 = .{NULL} ** fanout,

            pub inline fn isEmpty(node: Node) bool {
                inline for (node.children) |child| {
                    if (child != NULL) {
                        return false;
                    }
                }

                return true;
            }

            pub inline fn getQuadrant(node: Node, quadrant: Cell) u32 {
                const i = indexFromCell(quadrant);
                return node.children[i];
            }

            pub inline fn setQuadrant(node: *Node, quadrant: Cell, child: u32) void {
                const i = indexFromCell(quadrant);
                node.children[i] = child;
            }

            pub fn add(node: *Node, body: *const Body) void {
                const node_mass: Vector = @splat(node.body.mass);
                const body_mass: Vector = @splat(body.mass);
                const total_mass = node_mass + body_mass;

                node.body.position = node.body.position * node_mass + body.position * body_mass;
                node.body.position /= total_mass;
                node.body.mass += body.mass;
            }

            pub fn remove(node: *Node, body: *const Body) void {
                const total_mass: Vector = @splat(node.body.mass);
                const body_mass: Vector = @splat(body.mass);
                const node_mass = total_mass - body_mass;

                node.body.position = node.body.position * node_mass - body.position * body_mass;
                node.body.position /= total_mass;
                node.body.mass -= body.mass;
            }
        };

        area: Area,
        tree: std.ArrayList(Node),
        force: Force,
        threshold: f32,

        pub fn init(allocator: std.mem.Allocator, area: Area, options: Options) Self {
            return .{
                .area = area,
                .tree = std.ArrayList(Node).init(allocator),
                .threshold = options.threshold,
                .force = options.force,
            };
        }

        pub fn deinit(self: Self) void {
            self.tree.deinit();
        }

        pub fn reset(self: *Self, area: Area) void {
            self.area = area;
            self.tree.clearRetainingCapacity();
        }

        pub fn insert(self: *Self, position: Point, mass: f32) Error!void {
            if (!self.area.contains(position))
                return Error.OutOfBounds;

            const body = Body{ .position = position, .mass = mass };
            if (self.tree.items.len == 0) {
                try self.tree.append(Node{ .body = body });
            } else {
                try self.insertNode(0, self.area, &body);
            }
        }

        fn insertNode(self: *Self, id: u32, area: Area, body: *const Body) Error!void {
            std.debug.assert(id < self.tree.items.len);
            std.debug.assert(area.s > 0);

            if (self.tree.items[id].isEmpty()) {
                const node = self.tree.items[id];

                const index: u32 = @intCast(self.tree.items.len);
                try self.tree.append(node);

                self.tree.items[id].setQuadrant(area.locate(node.body.position), index);
            }

            self.tree.items[id].add(body);

            const quadrant = area.locate(body.position);
            const child = self.tree.items[id].getQuadrant(quadrant);

            if (child != Node.NULL) {
                const center = self.tree.items[child].body.position;
                if (@reduce(.And, center == body.position)) {
                    self.tree.items[child].body.mass += body.mass;
                    return;
                }

                try self.insertNode(child, area.divide(quadrant), body);
            } else {
                const index: u32 = @intCast(self.tree.items.len);
                try self.tree.append(.{ .body = body.* });
                self.tree.items[id].setQuadrant(quadrant, index);
            }
        }

        pub fn remove(self: *Self, position: Point, mass: f32) Error!void {
            if (self.tree.items.len == 0)
                return Error.Empty;

            if (!self.area.contains(position))
                return Error.OutOfBounds;

            const body = Body{ .position = position, .mass = mass };
            const remove_root = try self.removeNode(0, self.area, &body);
            if (remove_root)
                self.tree.clearRetainingCapacity();
        }

        fn removeNode(self: *Self, id: u32, area: Area, body: *const Body) Error!bool {
            std.debug.assert(area.s > 0);
            std.debug.assert(id < self.tree.items.len);

            if (self.tree.items[id].isEmpty()) {
                self.tree.items[id].body.mass -= body.mass;
                std.debug.assert(@abs(self.tree.items[id].body.mass) < utils.epsilon);
                return true;
            }

            const quadrant = area.locate(body.position);
            const child = self.tree.items[id].getQuadrant(quadrant);
            const remove_child = try self.removeNode(child, area.divide(quadrant), body);
            if (remove_child)
                self.tree.items[id].setQuadrant(quadrant, Node.NULL);

            self.tree.items[id].remove(body);
            return self.tree.items[id].isEmpty();
        }

        pub fn getTotalMass(self: Self) f32 {
            if (self.tree.items.len == 0) {
                return 0;
            } else {
                return self.tree.items[0].body.mass;
            }
        }

        pub inline fn setThreshold(self: Self, threshold: f32) void {
            self.threshold = threshold;
        }

        pub inline fn setForceParams(self: Self, params: Force.Params) void {
            self.force = Force.create(params);
        }

        pub fn getForce(self: Self, position: Point, mass: f32) Vector {
            if (self.tree.items.len == 0)
                return @as(Vector, @splat(0));

            const body = Body{ .position = position, .mass = mass };
            return self.getForceNode(0, self.area.s, &body);
        }

        fn getForceNode(self: Self, id: u32, s: f32, body: *const Body) Vector {
            if (id >= self.tree.items.len)
                @panic("index out of range");

            const node = self.tree.items[id];
            if (node.isEmpty())
                return self.force.getForce(R, body.position, body.mass, node.body.position, node.body.mass);

            const d = utils.getNorm(R, node.body.position - body.position);
            if (s / d < self.threshold)
                return self.force.getForce(R, body.position, body.mass, node.body.position, node.body.mass);

            var f: Vector = @splat(0);
            inline for (node.children) |child| {
                if (child != Node.NULL) {
                    f += self.getForceNode(child, s / 2, body);
                }
            }

            return f;
        }

        pub fn print(self: *Self, log: std.fs.File.Writer) !void {
            try self.printNode(log, 0, 1);
        }

        fn printNode(self: *Self, log: std.fs.File.Writer, id: u32, depth: usize) !void {
            std.debug.assert(id < self.tree.items.len);

            const node = self.tree.items[id];
            if (node.isEmpty()) {
                try log.print("leaf {d} (mass {d})\n", .{ id, node.body.mass });
            } else {
                try log.print("node {d} (mass {d})\n", .{ id, node.body.mass });
                inline for (0..fanout) |i| {
                    try log.writeByteNTimes(' ', depth * (R + 2));

                    // try log.print("{b:0>2}: ", .{i});
                    const width = comptime getWidthFormat();
                    try log.print(width ++ ": ", .{i});

                    if (node.children[i] == Node.NULL) {
                        try log.print("leaf {d} (mass {d})\n", .{ id, node.body.mass });
                    } else {
                        try self.printNode(log, node.children[i], depth + 1);
                    }
                }
            }
        }

        fn getWidthFormat() []const u8 {
            var width: [1]u8 = undefined;
            _ = std.fmt.formatIntBuf(&width, R, 10, .lower, .{});
            return "{b:0>" ++ width ++ "}";
        }
    };
}

test "cellFromIndex / indexFromCell" {
    const Tree = RTree(2);

    inline for (0..Tree.fanout) |i| {
        const cell = Tree.cellFromIndex(i);
        try std.testing.expectEqual(i, Tree.indexFromCell(cell));
    }
}
