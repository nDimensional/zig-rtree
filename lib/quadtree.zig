const std = @import("std");

const qt = @import("quadtree");

var gpa = std.heap.GeneralPurposeAllocator(.{}){};
const allocator = gpa.allocator();

/// Initialize a new quadtree with given center coordinates and side length
export fn quadtree_init(side_len: f32, center_x: f32, center_y: f32) callconv(.C) ?*qt.Quadtree {
    const area = qt.Area{ .c = .{ center_x, center_y }, .s = side_len };

    const tree = allocator.create(qt.Quadtree) catch return null;
    tree.* = qt.Quadtree.init(std.heap.page_allocator, area, .{});
    return tree;
}

/// Deallocate and destroy the quadtree
export fn quadtree_deinit(tree: *qt.Quadtree) callconv(.C) void {
    tree.deinit();
    allocator.destroy(tree);
}

/// Reset the quadtree with new parameters
export fn quadtree_reset(tree: *qt.Quadtree, side_len: f32, center_x: f32, center_y: f32) callconv(.C) void {
    const area = qt.Area{ .c = .{ center_x, center_y }, .s = side_len };
    tree.reset(area);
}

/// Set the threshold for large body approximation
export fn quadtree_set_threshold(tree: *qt.Quadtree, threshold: f32) callconv(.C) void {
    tree.setThreshold(threshold);
}

/// Set the force equation parameters
export fn quadtree_set_force_params(tree: *qt.Quadtree, c: f32, r: f32) callconv(.C) void {
    tree.setForceParams(.{ .c = c, .r = r });
}

/// Insert a point with mass into the quadtree
/// Returns 0 on success, -1 on failure
export fn quadtree_insert(tree: *qt.Quadtree, position_x: f32, position_y: f32, mass: f32) callconv(.C) c_int {
    tree.insert(.{ position_x, position_y }, mass) catch return -1;
    return 0;
}

/// Remove a point with mass from the quadtree
/// Returns 0 on success, -1 on failure
export fn quadtree_remove(tree: *qt.Quadtree, position_x: f32, position_y: f32, mass: f32) callconv(.C) c_int {
    tree.remove(.{ position_x, position_y }, mass) catch return -1;
    return 0;
}

/// Get the total mass stored in the quadtree
export fn quadtree_get_total_mass(tree: *qt.Quadtree) callconv(.C) f32 {
    return tree.getTotalMass();
}

/// Get the force exerted on a body
export fn quadtree_get_force(tree: *qt.Quadtree, position_x: f32, position_y: f32, mass: f32, result_x: *f32, result_y: *f32) callconv(.C) void {
    const f = tree.getForce(.{ position_x, position_y }, mass);
    result_x.* = f[0];
    result_y.* = f[1];
}

/// Get the nearest body (inclusive)
export fn quadtree_get_nearest_body_inclusive(tree: *qt.Quadtree, position_x: f32, position_y: f32, result_x: *f32, result_y: *f32, result_mass: *f32) callconv(.C) void {
    const body = tree.getNearestBody(.{ position_x, position_y }, .inclusive) catch {
        result_x.* = std.math.nan(f32);
        result_y.* = std.math.nan(f32);
        result_mass.* = std.math.nan(f32);
        return;
    };

    result_x.* = body.position[0];
    result_y.* = body.position[1];
    result_mass.* = body.mass;
}

/// Get the nearest body (exclusive)
export fn quadtree_get_nearest_body_exclusive(tree: *qt.Quadtree, position_x: f32, position_y: f32, result_x: *f32, result_y: *f32, result_mass: *f32) callconv(.C) void {
    const body = tree.getNearestBody(.{ position_x, position_y }, .exclusive) catch {
        result_x.* = std.math.nan(f32);
        result_y.* = std.math.nan(f32);
        result_mass.* = std.math.nan(f32);
        return;
    };

    result_x.* = body.position[0];
    result_y.* = body.position[1];
    result_mass.* = body.mass;
}
