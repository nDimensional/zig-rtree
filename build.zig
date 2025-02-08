const std = @import("std");

pub fn build(b: *std.Build) !void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const rtree = b.addModule("rtree", .{
        .target = target,
        .optimize = optimize,
        .root_source_file = b.path("src/RTree.zig"),
    });

    _ = rtree;
}
