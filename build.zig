const std = @import("std");

pub fn build(b: *std.Build) !void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    _ = b.addModule("rtree", .{
        .target = target,
        .optimize = optimize,
        .root_source_file = b.path("src/RTree.zig"),
    });

    // Tests
    const tests = b.addTest(.{ .root_source_file = b.path("src/test.zig") });
    const run_tests = b.addRunArtifact(tests);
    b.step("test", "Run tests").dependOn(&run_tests.step);
}
