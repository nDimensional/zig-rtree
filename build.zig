const std = @import("std");

fn copy(comptime T: type, b: *std.Build, v: []const T) []const T {
    const c = b.allocator.alloc(u8, v.len) catch @panic("OOM");
    @memcpy(c, v);
    return c;
}

pub fn build(b: *std.Build) !void {
    var includes = std.ArrayList([]const u8).init(b.allocator);
    if (b.option([]const u8, "include", "Absolute path to Python include directory")) |include| {
        includes.append(include) catch @panic("OOM");
    } else {
        const result = std.process.Child.run(.{
            .allocator = b.allocator,
            .argv = &.{ "python3-config", "--includes" },
        }) catch @panic("failed to run python3-config --includes");
        defer b.allocator.free(result.stderr);
        defer b.allocator.free(result.stdout);

        switch (result.term) {
            .Exited => |status| if (status != 0) @panic("failed to run python3-config --includes"),
            else => @panic("failed to run python3-config --includes"),
        }

        var iter = std.mem.splitScalar(u8, result.stdout, ' ');
        while (iter.next()) |include| {
            var i = include;
            if (std.mem.startsWith(u8, i, "-I"))
                i = i[2..];
            if (std.mem.endsWith(u8, include, "\n"))
                i = i[0 .. i.len - 1];

            includes.append(copy(u8, b, i)) catch @panic("OOM");
        }
    }

    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    _ = b.addModule("rtree", .{
        .target = target,
        .optimize = optimize,
        .root_source_file = b.path("src/RTree.zig"),
    });

    _ = b.addModule("quadtree", .{
        .target = target,
        .optimize = optimize,
        .root_source_file = b.path("src/Quadtree.zig"),
    });

    addTarget(b, "x86_64-linux/libquadtree.so", .{ .cpu_arch = .x86_64, .os_tag = .linux }, optimize);
    addTarget(b, "aarch64-linux/libquadtree.so", .{ .cpu_arch = .aarch64, .os_tag = .linux }, optimize);
    addTarget(b, "x86_64-macos/libquadtree.dylib", .{ .cpu_arch = .x86_64, .os_tag = .macos }, optimize);
    addTarget(b, "aarch64-macos/libquadtree.dylib", .{ .cpu_arch = .aarch64, .os_tag = .macos }, optimize);

    // Tests
    const tests = b.addTest(.{ .root_source_file = b.path("src/test.zig") });
    const run_tests = b.addRunArtifact(tests);
    b.step("test", "Run tests").dependOn(&run_tests.step);
}

fn addTarget(b: *std.Build, name: []const u8, target: std.Target.Query, optimize: std.builtin.OptimizeMode) void {
    const lib = b.addSharedLibrary(.{
        .name = "quadtree",
        .target = b.resolveTargetQuery(target),
        .optimize = optimize,
        .root_source_file = b.path("lib/quadtree.zig"),
    });

    const quadtree = b.modules.get("quadtree") orelse return;
    lib.root_module.addImport("quadtree", quadtree);
    lib.linker_allow_shlib_undefined = true;

    const artifact = b.addInstallArtifact(lib, .{ .dest_sub_path = name });
    b.getInstallStep().dependOn(&artifact.step);
}
