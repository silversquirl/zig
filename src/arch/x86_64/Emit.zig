//! This file contains the functionality for lowering x86_64 MIR into
//! machine code

const Emit = @This();
const std = @import("std");
const math = std.math;
const Mir = @import("Mir.zig");
const bits = @import("bits.zig");
const link = @import("../../link.zig");
const Module = @import("../../Module.zig");
const ErrorMsg = Module.ErrorMsg;
const assert = std.debug.assert;
const DW = std.dwarf;
const leb128 = std.leb;
const Instruction = bits.Instruction;
const Register = bits.Register;
const DebugInfoOutput = @import("../../codegen.zig").DebugInfoOutput;

mir: Mir,
bin_file: *link.File,
debug_output: DebugInfoOutput,
target: *const std.Target,
err_msg: ?*ErrorMsg = null,
src_loc: Module.SrcLoc,
code: *std.ArrayList(u8),

prev_di_line: u32,
prev_di_column: u32,
/// Relative to the beginning of `code`.
prev_di_pc: usize,

const InnerError = error{
    OutOfMemory,
    EmitFail,
};

pub fn emitMir(emit: *Emit) !void {
    const mir_tags = emit.mir.instructions.items(.tag);

    for (mir_tags) |tag, index| {
        const inst = @intCast(u32, index);
        _ = inst;
        switch (tag) {
            else => {
                return emit.fail("Implement MIR->Isel lowering for x86_64 for pseudo-inst: {s}", .{tag});
            },
        }
    }
}

fn fail(emit: *Emit, comptime format: []const u8, args: anytype) InnerError {
    @setCold(true);
    assert(emit.err_msg == null);
    emit.err_msg = try ErrorMsg.create(emit.bin_file.allocator, emit.src_loc, format, args);
    return error.EmitFail;
}
