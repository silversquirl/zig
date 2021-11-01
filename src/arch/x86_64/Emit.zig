//! This file contains the functionality for lowering x86_64 MIR into
//! machine code

const Emit = @This();
const std = @import("std");
const log = std.log.scoped(.codegen);
const math = std.math;
const mem = std.mem;
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
        switch (tag) {
            .push => try emit.mirPush(inst),
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

fn mirPush(emit: *Emit, inst: Mir.Inst.Index) !void {
    const tag = emit.mir.instructions.items(.tag)[inst];
    const ops = emit.mir.instructions.items(.ops)[inst];
    assert(tag == .push);
    const flag = @truncate(u1, ops);
    const reg = @intToEnum(Register, @truncate(u7, ops >> 9));
    if (flag == 0b0) {
        // PUSH reg
        try emit.code.append(0x50 | @intCast(u8, reg.lowId()));
    } else {
        // PUSH r/m64
        const imm = emit.mir.instructions.items(.data)[inst].imm;
        const mod: u2 = if (@truncate(i8, imm) == imm) 0b01 else 0b10;
        const mod_rm = (@intCast(u8, mod) << 6) | (@intCast(u8, Register.rsi.lowId()) << 3) | reg.lowId();
        try emit.code.ensureUnusedCapacity(2);
        emit.code.appendSliceAssumeCapacity(&.{ 0xff, mod_rm });
        if (mod == 0b01) {
            mem.writeIntLittle(i8, try emit.code.addOne(), @intCast(i8, imm));
        } else {
            mem.writeIntLittle(i32, try emit.code.addManyAsArray(4), imm);
        }
    }
}
