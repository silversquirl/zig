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
const ErrorMsg = Module.ErrorMsg;
const Module = @import("../../Module.zig");
const assert = std.debug.assert;
const DW = std.dwarf;
const leb128 = std.leb;
const Instruction = bits.Instruction;
const Register = bits.Register;
const DebugInfoOutput = @import("../../codegen.zig").DebugInfoOutput;
const Encoder = bits.Encoder;

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

pub fn emitMir(emit: *Emit) InnerError!void {
    const mir_tags = emit.mir.instructions.items(.tag);

    for (mir_tags) |tag, index| {
        const inst = @intCast(u32, index);
        switch (tag) {
            .mov => try emit.mirMov(inst),
            .sub => try emit.mirSub(inst),
            .push => try emit.mirPush(inst),
            .ret => try emit.mirRet(inst),
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

fn mirPush(emit: *Emit, inst: Mir.Inst.Index) InnerError!void {
    const tag = emit.mir.instructions.items(.tag)[inst];
    const ops = Mir.Ops.decode(emit.mir.instructions.items(.ops)[inst]);
    assert(tag == .push);
    const encoder = try Encoder.init(emit.code, 6);
    if (@truncate(u1, ops.flags) == 0b0) {
        // PUSH reg
        encoder.opcode_1byte(0x50 | @intCast(u8, ops.reg1.lowId()));
    } else {
        // PUSH r/m64
        const imm = emit.mir.instructions.items(.data)[inst].imm;
        encoder.opcode_1byte(0xff);
        if (math.cast(i8, imm)) |imm_i8| {
            encoder.modRm_indirectDisp8(Register.rsi.lowId(), ops.reg1.lowId());
            encoder.imm8(@intCast(i8, imm_i8));
        } else |_| {
            encoder.modRm_indirectDisp32(Register.rsi.lowId(), ops.reg1.lowId());
            encoder.imm32(imm);
        }
    }
}

fn mirRet(emit: *Emit, inst: Mir.Inst.Index) InnerError!void {
    const tag = emit.mir.instructions.items(.tag)[inst];
    const ops = Mir.Ops.decode(emit.mir.instructions.items(.ops)[inst]);
    assert(tag == .ret);
    const encoder = try Encoder.init(emit.code, 3);
    switch (ops.flags) {
        0b00 => {
            // RETF imm16
            const imm = emit.mir.instructions.items(.data)[inst].imm;
            encoder.opcode_1byte(0xca);
            encoder.imm16(@intCast(i16, imm));
        },
        0b01 => encoder.opcode_1byte(0xcb), // RETF
        0b10 => {
            // RET imm16
            const imm = emit.mir.instructions.items(.data)[inst].imm;
            encoder.opcode_1byte(0xc2);
            encoder.imm16(@intCast(i16, imm));
        },
        0b11 => encoder.opcode_1byte(0xc3), // RET
    }
}

fn mirMov(emit: *Emit, inst: Mir.Inst.Index) InnerError!void {
    const tag = emit.mir.instructions.items(.tag)[inst];
    const ops = Mir.Ops.decode(emit.mir.instructions.items(.ops)[inst]);
    assert(tag == .mov);
    const encoder = try Encoder.init(emit.code, 15); // TODO reduce from max inst size
    switch (ops.flags) {
        0b00 => blk: {
            if (ops.reg2 == .none) {
                const imm = emit.mir.instructions.items(.data)[inst].imm;
                // MOV r/m, imm
                encoder.rex(.{
                    .w = ops.reg1.size() == 64,
                    .r = ops.reg1.isExtended(),
                });
                encoder.opcode_1byte(0xc7);
                encoder.modRm_direct(0x0, ops.reg1.lowId());
                encoder.imm32(imm);
                break :blk;
            }

            // MOV r/m, r
            encoder.rex(.{
                .w = ops.reg1.size() == 64,
                .r = ops.reg2.isExtended(),
                .b = ops.reg1.isExtended(),
            });
            encoder.opcode_1byte(0x89);
            encoder.modRm_direct(ops.reg2.lowId(), ops.reg1.lowId());
        },
        else => return emit.fail("TODO more MOV alternatives", .{}),
    }
}

fn mirSub(emit: *Emit, inst: Mir.Inst.Index) InnerError!void {
    const tag = emit.mir.instructions.items(.tag)[inst];
    const ops = Mir.Ops.decode(emit.mir.instructions.items(.ops)[inst]);
    assert(tag == .sub);
    const encoder = try Encoder.init(emit.code, 15); // TODO reduce from max inst size
    switch (ops.flags) {
        0b00 => blk: {
            if (ops.reg2 == .none) {
                const imm = emit.mir.instructions.items(.data)[inst].imm;
                // SUB r/m, imm
                encoder.rex(.{
                    .w = ops.reg1.size() == 64,
                    .r = ops.reg1.isExtended(),
                });
                encoder.opcode_1byte(0x81);
                encoder.modRm_direct(0x5, ops.reg1.lowId());
                encoder.imm32(imm);
                break :blk;
            }

            // SUB r/m, r
            encoder.rex(.{
                .w = ops.reg1.size() == 64,
                .r = ops.reg2.isExtended(),
                .b = ops.reg1.isExtended(),
            });
            encoder.opcode_1byte(0x29); // TODO SUB r/m8, r8, different opcode 0x28
            encoder.modRm_direct(ops.reg2.lowId(), ops.reg1.lowId());
        },
        else => return emit.fail("TODO more SUB alternatives", .{}),
    }
}
