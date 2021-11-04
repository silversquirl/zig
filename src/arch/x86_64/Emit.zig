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
            .adc => try emit.mirArith(.adc, inst),
            .add => try emit.mirArith(.add, inst),
            .sub => try emit.mirArith(.sub, inst),
            .xor => try emit.mirArith(.xor, inst),
            .@"and" => try emit.mirArith(.@"and", inst),
            .@"or" => try emit.mirArith(.@"or", inst),
            .sbb => try emit.mirArith(.sbb, inst),
            .cmp => try emit.mirArith(.cmp, inst),

            .adc_scale_src => try emit.mirArithScaleSrc(.adc, inst),
            .add_scale_src => try emit.mirArithScaleSrc(.add, inst),
            .sub_scale_src => try emit.mirArithScaleSrc(.sub, inst),
            .xor_scale_src => try emit.mirArithScaleSrc(.xor, inst),
            .and_scale_src => try emit.mirArithScaleSrc(.@"and", inst),
            .or_scale_src => try emit.mirArithScaleSrc(.@"or", inst),
            .sbb_scale_src => try emit.mirArithScaleSrc(.sbb, inst),
            .cmp_scale_src => try emit.mirArithScaleSrc(.cmp, inst),

            .adc_scale_dst => try emit.mirArithScaleDst(.adc, inst),
            .add_scale_dst => try emit.mirArithScaleDst(.add, inst),
            .sub_scale_dst => try emit.mirArithScaleDst(.sub, inst),
            .xor_scale_dst => try emit.mirArithScaleDst(.xor, inst),
            .and_scale_dst => try emit.mirArithScaleDst(.@"and", inst),
            .or_scale_dst => try emit.mirArithScaleDst(.@"or", inst),
            .sbb_scale_dst => try emit.mirArithScaleDst(.sbb, inst),
            .cmp_scale_dst => try emit.mirArithScaleDst(.cmp, inst),

            .adc_scale_imm => try emit.mirArithScaleImm(.adc, inst),
            .add_scale_imm => try emit.mirArithScaleImm(.add, inst),
            .sub_scale_imm => try emit.mirArithScaleImm(.sub, inst),
            .xor_scale_imm => try emit.mirArithScaleImm(.xor, inst),
            .and_scale_imm => try emit.mirArithScaleImm(.@"and", inst),
            .or_scale_imm => try emit.mirArithScaleImm(.@"or", inst),
            .sbb_scale_imm => try emit.mirArithScaleImm(.sbb, inst),
            .cmp_scale_imm => try emit.mirArithScaleImm(.cmp, inst),

            // Even though MOV is technically not an arithmetic op,
            // its structure can be represented using the same set of
            // opcode primitives.
            .mov => try emit.mirArith(.mov, inst),
            .mov_scale_src => try emit.mirArithScaleSrc(.mov, inst),
            .mov_scale_dst => try emit.mirArithScaleDst(.mov, inst),
            .mov_scale_imm => try emit.mirArithScaleImm(.mov, inst),
            .movabs => try emit.mirMovabs(inst),

            .lea => try emit.mirLea(inst),

            .push => try emit.mirPushPop(.push, inst),
            .pop => try emit.mirPushPop(.pop, inst),

            .jmp => try emit.mirJmpCall(.jmp, inst),
            .call => try emit.mirJmpCall(.call, inst),

            .ret => try emit.mirRet(inst),

            .syscall => try emit.mirSyscall(),
            .brk => try emit.mirBrk(),

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

fn mirBrk(emit: *Emit) InnerError!void {
    const encoder = try Encoder.init(emit.code, 1);
    encoder.opcode_1byte(0xcc);
}

fn mirSyscall(emit: *Emit) InnerError!void {
    const encoder = try Encoder.init(emit.code, 2);
    encoder.opcode_2byte(0x0f, 0x05);
}

fn mirPushPop(emit: *Emit, tag: Mir.Inst.Tag, inst: Mir.Inst.Index) InnerError!void {
    const ops = Mir.Ops.decode(emit.mir.instructions.items(.ops)[inst]);
    if (@truncate(u1, ops.flags) == 0b0) {
        // PUSH/POP reg
        const opc: u8 = switch (tag) {
            .push => 0x50,
            .pop => 0x58,
            else => unreachable,
        };
        const encoder = try Encoder.init(emit.code, 1);
        encoder.opcode_withReg(opc, ops.reg1.lowId());
    } else {
        // PUSH/POP r/m64
        const imm = emit.mir.instructions.items(.data)[inst].imm;
        const opc: u8 = switch (tag) {
            .push => 0xff,
            .pop => 0x8f,
            else => unreachable,
        };
        const modrm_ext: u3 = switch (tag) {
            .push => 0x6,
            .pop => 0x0,
            else => unreachable,
        };
        const encoder = try Encoder.init(emit.code, 6);
        encoder.opcode_1byte(opc);
        if (math.cast(i8, imm)) |imm_i8| {
            encoder.modRm_indirectDisp8(modrm_ext, ops.reg1.lowId());
            encoder.imm8(@intCast(i8, imm_i8));
        } else |_| {
            encoder.modRm_indirectDisp32(modrm_ext, ops.reg1.lowId());
            encoder.imm32(imm);
        }
    }
}

fn mirJmpCall(emit: *Emit, tag: Mir.Inst.Tag, inst: Mir.Inst.Index) InnerError!void {
    const ops = Mir.Ops.decode(emit.mir.instructions.items(.ops)[inst]);
    const flag = @truncate(u1, ops.flags);
    if (flag == 0) {
        // const imm = emit.mir.instructions.items(.data)[inst].imm;
        // const opc: u8 = switch (tag) {
        //     .jmp => if (imm <= math.maxInt(i8)) @as(u8, 0xeb) else 0xe9,
        //     .call => 0xe8,
        //     else => unreachable,
        // };
        // const encoder = try Encoder.init(emit.code, 5);
        // encoder.opcode_1byte(opc);
        // if (imm <= math.maxInt(i8)) {
        //     encoder.imm8(@intCast(i8, imm));
        // } else {
        //     encoder.imm32(imm);
        // }
        return emit.fail("TODO implement emitting JMP/CALL inst/rel", .{});
    }
    const modrm_ext: u3 = switch (tag) {
        .jmp => 0x4,
        .call => 0x2,
        else => unreachable,
    };
    if (ops.reg1 == .none) {
        // JMP/CALL [imm]
        const imm = emit.mir.instructions.items(.data)[inst].imm;
        const encoder = try Encoder.init(emit.code, 7);
        encoder.opcode_1byte(0xff);
        encoder.modRm_SIBDisp0(modrm_ext);
        encoder.sib_disp32();
        encoder.imm32(imm);
        return;
    }
    // JMP/CALL reg
    const encoder = try Encoder.init(emit.code, 2);
    encoder.opcode_1byte(0xff);
    encoder.modRm_direct(modrm_ext, ops.reg1.lowId());
}

fn mirRet(emit: *Emit, inst: Mir.Inst.Index) InnerError!void {
    const tag = emit.mir.instructions.items(.tag)[inst];
    assert(tag == .ret);
    const ops = Mir.Ops.decode(emit.mir.instructions.items(.ops)[inst]);
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

const EncType = enum {
    /// OP r/m64, imm32
    mi,

    /// OP r/m64, r64
    mr,

    /// OP r64, r/m64
    rm,
};

const OpCode = struct {
    opc: u8,
    /// Only used if `EncType == .mi`.
    modrm_ext: u3,
};

inline fn getArithOpCode(tag: Mir.Inst.Tag, enc: EncType) OpCode {
    switch (enc) {
        .mi => return switch (tag) {
            .adc => .{ .opc = 0x81, .modrm_ext = 0x2 },
            .add => .{ .opc = 0x81, .modrm_ext = 0x0 },
            .sub => .{ .opc = 0x81, .modrm_ext = 0x5 },
            .xor => .{ .opc = 0x81, .modrm_ext = 0x6 },
            .@"and" => .{ .opc = 0x81, .modrm_ext = 0x4 },
            .@"or" => .{ .opc = 0x81, .modrm_ext = 0x1 },
            .sbb => .{ .opc = 0x81, .modrm_ext = 0x3 },
            .cmp => .{ .opc = 0x81, .modrm_ext = 0x7 },
            .mov => .{ .opc = 0xc7, .modrm_ext = 0x0 },
            else => unreachable,
        },
        .mr => {
            const opc: u8 = switch (tag) {
                .adc => 0x11,
                .add => 0x01,
                .sub => 0x29,
                .xor => 0x31,
                .@"and" => 0x21,
                .@"or" => 0x09,
                .sbb => 0x19,
                .cmp => 0x39,
                .mov => 0x89,
                else => unreachable,
            };
            return .{ .opc = opc, .modrm_ext = undefined };
        },
        .rm => {
            const opc: u8 = switch (tag) {
                .adc => 0x13,
                .add => 0x03,
                .sub => 0x2b,
                .xor => 0x33,
                .@"and" => 0x23,
                .@"or" => 0x0b,
                .sbb => 0x1b,
                .cmp => 0x3b,
                .mov => 0x8b,
                else => unreachable,
            };
            return .{ .opc = opc, .modrm_ext = undefined };
        },
    }
}

fn mirArith(emit: *Emit, tag: Mir.Inst.Tag, inst: Mir.Inst.Index) InnerError!void {
    const ops = Mir.Ops.decode(emit.mir.instructions.items(.ops)[inst]);
    switch (ops.flags) {
        0b00 => blk: {
            if (ops.reg2 == .none) {
                // OP reg1, imm32
                // OP r/m64, imm32
                const imm = emit.mir.instructions.items(.data)[inst].imm;
                const opcode = getArithOpCode(tag, .mi);
                const opc = if (ops.reg1.size() == 8) opcode.opc - 1 else opcode.opc;
                const encoder = try Encoder.init(emit.code, 7);
                encoder.rex(.{
                    .w = ops.reg1.size() == 64,
                    .r = ops.reg1.isExtended(),
                });
                encoder.opcode_1byte(opc);
                encoder.modRm_direct(opcode.modrm_ext, ops.reg1.lowId());
                encoder.imm32(imm);
                break :blk;
            }
            // OP reg1, reg2
            // OP r/m64, r64
            const opcode = getArithOpCode(tag, .mr);
            const opc = if (ops.reg1.size() == 8) opcode.opc - 1 else opcode.opc;
            const encoder = try Encoder.init(emit.code, 3);
            encoder.rex(.{
                .w = ops.reg1.size() == 64,
                .r = ops.reg2.isExtended(),
                .b = ops.reg1.isExtended(),
            });
            encoder.opcode_1byte(opc);
            encoder.modRm_direct(ops.reg2.lowId(), ops.reg1.lowId());
        },
        0b01 => blk: {
            const imm = emit.mir.instructions.items(.data)[inst].imm;
            const opcode = getArithOpCode(tag, .rm);
            const opc = if (ops.reg1.size() == 8) opcode.opc - 1 else opcode.opc;
            if (ops.reg2 == .none) {
                // OP reg1, [imm32]
                // OP r64, r/m64
                const encoder = try Encoder.init(emit.code, 8);
                encoder.rex(.{
                    .w = ops.reg1.size() == 64,
                    .r = ops.reg1.isExtended(),
                });
                encoder.opcode_1byte(opc);
                encoder.modRm_SIBDisp0(ops.reg1.lowId());
                encoder.sib_disp32();
                encoder.disp32(imm);
                break :blk;
            }
            // OP reg1, [reg2 + imm32]
            // OP r64, r/m64
            const encoder = try Encoder.init(emit.code, 7);
            encoder.rex(.{
                .w = ops.reg1.size() == 64,
                .r = ops.reg1.isExtended(),
                .b = ops.reg2.isExtended(),
            });
            encoder.opcode_1byte(opc);
            if (imm <= math.maxInt(i8)) {
                encoder.modRm_indirectDisp8(ops.reg1.lowId(), ops.reg2.lowId());
                encoder.disp8(@intCast(i8, imm));
            } else {
                encoder.modRm_indirectDisp32(ops.reg1.lowId(), ops.reg2.lowId());
                encoder.disp32(imm);
            }
        },
        0b10 => blk: {
            if (ops.reg2 == .none) {
                // OP [reg1 + 0], imm32
                // OP r/m64, imm32
                const imm = emit.mir.instructions.items(.data)[inst].imm;
                const opcode = getArithOpCode(tag, .mi);
                const opc = if (ops.reg1.size() == 8) opcode.opc - 1 else opcode.opc;
                const encoder = try Encoder.init(emit.code, 7);
                encoder.rex(.{
                    .w = ops.reg1.size() == 64,
                    .b = ops.reg1.isExtended(),
                });
                encoder.opcode_1byte(opc);
                encoder.modRm_indirectDisp0(opcode.modrm_ext, ops.reg1.lowId());
                if (imm <= math.maxInt(i8)) {
                    encoder.imm8(@intCast(i8, imm));
                } else if (imm <= math.maxInt(i16)) {
                    encoder.imm16(@intCast(i16, imm));
                } else {
                    encoder.imm32(imm);
                }
                break :blk;
            }
            // OP [reg1 + imm32], reg2
            // OP r/m64, r64
            const imm = emit.mir.instructions.items(.data)[inst].imm;
            const opcode = getArithOpCode(tag, .mr);
            const opc = if (ops.reg1.size() == 8) opcode.opc - 1 else opcode.opc;
            const encoder = try Encoder.init(emit.code, 7);
            encoder.rex(.{
                .w = ops.reg1.size() == 64,
                .r = ops.reg2.isExtended(),
                .b = ops.reg1.isExtended(),
            });
            encoder.opcode_1byte(opc);
            if (imm <= math.maxInt(i8)) {
                encoder.modRm_indirectDisp8(ops.reg2.lowId(), ops.reg1.lowId());
                encoder.disp8(@intCast(i8, imm));
            } else {
                encoder.modRm_indirectDisp32(ops.reg2.lowId(), ops.reg1.lowId());
                encoder.disp32(imm);
            }
        },
        0b11 => {
            // OP [reg1 + imm32], imm32
            // OP r/m64, imm32
            const payload = emit.mir.instructions.items(.data)[inst].payload;
            const imm_pair = emit.mir.extraData(Mir.ImmPair, payload).data;
            const opcode = getArithOpCode(tag, .mi);
            const opc = if (ops.reg1.size() == 8) opcode.opc - 1 else opcode.opc;
            const encoder = try Encoder.init(emit.code, 11);
            encoder.rex(.{
                .w = ops.reg1.size() == 64,
                .b = ops.reg1.isExtended(),
            });
            encoder.opcode_1byte(opc);
            if (imm_pair.dest_off <= math.maxInt(i8)) {
                encoder.modRm_indirectDisp8(opcode.modrm_ext, ops.reg1.lowId());
                encoder.disp8(@intCast(i8, imm_pair.dest_off));
            } else {
                encoder.modRm_indirectDisp32(opcode.modrm_ext, ops.reg1.lowId());
                encoder.disp32(imm_pair.dest_off);
            }
            encoder.imm32(imm_pair.operand);
        },
    }
}

fn mirArithScaleSrc(emit: *Emit, tag: Mir.Inst.Tag, inst: Mir.Inst.Index) InnerError!void {
    const ops = Mir.Ops.decode(emit.mir.instructions.items(.ops)[inst]);
    const scale = ops.flags;
    // OP reg1, [reg2 + scale*rcx + imm32]
    const opcode = getArithOpCode(tag, .rm);
    const opc = if (ops.reg1.size() == 8) opcode.opc - 1 else opcode.opc;
    const imm = emit.mir.instructions.items(.data)[inst].imm;
    const encoder = try Encoder.init(emit.code, 8);
    encoder.rex(.{
        .w = ops.reg1.size() == 64,
        .r = ops.reg1.isExtended(),
        .b = ops.reg2.isExtended(),
    });
    encoder.opcode_1byte(opc);
    if (imm <= math.maxInt(i8)) {
        encoder.modRm_SIBDisp8(ops.reg1.lowId());
        encoder.sib_scaleIndexBaseDisp8(scale, Register.rcx.lowId(), ops.reg2.lowId());
        encoder.disp8(@intCast(i8, imm));
    } else {
        encoder.modRm_SIBDisp32(ops.reg1.lowId());
        encoder.sib_scaleIndexBaseDisp32(scale, Register.rcx.lowId(), ops.reg2.lowId());
        encoder.disp32(imm);
    }
}

fn mirArithScaleDst(emit: *Emit, tag: Mir.Inst.Tag, inst: Mir.Inst.Index) InnerError!void {
    const ops = Mir.Ops.decode(emit.mir.instructions.items(.ops)[inst]);
    const scale = ops.flags;
    const imm = emit.mir.instructions.items(.data)[inst].imm;

    if (ops.reg2 == .none) {
        // OP [reg1 + scale*rax + 0], imm32
        const opcode = getArithOpCode(tag, .mi);
        const opc = if (ops.reg1.size() == 8) opcode.opc - 1 else opcode.opc;
        const encoder = try Encoder.init(emit.code, 8);
        encoder.rex(.{
            .w = ops.reg1.size() == 64,
            .b = ops.reg1.isExtended(),
        });
        encoder.opcode_1byte(opc);
        encoder.modRm_SIBDisp0(opcode.modrm_ext);
        encoder.sib_scaleIndexBase(scale, Register.rax.lowId(), ops.reg1.lowId());
        if (imm <= math.maxInt(i8)) {
            encoder.imm8(@intCast(i8, imm));
        } else if (imm <= math.maxInt(i16)) {
            encoder.imm16(@intCast(i16, imm));
        } else {
            encoder.imm32(imm);
        }
        return;
    }

    // OP [reg1 + scale*rax + imm32], reg2
    const opcode = getArithOpCode(tag, .mr);
    const opc = if (ops.reg1.size() == 8) opcode.opc - 1 else opcode.opc;
    const encoder = try Encoder.init(emit.code, 8);
    encoder.rex(.{
        .w = ops.reg1.size() == 64,
        .r = ops.reg2.isExtended(),
        .b = ops.reg1.isExtended(),
    });
    encoder.opcode_1byte(opc);
    if (imm <= math.maxInt(i8)) {
        encoder.modRm_SIBDisp8(ops.reg2.lowId());
        encoder.sib_scaleIndexBaseDisp8(scale, Register.rax.lowId(), ops.reg1.lowId());
        encoder.disp8(@intCast(i8, imm));
    } else {
        encoder.modRm_SIBDisp32(ops.reg2.lowId());
        encoder.sib_scaleIndexBaseDisp32(scale, Register.rax.lowId(), ops.reg1.lowId());
        encoder.disp32(imm);
    }
}

fn mirArithScaleImm(emit: *Emit, tag: Mir.Inst.Tag, inst: Mir.Inst.Index) InnerError!void {
    const ops = Mir.Ops.decode(emit.mir.instructions.items(.ops)[inst]);
    const scale = ops.flags;
    const payload = emit.mir.instructions.items(.data)[inst].payload;
    const imm_pair = emit.mir.extraData(Mir.ImmPair, payload).data;
    const opcode = getArithOpCode(tag, .mi);
    const opc = if (ops.reg1.size() == 8) opcode.opc - 1 else opcode.opc;
    const encoder = try Encoder.init(emit.code, 2);
    encoder.rex(.{
        .w = ops.reg1.size() == 64,
        .b = ops.reg1.isExtended(),
    });
    encoder.opcode_1byte(opc);
    if (imm_pair.dest_off <= math.maxInt(i8)) {
        encoder.modRm_SIBDisp8(opcode.modrm_ext);
        encoder.sib_scaleIndexBaseDisp8(scale, Register.rax.lowId(), ops.reg1.lowId());
        encoder.disp8(@intCast(i8, imm_pair.dest_off));
    } else {
        encoder.modRm_SIBDisp32(opcode.modrm_ext);
        encoder.sib_scaleIndexBaseDisp32(scale, Register.rax.lowId(), ops.reg1.lowId());
        encoder.disp32(imm_pair.dest_off);
    }
    encoder.imm32(imm_pair.operand);
}

fn mirMovabs(emit: *Emit, inst: Mir.Inst.Index) InnerError!void {
    const tag = emit.mir.instructions.items(.tag)[inst];
    assert(tag == .movabs);
    const ops = Mir.Ops.decode(emit.mir.instructions.items(.ops)[inst]);

    const encoder = try Encoder.init(emit.code, 10);
    const is_64 = blk: {
        if (ops.flags == 0b00) {
            // movabs reg, imm64
            const opc: u8 = if (ops.reg1.size() == 8) 0xb0 else 0xb8;
            if (ops.reg1.size() == 64) {
                encoder.rex(.{
                    .w = true,
                    .b = ops.reg1.isExtended(),
                });
                encoder.opcode_withReg(opc, ops.reg1.lowId());
                break :blk true;
            }
            break :blk false;
        }
        if (ops.reg1 == .none) {
            // movabs moffs64, rax
            const opc: u8 = if (ops.reg2.size() == 8) 0xa2 else 0xa3;
            encoder.rex(.{
                .w = ops.reg2.size() == 64,
            });
            encoder.opcode_1byte(opc);
            break :blk ops.reg2.size() == 64;
        } else {
            // movabs rax, moffs64
            const opc: u8 = if (ops.reg2.size() == 8) 0xa0 else 0xa1;
            encoder.rex(.{
                .w = ops.reg1.size() == 64,
            });
            encoder.opcode_1byte(opc);
            break :blk ops.reg1.size() == 64;
        }
    };

    if (is_64) {
        const payload = emit.mir.instructions.items(.data)[inst].payload;
        const imm64 = emit.mir.extraData(Mir.Imm64, payload).data;
        encoder.imm64(imm64.decode());
    } else {
        const imm = emit.mir.instructions.items(.data)[inst].imm;
        if (imm <= math.maxInt(i8)) {
            encoder.imm8(@intCast(i8, imm));
        } else if (imm <= math.maxInt(i16)) {
            encoder.imm16(@intCast(i16, imm));
        } else {
            encoder.imm32(imm);
        }
    }
}

fn mirLea(emit: *Emit, inst: Mir.Inst.Index) InnerError!void {
    const tag = emit.mir.instructions.items(.tag)[inst];
    assert(tag == .lea);
    const ops = Mir.Ops.decode(emit.mir.instructions.items(.ops)[inst]);
    assert(ops.flags == 0b01);
    const imm = emit.mir.instructions.items(.data)[inst].imm;

    if (imm == 0) {
        const encoder = try Encoder.init(emit.code, 3);
        encoder.rex(.{
            .w = ops.reg1.size() == 64,
            .r = ops.reg1.isExtended(),
            .b = ops.reg2.isExtended(),
        });
        encoder.opcode_1byte(0x8d);
        encoder.modRm_indirectDisp0(ops.reg1.lowId(), ops.reg2.lowId());
    } else if (imm <= math.maxInt(i8)) {
        const encoder = try Encoder.init(emit.code, 4);
        encoder.rex(.{
            .w = ops.reg1.size() == 64,
            .r = ops.reg1.isExtended(),
            .b = ops.reg2.isExtended(),
        });
        encoder.opcode_1byte(0x8d);
        encoder.modRm_indirectDisp8(ops.reg1.lowId(), ops.reg2.lowId());
        encoder.disp8(@intCast(i8, imm));
    } else {
        const encoder = try Encoder.init(emit.code, 7);
        encoder.rex(.{
            .w = ops.reg1.size() == 64,
            .r = ops.reg1.isExtended(),
            .b = ops.reg2.isExtended(),
        });
        encoder.opcode_1byte(0x8d);
        encoder.modRm_indirectDisp32(ops.reg1.lowId(), ops.reg2.lowId());
        encoder.disp32(imm);
    }
}
