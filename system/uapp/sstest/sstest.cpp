// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <fbl/type_support.h>
#include <fbl/unique_ptr.h>
#include <fdio/io.h>
#include <inttypes.h>
#include <launchpad/launchpad.h>
#include <stdio.h>
#include <string.h>
#include <zircon/syscalls/debug.h>
#include <zircon/syscalls/exception.h>
#include <lib/zx/port.h>
#include <lib/zx/process.h>
#include <lib/zx/thread.h>

static constexpr uint64_t kExceptionKey = 0x7676767676767676;

zx::thread ThreadForTid(const zx::process& process, uint64_t tid) {
    zx_handle_t thread_handle = 0;
    zx_object_get_child(process.get(), tid, ZX_RIGHT_SAME_RIGHTS, &thread_handle);
    return zx::thread(thread_handle);
}

bool GetGeneralRegs(const zx::thread& thread, zx_thread_state_general_regs* regs) {
    return thread.read_state(ZX_THREAD_STATE_GENERAL_REGS, regs,
                             sizeof(zx_thread_state_general_regs)) == ZX_OK;
}

bool SetGeneralRegs(zx::thread& thread, const zx_thread_state_general_regs& regs) {
    return thread.write_state(ZX_THREAD_STATE_GENERAL_REGS, &regs,
                              sizeof(zx_thread_state_general_regs)) == ZX_OK;
}

bool SetSingleStep(zx::thread& thread, bool enable) {
    zx_thread_state_single_step_t value = enable ? 1 : 0;
    zx_status_t status = thread.write_state(ZX_THREAD_STATE_SINGLE_STEP, &value, sizeof(value));
    if (status != ZX_OK) {
        fprintf(stderr, "SetSingleStep failed with %d\n", (int)status);
        return false;
    }
    return true;
}

void PrintSingleStepState(const zx::thread& thread, const char* msg) {
    zx_thread_state_single_step_t value;
    zx_status_t status = thread.read_state(ZX_THREAD_STATE_SINGLE_STEP, &value, sizeof(value));
    if (status == ZX_OK) {
        fprintf(stderr, "PARENT: %s single step state = %d\n", msg, (int)value);
    } else {
        fprintf(stderr, "PARENT: Unable to read back state\n");
    }
}

void PrintRegs(const zx_thread_state_general_regs& regs) {
    fprintf(stderr, "PARENT: x0 = 0x%" PRIx64 "  x1 = 0x%" PRIx64 "\n", regs.r[0], regs.r[1]);
}

zx::process Launch() {
    //const char kPath[] = "/system/bin/sstest";  // QEMU
    const char kPath[] = "/boot/bin/sstest";  // Hikey

    launchpad_t* lp;
    zx_status_t status = launchpad_create(0, "sstest", &lp);
    if (status != ZX_OK)
        return zx::process();

    status = launchpad_load_from_file(lp, kPath);
    if (status != ZX_OK)
        return zx::process();

    const char* argv[2] = {kPath, "--child"};
    status = launchpad_set_args(lp, 2, argv);
    if (status != ZX_OK)
        return zx::process();

    status = launchpad_transfer_fd(lp, 1, FDIO_FLAG_USE_FOR_STDIO | 0);
    if (status != ZX_OK)
        return zx::process();

    launchpad_clone(lp, LP_CLONE_FDIO_NAMESPACE | LP_CLONE_ENVIRON | LP_CLONE_DEFAULT_JOB);
    if (status != ZX_OK)
        return zx::process();

    zx_handle_t child;
    status = launchpad_go(lp, &child, NULL);
    if (status != ZX_OK)
        return zx::process();

    return zx::process(child);
}

zx::port AttachToDebugPort(const zx::process& proc) {
    zx::port port;
    zx_status_t status = zx::port::create(0, &port);
    if (status != ZX_OK)
        return zx::port();

    status = zx_task_bind_exception_port(proc.get(), port.get(), kExceptionKey,
                                         ZX_EXCEPTION_PORT_DEBUGGER);
    if (status != ZX_OK)
        return zx::port();

    status = proc.wait_async(port, kExceptionKey, ZX_PROCESS_TERMINATED, ZX_WAIT_ASYNC_REPEATING);
    if (status != ZX_OK)
        return zx::port();

    return fbl::move(port);
}

void SoftwareBreakpoint(zx::thread& thread) {
    zx_thread_state_general_regs regs;
    GetGeneralRegs(thread, &regs);
    fprintf(stderr, "PARENT: Got software breakpoint @ 0x%" PRIx64 "\n", regs.pc);

    // The context switch code checks for (x0 + 3) * (x1 + 7) + 22 == x3
    // to write the MDSCR to r0 and SPSR to x1. These numbers satisfy that requirement.
    PrintRegs(regs);
    regs.r[0] = 1;
    regs.r[1] = 1;
    regs.r[3] = 54;

    // Software breakpoints will have the exception pointing to the actual address. To move
    // beyond it, advance 4 bytes.
    regs.pc += 4;
    SetGeneralRegs(thread, regs);

    fprintf(stderr, "PARENT: Single-stepping after software breakpoint @ 0x%" PRIx64 "\n", regs.pc);

    // Resume via single step.
    PrintSingleStepState(thread, "Before setting");
    SetSingleStep(thread, true);
    PrintSingleStepState(thread, "After setting");
    thread.resume(ZX_RESUME_EXCEPTION);
}

// Returns false to exit program.
bool HardwareBreakpoint(zx::thread& thread) {
    zx_thread_state_general_regs regs;
    GetGeneralRegs(thread, &regs);
    fprintf(stderr, "PARENT: Got hardware breakpoint @ 0x%" PRIx64 "\n", regs.pc);
    PrintRegs(regs);

    // regs.pc += 4;
    // SetGeneralRegs(thread, regs);

    static int hardware_exceptions_left = 3;
    hardware_exceptions_left--;
    if (hardware_exceptions_left == 0) {
        fprintf(stderr, "PARENT: Too many hardware exceptions, giving up\n");
        return false;
    }

    // Resume via single step.
    SetSingleStep(thread, true);
    thread.resume(ZX_RESUME_EXCEPTION);
    return true;
}

int RunParent() {
    zx::process child = Launch();
    if (child == 0) {
        fprintf(stderr, "PARENT: Failed to launch child.\n");
        return 1;
    }

    zx::port debug_port = AttachToDebugPort(child);
    if (!debug_port.is_valid()) {
        fprintf(stderr, "PARENT: Debug port attachment failed\n");
        return 1;
    }

    // Exception loop.
    zx_port_packet_t packet;
    while (true) {
        zx_status_t status = debug_port.wait(zx::time::infinite(), &packet, 1);
        if (status != ZX_OK) {
            fprintf(stderr, "PARENT: Wait failed\n");
            return 1;
        }
        if (ZX_PKT_IS_EXCEPTION(packet.type)) {
            zx::thread thread = ThreadForTid(child, packet.exception.tid);
            switch (packet.type) {
            case ZX_EXCP_THREAD_STARTING:
                thread.resume(ZX_RESUME_EXCEPTION);
                break;
            case ZX_EXCP_SW_BREAKPOINT:
                SoftwareBreakpoint(thread);
                break;
            case ZX_EXCP_HW_BREAKPOINT:
                if (!HardwareBreakpoint(thread)) {
                    child.kill();
                    return 0;
                }
                break;
            default:
                fprintf(stderr, "PARENT: Some other kind of exception %x\n", (unsigned)packet.type);
                break;
            }
            continue;
        } else {
            //fprintf(stderr, "PARENT unknown port event type = %d = 0x%x\n", packet.type, packet.type);
            fprintf(stderr, "PARENT unknown port event (type = 0x%x), exiting\n", (int)packet.type);
            return 0;
        }
    }

    return 0;
}

int RunChild() {
    sleep(1);  // Give the parent a chance to attach to the debug port.
    uint64_t pc;

    asm volatile("adr %0, ." : "=r"(pc));
    printf("Child PC = 0x%" PRIx64 "\n", pc);
    
    asm volatile("brk 0");
    volatile int x = 4;
    x = x + 1;
    x = 0;


    asm volatile("brk 0");
    fprintf(stderr, "CHILD: Continued from break\n");
    return x;
}

int main(int argc, char* argv[]) {
    for (int i = 0; i < argc; i++) {
        if (strcmp(argv[i], "--child") == 0)
            return RunChild();
    }
    return RunParent();
}
