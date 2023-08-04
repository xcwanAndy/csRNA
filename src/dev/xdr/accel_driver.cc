/*
 *======================= START OF LICENSE NOTICE =======================
 *  Copyright (C) 2021 Kang Ning, NCIC, ICT, CAS.
 *  All Rights Reserved.
 *
 *  NO WARRANTY. THE PRODUCT IS PROVIDED BY DEVELOPER "AS IS" AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL DEVELOPER BE LIABLE FOR
 *  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 *  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THE PRODUCT, EVEN
 *  IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *======================== END OF LICENSE NOTICE ========================
 *  Primary Author: Kang Ning
 *      <kangning18z@ict.ac.cn>
 *  Date : 2021.07.08
 */

#include "dev/xdr/accel_driver.hh"
#include "dev/xdr/kfd_ioctl.hh"
#include <cstring>



AccelDriver::AccelDriver(Params *p)
  : EmulatedDriver(p), accel(p->accel) {
    // HANGU_PRINT(AccelDriver, "HanGu RNIC driver.\n");
}

/**
 * Create an FD entry for the KFD inside of the owning process.
 */
int
AccelDriver::open(ThreadContext *tc, int mode, int flags) {

    HANGU_PRINT(AccelDriver, "open : %s.\n", filename);

    auto process = tc->getProcessPtr();
    auto device_fd_entry = std::make_shared<DeviceFDEntry>(this, filename);
    int tgt_fd = process->fds->allocFD(device_fd_entry);
    cpu_id = tc->contextId();

    // Configure PCI config space
    configDevice();

    return tgt_fd;
}

void
AccelDriver::configDevice() {

}

/**
 * Currently, mmap() will simply setup a mapping for the associated
 * rnic's send doorbells.
 */
Addr AccelDriver::mmap(ThreadContext *tc, Addr start, uint64_t length, int prot,
                int tgt_flags, int tgt_fd, int offset) {
    HANGU_PRINT(AccelDriver, " Accel mmap (start: %p, length: 0x%x,"
            "offset: 0x%x) cxt_id %d\n", start, length, offset, tc->contextId());

    auto process = tc->getProcessPtr();
    auto mem_state = process->memState;

    // Extend global mmap region if necessary.
    if (start == 0) {
        // Assume mmap grows down, as in x86 Linux.
        start = mem_state->getMmapEnd() - length;
        mem_state->setMmapEnd(start);
    }

    /**
     * Now map this virtual address to our PIO doorbell interface
     * in the page tables (non-cacheable).
     */
    NicCtrl *nic_ctrl = accel->ibv->nicCtrl;
    AddrRangeList addrList = nic_ctrl->getAddrRanges();
    HANGU_PRINT(AccelDriver, " addrList size %d\n", addrList.size());
    AddrRange baseAddrBar0 = addrList.front();
    HANGU_PRINT(AccelDriver, " baseAddrBar0.start 0x%x, baseAddrBar0.size() 0x%x\n", baseAddrBar0.start(), baseAddrBar0.size());
    process->pTable->map(start, baseAddrBar0.start(), 128, false);
    HANGU_PRINT(AccelDriver, " BAR0 mapped to 0x%x\n", start);
    hcrAddr = start + 8; /* The first 8 bytes are used for sync */
    return hcrAddr;
}


int AccelDriver::ioctl(ThreadContext *tc, unsigned req, Addr ioc_buf) {
    auto &virt_proxy = tc->getVirtProxy();
    size_t size_type = sizeof(struct accelkfd_ioctl_type);
    struct accelkfd_ioctl_type ioctl_type;

    Addr pAddr;
    auto process = tc->getProcessPtr();
    process->pTable->translate(ioc_buf, pAddr);

    switch (req) {
        case ACCELKFD_SEND_MR_ADDR:
            {
                HANGU_PRINT(AccelDriver, " ioctl : ACCELKFD_SEND_MR_ADDR.\n");

                TypedBufferArg<accelkfd_ioctl_mr_addr> args(ioc_buf);
                args.copyIn(virt_proxy);
                HANGU_PRINT(AccelDriver, "mr_addr: 0x%x, mr_len: %d\n", args->vaddr, args->size);

                process->pTable->translate((Addr)args->vaddr, (Addr &)args->paddr);
                HANGU_PRINT(AccelDriver, "The translated paddr: 0x%x\n", args->paddr);

                ioctl_type.type = ACCELKFD_SEND_MR_ADDR;
                ioctl_type.len = sizeof(struct accelkfd_ioctl_mr_addr);
                uint8_t *combined_args = (uint8_t *)malloc(size_type + ioctl_type.len);
                memcpy(combined_args, &ioctl_type, size_type);
                memcpy(combined_args + size_type, args, ioctl_type.len);

                virt_proxy.writeBlob(hcrAddr, combined_args, ioctl_type.len + size_type);
            }
            break;
        case ACCELKFD_START_CLT:
            {
                HANGU_PRINT(AccelDriver, " ioctl : ACCELKFD_START_CLT.\n");
                ioctl_type.type = ACCELKFD_START_CLT;
                ioctl_type.len = 0;
                uint8_t *combined_args = (uint8_t *)malloc(size_type + ioctl_type.len);
                memcpy(combined_args, &ioctl_type, size_type);

                virt_proxy.writeBlob(hcrAddr, combined_args, ioctl_type.len + size_type);
            }
            break;
        case ACCELKFD_START_SVR:
            {
                ioctl_type.type = ACCELKFD_START_SVR;
                ioctl_type.len = 0;
                uint8_t *combined_args = (uint8_t *)malloc(size_type + ioctl_type.len);
                memcpy(combined_args, &ioctl_type, size_type);

                virt_proxy.writeBlob(hcrAddr, combined_args, ioctl_type.len + size_type);
            }
            break;
        default:
            {
                fatal("%s: bad ioctl %d\n", req);
            }
            break;
    }
    return 0;
}

/* -------------------------- HCR {begin} ------------------------ */

uint8_t AccelDriver::checkHcr(PortProxy& portProxy) {

    uint32_t goOp;
    // HANGU_PRINT(AccelDriver, " Start read `GO`.\n");
    portProxy.readBlob(hcrAddr + (Addr)&(((HanGuRnicDef::Hcr*)0)->goOpcode), &goOp, sizeof(goOp));


    if ((goOp >> 31) == 1) {
        // HANGU_PRINT(AccelDriver, " `GO` is still high\n");
        return 1;
    }
    // HANGU_PRINT(AccelDriver, " `GO` is cleared.\n");
    return 0;
}

void AccelDriver::postHcr(PortProxy& portProxy, uint64_t inParam,
        uint32_t inMod, uint64_t outParam, uint8_t opcode) {

    HanGuRnicDef::Hcr hcr;

    HANGU_PRINT(AccelDriver, " Start Write hcr\n");

    hcr.inParam_l  = inParam & 0xffffffff;
    hcr.inParam_h  = inParam >> 32;
    hcr.inMod      = inMod;
    hcr.outParam_l = outParam & 0xffffffff;
    hcr.outParam_h = outParam >> 32;
    hcr.goOpcode   = (1 << 31) | opcode;
    // HANGU_PRINT(AccelDriver, " inParam_l: 0x%x\n", hcr.inParam_l);
    // HANGU_PRINT(AccelDriver, " inParam_h: 0x%x\n", hcr.inParam_h);
    // HANGU_PRINT(AccelDriver, " inMod: 0x%x\n", hcr.inMod);
    // HANGU_PRINT(AccelDriver, " outParam_l: 0x%x\n", hcr.outParam_l);
    // HANGU_PRINT(AccelDriver, " outParam_h: 0x%x\n", hcr.outParam_h);
    // HANGU_PRINT(AccelDriver, " goOpcode: 0x%x\n", hcr.goOpcode);

    portProxy.writeBlob(hcrAddr, &hcr, sizeof(hcr));
}

/* -------------------------- HCR {end} ------------------------ */


AccelDriver* AccelDriverParams::create()
{
    return new AccelDriver(this);
}
