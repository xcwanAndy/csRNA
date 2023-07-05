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
 *  <kangning18z@ict.ac.cn>
 *
 *  Copyright (C) 2023 Xinyu Yang, HKUST.
 */


#ifndef __NIC_CTRL_HH__
#define __NIC_CTRL_HH__

#include <deque>
#include <queue>
#include <string>
#include <list>
#include <unordered_map>

#include "base/addr_range.hh"
#include "dev/rdma/hangu_rnic_defs.hh"
#include "dev/rdma/hangu_rnic.hh"
#include "dev/rdma/kfd_ioctl.h"

#include "base/inet.hh"
#include "debug/EthernetDesc.hh"
#include "debug/EthernetIntr.hh"
#include "dev/net/etherdevice.hh"
#include "dev/net/etherint.hh"
#include "dev/net/etherpkt.hh"
#include "dev/net/pktfifo.hh"
#include "dev/pci/device.hh"
#include "sim/syscall_emul_buf.hh"
#include "params/NicCtrl.hh"

using namespace HanGuRnicDef;

typedef struct {
    AddrRange vaddr;
    AddrRange paddr;
}MemBlock;

class MemAllocator {
    private:
        Addr baseAddr;
        // Ordered by paddr
        std::list<MemBlock> memMap;

    public:
        MemAllocator(Addr deviceAddr) {
            baseAddr = deviceAddr;
            MemBlock initBlock = {
                .vaddr = AddrRange(0, 0),
                .paddr = AddrRange(baseAddr, baseAddr)
            };
            memMap.emplace_front(initBlock);
        }
        ~MemAllocator();

        /* Alloc a block of memory
         * Return a virtual addr
         */
        MemBlock allocMem(size_t size);

        void destroyMem(MemBlock block);

        /* Get MemBlock from physical addr
         */
        MemBlock getPhyBlock(Addr paddr);

        /* Get MemBlock from virtual addr
         */
        MemBlock getVirBlock(Addr vaddr);

        /* Get physical addr from virtual addr
         */
        Addr getPhyAddr(Addr vaddr);

        /* Get virtual addr from physical addr
         */
        Addr getVirAddr(Addr paddr);
};

class NicCtrl : public PciDevice {
    private:
        HanGuRnic *rnic;
        Addr hcrAddr;
        Addr doorBell;
        MemAllocator memAlloc;

    public:
        typedef NicCtrlParams Params;
        const Params *params() const {
                return dynamic_cast<const Params *>(_params);
            }

        NicCtrl(const Params *params);
        ~NicCtrl();
        void init() override;

        // Control Interface
        int nicCtrl(unsigned req, Addr ioc_buf);
        EventFunctionWrapper nicCtrlEvent;
        unsigned nicCtrlReq;
        Addr ioc_buf;

        // PIO Interface
        Tick writeConfig(PacketPtr pkt) override;
        Tick read(PacketPtr pkt) override;
        Tick write(PacketPtr pkt) override;

        // mailbox;
        struct Mailbox {
            Addr addr;
            uint8_t *data;
        };
        Mailbox mailbox;
        Addr mailboxBase = 0xd100000000000000;
        void initMailbox();

        // Resc
        struct RescMeta {
            Addr     start ;  // start index (icm vaddr) of the resource
            uint64_t size  ;  // size of the resource(in byte)
            uint32_t entrySize; // size of one entry (in byte)
            uint8_t  entryNumLog;
            uint32_t entryNumPage;
            // ICM space bitmap, one bit indicates one page.
            uint8_t *bitmap;  // resource bitmap,
        };
        uint32_t allocResc(uint8_t rescType, RescMeta &rescMeta);

        // ICM
        std::unordered_map<Addr, Addr> icmAddrmap; // <icm vaddr page, icm paddr>
        Addr icmBase = 0xd200000000000000;
        void initIcm(uint8_t qpcNumLog, uint8_t cqcNumLog,
            uint8_t mptNumLog, uint8_t mttNumLog);
        uint8_t isIcmMapped(RescMeta &rescMeta, Addr index);
        Addr allocIcm(RescMeta &rescMeta, Addr index);
        void writeIcm(uint8_t rescType, RescMeta &rescMeta, Addr icmVPage);

        // HCR
        uint8_t checkHcr(PortProxy& portProxy);
        void postHcr(uint64_t inParam, uint32_t inMod, uint64_t outParam, uint8_t opcode);

        // MTT
        RescMeta mttMeta;
        void allocMtt(TypedBufferArg<kfd_ioctl_init_mtt_args> &args);
        void writeMtt(TypedBufferArg<kfd_ioctl_init_mtt_args> &args);

        // MPT
        RescMeta mptMeta;
        void allocMpt(TypedBufferArg<kfd_ioctl_alloc_mpt_args> &args);
        void writeMpt(TypedBufferArg<kfd_ioctl_write_mpt_args> &args);

        // CQC
        RescMeta cqcMeta;
        void allocCqc(TypedBufferArg<kfd_ioctl_alloc_cq_args> &args);
        void writeCqc(TypedBufferArg<kfd_ioctl_write_cqc_args> &args);

        // QPC
        RescMeta qpcMeta;
        void allocQpc(TypedBufferArg<kfd_ioctl_alloc_qp_args> &args);
        void writeQpc(TypedBufferArg<kfd_ioctl_write_qpc_args> &args);
};



#endif //__NIC_CTRL_HH__
