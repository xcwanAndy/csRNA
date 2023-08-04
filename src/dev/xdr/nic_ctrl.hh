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

#include "dev/xdr/accel_defs.hh"
#include "dev/rdma/hangu_rnic.hh"
#include "dev/rdma/kfd_ioctl.h"
#include "dev/xdr/accel.hh"

#include "base/inet.hh"
#include "dev/net/etherdevice.hh"
#include "dev/net/etherint.hh"
#include "dev/net/etherpkt.hh"
#include "dev/net/pktfifo.hh"
#include "dev/pci/device.hh"
#include "params/SrcClockDomain.hh"
#include "sim/syscall_emul_buf.hh"
#include "sim/sim_object.hh"
#include "params/NicCtrl.hh"

using namespace HanGuRnicDef;

class NicCtrl : public PciDevice {
    private:
        HanGuRnic *rnic;
        Addr cmdBase;
        Addr hcrAddr;
        Addr doorBell;
        MemAllocator mailboxAlloc;
        MemAllocator memAlloc;
        MemAllocator hostmemAlloc;

    public:
        typedef NicCtrlParams Params;
        const Params *params() const {
                return dynamic_cast<const Params *>(_params);
            }

        NicCtrl(const Params *params);
        ~NicCtrl();
        void init() override;

        Accel *accel;

        // Attirbute access
        Addr getDoorbell() { return doorBell; }
        Addr getCmdBase() { return cmdBase; }
        MemAllocator *getMemAlloc(){ return &memAlloc; }
        MemAllocator *getHostMemAlloc(){ return &hostmemAlloc; }

        // Control Interface
        //int nicCtrl();
        int nicCtrl(unsigned nicCtrlReq, void * ioc_buf);
        //EventFunctionWrapper nicCtrlEvent;
        //unsigned nicCtrlReq;
        std::queue<unsigned> nicCtrlReqFifo;
        //void *ioc_buf;
        std::queue<void *> ioc_buf_fifo;

        // PIO Interface
        Tick writeConfig(PacketPtr pkt) override;
        Tick read(PacketPtr pkt) override;
        Tick write(PacketPtr pkt) override;

        // mailbox;
        typedef struct {
            Addr src;
            uint64_t size;
            uint64_t inParam;
            uint32_t inMod;
            uint64_t outParam;
            uint8_t opcode;
        }MailElem;

        AddrRange mailboxRange;
        //void initMailbox();
        void scheduleMailbox(MailElem mailElem);
        std::queue<MailElem> mailFifo;

        EventFunctionWrapper sendMailEvent;
        void sendMail();
        uint8_t pendMailRecord;

        EventFunctionWrapper wait2SendEvent;
        void wait2Send();

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
        //Addr icmBase = 0xd200000000000000;
        void initIcm(uint8_t qpcNumLog, uint8_t cqcNumLog,
            uint8_t mptNumLog, uint8_t mttNumLog);
        uint8_t isIcmMapped(RescMeta &rescMeta, Addr index);
        Addr allocIcm(RescMeta &rescMeta, Addr index);
        void writeIcm(uint8_t rescType, Addr icmVPage);

        // HCR
        uint8_t checkHcr();
        void postHcr(uint64_t inParam, uint32_t inMod, uint64_t outParam, uint8_t opcode);

        // MTT
        RescMeta mttMeta;
        void allocMtt(struct kfd_ioctl_init_mtt_args *args);
        void writeMtt(struct kfd_ioctl_init_mtt_args *args);

        // MPT
        RescMeta mptMeta;
        void allocMpt(struct kfd_ioctl_alloc_mpt_args *args);
        void writeMpt(struct kfd_ioctl_write_mpt_args *args);

        // CQC
        RescMeta cqcMeta;
        void allocCqc(struct kfd_ioctl_alloc_cq_args *args);
        void writeCqc(struct kfd_ioctl_write_cqc_args *args);

        // QPC
        RescMeta qpcMeta;
        void allocQpc(struct kfd_ioctl_alloc_qp_args *args);
        void writeQpc(struct kfd_ioctl_write_qpc_args *args);
};



#endif //__NIC_CTRL_HH__
