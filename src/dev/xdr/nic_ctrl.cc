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

#include "dev/xdr/nic_ctrl.hh"


#include <algorithm>
#include <cstring>
#include <memory>
#include <queue>

#include "base/addr_range.hh"
#include "base/inet.hh"
#include "base/trace.hh"
#include "base/random.hh"
#include "debug/Drain.hh"
#include "dev/net/etherpkt.hh"
#include "debug/XDR.hh"
#include "dev/pci/device.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/NicCtrl.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

using namespace HanGuRnicDef;
using namespace Net;
using namespace std;

NicCtrl::NicCtrl(const Params *p)
    : PciDevice(p),
    rnic(&p->rnic),
    memAlloc(0xd300000000000000),
    nicCtrlEvent([this]{ nicCtrl(); }, name()),
    dmaReadDelay(p->dma_read_delay),
    dmaWriteDelay(p->dma_write_delay),
    pciBandwidth(p->pci_speed),
    etherBandwidth(p->ether_speed),
    LinkDelay     (p->link_delay),
    ethRxPktProcEvent([this]{ ethRxPktProc(); }, name())
{

        DPRINTF(NicCtrl, " qpc_cache_cap %d  reorder_cap %d cpuNum 0x%x\n",
                p->qpc_cache_cap, p->reorder_cap, p->cpu_num);

        BARSize[0]  = (1 << 12);
        BARAddrs[0] = 0xd000000000000000;

        AddrRangeList addr_list = rnic->getAddrRanges();
        AddrRange bar0 = addr_list.pop_front(); // Get BAR0
        hcrAddr = bar0.start();

        doorBell = hcrAddr + 0x18;
} // NicCtrl::NicCtrl

NicCtrl::~NicCtrl() {
}

void NicCtrl::init() {
    PciDevice::init();
}

NicCtrl* HanGuRnicParams::create() {
    return new NicCtrl(this);
}


/*********************** Control Interface ****************************/
/* This funtion is corresponded to ioctl in hangu_driver
 * This function should be scheduled by upper layer
 */
int NicCtrl::nicCtrl() {

    if (HGKFD_IOC_GET_TIME == nicCtrlReq) {
        DPRINTF(NicCtrl, " ioctl: HGKFD_IOC_GET_TIME %ld\n", curTick());

        /* TODO: TypedBufferArg should be substituted. */
        /* Get && copy current time */
        TypedBufferArg<kfd_ioctl_get_time_args> args(ioc_buf);
        args->cur_time = curTick();

        return 0;
    //} else if (checkHcr(virt_proxy)) {
        //DPRINTF(NicCtrl, " `GO` bit is still high! Try again later.\n");
        //return -1;
    }

    /* ioc_buf should be allocated by memAlloc
     */
    Addr pAddr = memAlloc.getPhyAddr(ioc_buf);

    switch (nicCtrlReq) {
      case HGKFD_IOC_INIT_DEV: // Input
        {
            DPRINTF(NicCtrl, " ioctl : HGKFD_IOC_INIT_DEV.\n");

            TypedBufferArg<kfd_ioctl_init_dev_args> args(ioc_buf);

            initMailbox();
            DPRINTF(NicCtrl, " HGKFD_IOC_INIT_DEV mailbox initialized\n");

            // We don't use input parameter here
            initIcm(RESC_LEN_LOG, RESC_LEN_LOG, RESC_LEN_LOG, RESC_LEN_LOG);
        }
        break;
      case HGKFD_IOC_ALLOC_MTT: // Input Output
        {
            DPRINTF(NicCtrl, " ioctl : HGKFD_IOC_ALLOC_MTT.\n");
            TypedBufferArg<kfd_ioctl_init_mtt_args> args(ioc_buf);
            allocMtt(args);
            DPRINTF(NicCtrl, " HGKFD_IOC_ALLOC_MTT mtt allocated\n");

            uint32_t last_mtt_index = (args->mtt_index + args->batch_size - 1);
            if (!isIcmMapped(mttMeta, last_mtt_index)) { /* last mtt index in this allocation */
                DPRINTF(NicCtrl, " HGKFD_IOC_ALLOC_MTT mtt not mapped\n");
                Addr icmVPage = allocIcm (mttMeta, args->mtt_index);
                writeIcm(HanGuRnicDef::ICMTYPE_MTT, mttMeta, icmVPage);
                DPRINTF(NicCtrl, " HGKFD_IOC_ALLOC_MTT mtt ICM mapping is written\n");
            }
        }
        break;
      case HGKFD_IOC_WRITE_MTT: // Input
        {
            DPRINTF(NicCtrl, " ioctl : HGKFD_IOC_WRITE_MTT.\n");
            TypedBufferArg<kfd_ioctl_init_mtt_args> args(ioc_buf);
            writeMtt(args);
        }
        break;
      case HGKFD_IOC_ALLOC_MPT: // Output
        {
            TypedBufferArg<kfd_ioctl_alloc_mpt_args> args(ioc_buf);
            DPRINTF(NicCtrl, " ioctl : HGKFD_IOC_ALLOC_MPT. batch_size %d\n", args->batch_size);

            allocMpt(args);

            DPRINTF(NicCtrl, " get into ioctl HGKFD_IOC_ALLOC_MPT: mpt_start_index: %d\n", args->mpt_index);
            if (!isIcmMapped(mptMeta, args->mpt_index + args->batch_size - 1)) {
                Addr icmVPage = allocIcm(mptMeta, args->mpt_index);
                writeIcm(HanGuRnicDef::ICMTYPE_MPT, mptMeta, icmVPage);
            }
        }
        break;
      case HGKFD_IOC_WRITE_MPT: // Input
        {
            DPRINTF(NicCtrl, " ioctl : HGKFD_IOC_WRITE_MPT.\n");

            TypedBufferArg<kfd_ioctl_write_mpt_args> args(ioc_buf);

            writeMpt(args);
        }
        break;
      case HGKFD_IOC_ALLOC_CQ: // Output
        {
            DPRINTF(NicCtrl, " ioctl : HGKFD_IOC_ALLOC_CQ.\n");
            TypedBufferArg<kfd_ioctl_alloc_cq_args> args(ioc_buf);

            allocCqc(args);

            if (!isIcmMapped(cqcMeta, args->cq_num)) {
                Addr icmVPage = allocIcm (cqcMeta, args->cq_num);
                writeIcm(HanGuRnicDef::ICMTYPE_CQC, cqcMeta, icmVPage);
            }
        }
        break;
      case HGKFD_IOC_WRITE_CQC: // Input
        {
            DPRINTF(NicCtrl, " ioctl : HGKFD_IOC_WRITE_CQC.\n");

            TypedBufferArg<kfd_ioctl_write_cqc_args> args(ioc_buf);

            writeCqc(args);
        }
        break;
      case HGKFD_IOC_ALLOC_QP: // Output
        {
            DPRINTF(NicCtrl, " ioctl : HGKFD_IOC_ALLOC_QP.\n");
            TypedBufferArg<kfd_ioctl_alloc_qp_args> args(ioc_buf);

            allocQpc(args);

            if (!isIcmMapped(qpcMeta, args->qp_num + args->batch_size - 1)) {
                Addr icmVPage = allocIcm (qpcMeta, args->qp_num);
                writeIcm(HanGuRnicDef::ICMTYPE_QPC, qpcMeta, icmVPage);
            }
            DPRINTF(NicCtrl, " ioctl : HGKFD_IOC_ALLOC_QP, qp_num: 0x%x(%d), batch_size %d\n",
                    args->qp_num, RESC_LIM_MASK&args->qp_num, args->batch_size);
        }
        break;
      case HGKFD_IOC_WRITE_QPC: // Input
        {
            DPRINTF(NicCtrl, " ioctl : HGKFD_IOC_WRITE_QPC\n");
            TypedBufferArg<kfd_ioctl_write_qpc_args> args(ioc_buf);
            writeQpc(args);
        }
        break;
      case HGKFD_IOC_CHECK_GO: 
        {   
            /* We don't check `go` bit here, cause it 
             * has been checked at the beginning of ioctl. */
            // DPRINTF(NicCtrl, " ioctl : HGKFD_IOC_CHECK_GO, `GO` is cleared.\n");
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

/*********************** Control Interface ****************************/

///////////////////////// NicCtrl::PIO relevant {begin}////////////////////////

Tick NicCtrl::writeConfig(PacketPtr pkt) {
    int offset = pkt->getAddr() & PCI_CONFIG_SIZE;
    if (offset < PCI_DEVICE_SPECIFIC) {
        PciDevice::writeConfig(pkt);
    }
    else {
        panic("Device specific PCI config space not implemented.\n");
    }

    /* !TODO: We will implement PCI configuration here.
     * Some work may need to be done here based for the pci
     * COMMAND bits, we don't realize now. */

    return configDelay;
}


Tick NicCtrl::read(PacketPtr pkt) {
    int bar;
    Addr daddr;

    if (!getBAR(pkt->getAddr(), bar, daddr)) {
        panic("Invalid PCI memory access to unmapped memory.\n");
    }

    /* Only HCR Space (BAR0-1) is allowed */
    assert(bar == 0);

    /* Only 32bit accesses allowed */
    assert(pkt->getSize() == 4);

    DPRINTF(PioEngine, " Read device addr 0x%x, pioDelay: %d\n", daddr, pioDelay);


    /* Handle read of register here.
     * Here we only implement read go bit */
    if (daddr == (Addr)&(((HanGuRnicDef::Hcr*)0)->goOpcode)) {/* Access `GO` bit */
        pkt->setLE<uint32_t>(regs.cmdCtrl.go()<<31 | regs.cmdCtrl.op());
    } else if (daddr == 0x20) {/* Access `sync` reg */
        pkt->setLE<uint32_t>(syncSucc);
    } else {
        pkt->setLE<uint32_t>(0);
    }

    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick NicCtrl::write(PacketPtr pkt) {
    int bar;
    Addr daddr;

    DPRINTF(PioEngine, " PioEngine.write: pkt addr 0x%x, size 0x%x\n",
            pkt->getAddr(), pkt->getSize());

    if (!getBAR(pkt->getAddr(), bar, daddr)) {
        panic("Invalid PCI memory access to unmapped memory.\n");
    }

    /* Only BAR 0 is allowed */
    assert(bar == 0);

    if (daddr == 0 && pkt->getSize() == sizeof(Hcr)) {
        DPRINTF(PioEngine,
                " PioEngine.write: HCR, inparam: 0x%x\n",
                pkt->getLE<Hcr>().inParam_l);

        regs.inParam.iparaml(pkt->getLE<Hcr>().inParam_l);
        regs.inParam.iparamh(pkt->getLE<Hcr>().inParam_h);
        regs.modifier = pkt->getLE<Hcr>().inMod;
        regs.outParam.oparaml(pkt->getLE<Hcr>().outParam_l);
        regs.outParam.oparamh(pkt->getLE<Hcr>().outParam_h);
        regs.cmdCtrl = pkt->getLE<Hcr>().goOpcode;

        /* Schedule CEU */
        if (!ceuProcEvent.scheduled()) {
            schedule(ceuProcEvent, curTick() + clockPeriod());
        }

    } else if (daddr == 0x18 && pkt->getSize() == sizeof(uint64_t)) {

        /*  Used to Record start of time */
        DPRINTF(NicCtrl,
                " PioEngine.write: Doorbell, value %#X pio interval %ld\n",
                pkt->getLE<uint64_t>(), curTick() - this->tick);

        regs.db._data = pkt->getLE<uint64_t>();

        DoorbellPtr dbell = make_shared<DoorbellFifo>(regs.db.opcode(),
                regs.db.num(), regs.db.qpn(), regs.db.offset());
        pio2ccuDbFifo.push(dbell);

        /* Record last tick */
        this->tick = curTick();

        /* Schedule doorbellProc */
        if (!doorbellProcEvent.scheduled()) {
            schedule(doorbellProcEvent, curTick() + clockPeriod());
        }

        DPRINTF(NicCtrl, " PioEngine.write: qpn %d, opcode %x, num %d\n",
                regs.db.qpn(), regs.db.opcode(), regs.db.num());
    } else if (daddr == 0x20
            && pkt->getSize() == sizeof(uint32_t)) {
            /* latency sync */

        DPRINTF(NicCtrl,
                " PioEngine.write: sync bit, value %#X, syncCnt %d\n",
                pkt->getLE<uint32_t>(), syncCnt);

        if (pkt->getLE<uint32_t>() == 1) {
            syncCnt += 1;
            assert(syncCnt <= cpuNum);
            if (syncCnt == cpuNum) {
                syncSucc = 1;
            }
        } else {
            assert(syncCnt > 0);
            syncCnt -= 1;
            if (syncCnt == 0) {
                syncSucc = 0;
            }
        }

        DPRINTF(NicCtrl,
                " PioEngine.write: sync bit end, value %#X, syncCnt %d\n",
                pkt->getLE<uint32_t>(), syncCnt);
    } else {
        panic("Write request to unknown address : %#x && size 0x%x\n",
                daddr, pkt->getSize());
    }

    pkt->makeAtomicResponse();
    return pioDelay;
}
/////////////////////////// NicCtrl::PIO relevant {end}////////////////////////


/////////////////////////// NicCtrl::HCR relevant {begin}//////////////////////

uint8_t NicCtrl::checkHcr() {

    uint32_t goOp;
    // DPRINTF(NicCtrl, " Start read `GO`.\n");
    dmaRead(hcrAddr + (Addr)&(((HanGuRnicDef::Hcr*)0)->goOpcode),
            sizeof(goOp), nullptr, &goOp);

    if ((goOp >> 31) == 1) {
        // DPRINTF(NicCtrl, " `GO` is still high\n");
        return 1;
    }
    // DPRINTF(NicCtrl, " `GO` is cleared.\n");
    return 0;
}

void NicCtrl::postHcr(uint64_t inParam,
        uint32_t inMod, uint64_t outParam, uint8_t opcode) {

    HanGuRnicDef::Hcr hcr;

    DPRINTF(NicCtrl, " Start Write hcr\n");

    hcr.inParam_l  = inParam & 0xffffffff;
    hcr.inParam_h  = inParam >> 32;
    hcr.inMod      = inMod;
    hcr.outParam_l = outParam & 0xffffffff;
    hcr.outParam_h = outParam >> 32;
    hcr.goOpcode   = (1 << 31) | opcode;
    DPRINTF(NicCtrl, " inParam_l: 0x%x\n", hcr.inParam_l);
    DPRINTF(NicCtrl, " inParam_h: 0x%x\n", hcr.inParam_h);
    DPRINTF(NicCtrl, " inMod: 0x%x\n", hcr.inMod);
    DPRINTF(NicCtrl, " outParam_l: 0x%x\n", hcr.outParam_l);
    DPRINTF(NicCtrl, " outParam_h: 0x%x\n", hcr.outParam_h);
    DPRINTF(NicCtrl, " goOpcode: 0x%x\n", hcr.goOpcode);

    dmaWrite(hcrAddr, sizeof(hcr), nullptr, &hcr);
}

////////////////////////// NicCtrl::HCR relevant {end}/////////////////////////

/* -------------------------- Mailbox {begin} ------------------------ */
void NicCtrl::initMailbox() {
    // The size of mailbox: 128KB
    uint32_t allocPages = MAILBOX_PAGE_NUM;
    mailbox.data = new uint8_t[4096 * allocPages];

    // The index
    mailbox.addr = mailboxBase;

    DPRINTF(NicCtrl, " mailbox.addr : 0x%x\n", mailbox.addr);
}

/* -------------------------- Mailbox {end} ------------------------ */

/* -------------------------- ICM {begin} ------------------------ */
// Interconnect Context Memory (ICM)
void NicCtrl::initIcm(uint8_t qpcNumLog, uint8_t cqcNumLog,
        uint8_t mptNumLog, uint8_t mttNumLog) {

    Addr startPtr = 0;

    mttMeta.start     = startPtr;
    mttMeta.size      = ((1 << mttNumLog) * sizeof(HanGuRnicDef::MttResc));
    mttMeta.entrySize = sizeof(HanGuRnicDef::MttResc);
    mttMeta.entryNumLog = mttNumLog;
    mttMeta.entryNumPage= (1 << (mttNumLog-(12-3)));
    mttMeta.bitmap    = new uint8_t[mttMeta.entryNumPage];
    memset(mttMeta.bitmap, 0, mttMeta.entryNumPage);
    startPtr += mttMeta.size;
    DPRINTF(NicCtrl, " mttMeta.entryNumPage 0x%x\n", mttMeta.entryNumPage);

    mptMeta.start = startPtr;
    mptMeta.size  = ((1 << mptNumLog) * sizeof(HanGuRnicDef::MptResc));
    mptMeta.entrySize = sizeof(HanGuRnicDef::MptResc);
    mptMeta.entryNumLog = mptNumLog;
    mptMeta.entryNumPage = (1 << (mptNumLog-(12-5)));
    mptMeta.bitmap = new uint8_t[mptMeta.entryNumPage];
    memset(mptMeta.bitmap, 0, mptMeta.entryNumPage);
    startPtr += mptMeta.size;
    DPRINTF(NicCtrl, " mptMeta.entryNumPage 0x%x\n", mptMeta.entryNumPage);

    cqcMeta.start = startPtr;
    cqcMeta.size  = ((1 << cqcNumLog) * sizeof(HanGuRnicDef::CqcResc));
    cqcMeta.entrySize = sizeof(HanGuRnicDef::CqcResc);
    cqcMeta.entryNumLog = cqcNumLog;
    cqcMeta.entryNumPage = (1 << (cqcNumLog-(12-4)));
    cqcMeta.bitmap = new uint8_t[cqcMeta.entryNumPage];
    memset(cqcMeta.bitmap, 0, cqcMeta.entryNumPage);
    startPtr += cqcMeta.size;
    DPRINTF(NicCtrl, " cqcMeta.entryNumPage 0x%x\n", cqcMeta.entryNumPage);

    qpcMeta.start = startPtr;
    qpcMeta.size  = ((1 << qpcNumLog) * sizeof(HanGuRnicDef::QpcResc));
    qpcMeta.entrySize   = sizeof(HanGuRnicDef::QpcResc);
    qpcMeta.entryNumLog = qpcNumLog;
    qpcMeta.entryNumPage = (1 << (qpcNumLog-(12-8)));
    qpcMeta.bitmap = new uint8_t[qpcMeta.entryNumPage];
    memset(qpcMeta.bitmap, 0, qpcMeta.entryNumPage);
    startPtr += qpcMeta.size;
    DPRINTF(NicCtrl, " qpcMeta.entryNumPage 0x%x\n", qpcMeta.entryNumPage);

    /* put initResc into mailbox */
    HanGuRnicDef::InitResc initResc;
    initResc.qpcBase   = qpcMeta.start;
    initResc.qpsNumLog = qpcNumLog;
    initResc.cqcBase   = cqcMeta.start;
    initResc.cqsNumLog = cqcNumLog;
    initResc.mptBase   = mptMeta.start;
    initResc.mptNumLog = mptNumLog;
    initResc.mttBase   = mttMeta.start;
    // DPRINTF(NicCtrl, " qpcMeta.start: 0x%lx, cqcMeta.start : 0x%lx,
    //         mptMeta.start : 0x%lx, mttMeta.start : 0x%lx\n",
    //         qpcMeta.start, cqcMeta.start, mptMeta.start, mttMeta.start);
    memcpy(mailbox.data, &initResc, sizeof(HanGuRnicDef::InitResc));
    postHcr((uint64_t)mailbox.addr, 0, 0, HanGuRnicDef::INIT_ICM);
}


uint8_t NicCtrl::isIcmMapped(RescMeta &rescMeta, Addr index) {
    Addr icmVPage = (rescMeta.start + index * rescMeta.entrySize) >> 12;
    return (icmAddrmap.find(icmVPage) != icmAddrmap.end());
}

Addr NicCtrl::allocIcm(RescMeta &rescMeta, Addr index) {
    Addr icmVPage = (rescMeta.start + index * rescMeta.entrySize) >> 12;
    while (icmAddrmap.find(icmVPage) != icmAddrmap.end()) {
        /* cause we allocate multiply resources one time,
         * the resources may be cross-page. */
        ++icmVPage;
    }
    DPRINTF(NicCtrl, " rescMeta.start: 0x%lx, index 0x%x, entrySize %d icmVPage 0x%lx\n",
            rescMeta.start, index, rescMeta.entrySize, icmVPage);
    for (uint32_t i =  0; i < ICM_ALLOC_PAGE_NUM; ++i) {
        if (i == 0) {
            MemBlock memBlock = memAlloc.allocMem(4096 * ICM_ALLOC_PAGE_NUM);
            icmAddrmap[icmVPage] = memBlock.paddr.start();
        } else {
            icmAddrmap[icmVPage + i] = icmAddrmap[icmVPage] + (i << 12);
        }
        DPRINTF(NicCtrl, " icmAddrmap[0x%x(%d)]: 0x%lx\n", icmVPage+i, i, icmAddrmap[icmVPage+i]);
    }
    return icmVPage;
}

void NicCtrl::writeIcm(uint8_t rescType, Addr icmVPage) {

    // put IcmResc into mailbox
    HanGuRnicDef::IcmResc icmResc;
    icmResc.pageNum = ICM_ALLOC_PAGE_NUM; // now we support ICM_ALLOC_PAGE_NUM pages
    icmResc.vAddr   = icmVPage << 12;
    icmResc.pAddr   = icmAddrmap[icmVPage];
    memcpy(mailbox.data, &icmResc, sizeof(HanGuRnicDef::InitResc));
    DPRINTF(NicCtrl, " pageNum %d, vAddr 0x%lx, pAddr 0x%lx\n", icmResc.pageNum, icmResc.vAddr, icmResc.pAddr);
    postHcr((uint64_t)mailbox.addr, 1, rescType, HanGuRnicDef::WRITE_ICM);
}

/* -------------------------- ICM {end} ------------------------ */


/* -------------------------- Resc {begin} ------------------------ */
uint32_t NicCtrl::allocResc(uint8_t rescType, RescMeta &rescMeta) {
    uint32_t i = 0, j = 0;
    uint32_t rescNum = 0;
    while (rescMeta.bitmap[i] == 0xff) {
        ++i;
    }
    rescNum = i * 8;

    while ((rescMeta.bitmap[i] >> j) & 0x01) {
        ++rescNum;
        ++j;
    }
    rescMeta.bitmap[i] |= (1 << j);

    rescNum += (cpu_id << RESC_LIM_LOG);
    return rescNum;
}
/* -------------------------- Resc {end} ------------------------ */


/* -------------------------- MTT {begin} ------------------------ */

void NicCtrl::allocMtt(TypedBufferArg<kfd_ioctl_init_mtt_args> &args) {
    for (uint32_t i = 0; i < args->batch_size; ++i) {
        args->mtt_index = allocResc(HanGuRnicDef::ICMTYPE_MTT, mttMeta);
        DPRINTF(NicCtrl, " HGKFD_IOC_ALLOC_MTT: mtt_bitmap: %d\n", mttMeta.bitmap[0]);
        DPRINTF(NicCtrl, " HGKFD_IOC_ALLOC_MTT: mtt_index: %d\n", args->mtt_index);
        /* TODO: The vaddr and paddr mappings should be done when allocating
         */
        process->pTable->translate((Addr)args->vaddr[i], (Addr &)args->paddr[i]);
        DPRINTF(NicCtrl, " HGKFD_IOC_ALLOC_MTT: vaddr: 0x%lx, paddr: 0x%lx mtt_index %d\n",
                (uint64_t)args->vaddr[i], (uint64_t)args->paddr[i], args->mtt_index);
    }
    args->mtt_index -= (args->batch_size - 1);
}

void NicCtrl::writeMtt(TypedBufferArg<kfd_ioctl_init_mtt_args> &args) {

    // put mttResc into mailbox
    HanGuRnicDef::MttResc mttResc[MAX_MR_BATCH];
    for (uint32_t i = 0; i < args->batch_size; ++i) {
        mttResc[i].pAddr = args->paddr[i];
    }
    memcpy(mailbox.data, mttResc, sizeof(HanGuRnicDef::MttResc) * args->batch_size);

    postHcr((uint64_t)mailbox.addr, args->mtt_index, args->batch_size, HanGuRnicDef::WRITE_MTT);
}
/* -------------------------- MTT {end} ------------------------ */

/* -------------------------- MPT {begin} ------------------------ */
void NicCtrl::allocMpt(TypedBufferArg<kfd_ioctl_alloc_mpt_args> &args) {
    for (uint32_t i = 0; i < args->batch_size; ++i) {
        args->mpt_index = allocResc(HanGuRnicDef::ICMTYPE_MPT, mptMeta);
        DPRINTF(NicCtrl, " HGKFD_IOC_ALLOC_MPT: mpt_index %d batch_size %d\n", args->mpt_index, args->batch_size);
    }
    args->mpt_index -= (args->batch_size - 1);
}

void NicCtrl::writeMpt(TypedBufferArg<kfd_ioctl_write_mpt_args> &args) {
    // put MptResc into mailbox
    HanGuRnicDef::MptResc mptResc[MAX_MR_BATCH];
    for (uint32_t i = 0; i < args->batch_size; ++i) {
        mptResc[i].flag       = args->flag     [i];
        mptResc[i].key        = args->mpt_index[i];
        mptResc[i].length     = args->length   [i];
        mptResc[i].startVAddr = args->addr     [i];
        mptResc[i].mttSeg     = args->mtt_index[i];
        DPRINTF(NicCtrl, " HGKFD_IOC_WRITE_MPT: mpt_index %d(%d) mtt_index %d(%d) batch_size %d\n", 
                args->mpt_index[i], mptResc[i].key, args->mtt_index[i], mptResc[i].mttSeg, args->batch_size);
    }
    memcpy(mailbox.data, mptResc, sizeof(HanGuRnicDef::MptResc) * args->batch_size);

    postHcr((uint64_t)mailbox.addr, args->mpt_index[0], args->batch_size, HanGuRnicDef::WRITE_MPT);
}
/* -------------------------- MPT {end} ------------------------ */


/* -------------------------- CQC {begin} ------------------------ */
void NicCtrl::allocCqc(TypedBufferArg<kfd_ioctl_alloc_cq_args> &args) {
    args->cq_num = allocResc(HanGuRnicDef::ICMTYPE_CQC, cqcMeta);
}

void NicCtrl::writeCqc(TypedBufferArg<kfd_ioctl_write_cqc_args> &args) {
    /* put CqcResc into mailbox */
    HanGuRnicDef::CqcResc cqcResc;
    cqcResc.cqn    = args->cq_num  ;
    cqcResc.lkey   = args->lkey    ;
    cqcResc.offset = args->offset  ;
    cqcResc.sizeLog= args->size_log;
    memcpy(mailbox.data, &cqcResc, sizeof(HanGuRnicDef::CqcResc));
    postHcr((uint64_t)mailbox.addr, args->cq_num, 0, HanGuRnicDef::WRITE_CQC);
}
/* -------------------------- CQC {end} ------------------------ */


/* -------------------------- QPC {begin} ------------------------ */
void NicCtrl::allocQpc(TypedBufferArg<kfd_ioctl_alloc_qp_args> &args) {
    for (uint32_t i = 0; i < args->batch_size; ++i) {
        // DPRINTF(NicCtrl, " allocQpc: qpc_bitmap: 0x%x 0x%x 0x%x\n", qpcMeta.bitmap[0], qpcMeta.bitmap[1], qpcMeta.bitmap[2]);
        args->qp_num = allocResc(HanGuRnicDef::ICMTYPE_QPC, qpcMeta);
        DPRINTF(NicCtrl, " allocQpc: qpc_bitmap:  0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n"
                                                        " 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n"
                                                        " 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n"
                                                        " 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
                qpcMeta.bitmap[0], qpcMeta.bitmap[1], qpcMeta.bitmap[2], qpcMeta.bitmap[3], qpcMeta.bitmap[4], qpcMeta.bitmap[5], qpcMeta.bitmap[6], qpcMeta.bitmap[7],
                qpcMeta.bitmap[8], qpcMeta.bitmap[9], qpcMeta.bitmap[10], qpcMeta.bitmap[11], qpcMeta.bitmap[12], qpcMeta.bitmap[13], qpcMeta.bitmap[14], qpcMeta.bitmap[15],
                qpcMeta.bitmap[16], qpcMeta.bitmap[17], qpcMeta.bitmap[18], qpcMeta.bitmap[19], qpcMeta.bitmap[20], qpcMeta.bitmap[21], qpcMeta.bitmap[22], qpcMeta.bitmap[23],
                qpcMeta.bitmap[24], qpcMeta.bitmap[25], qpcMeta.bitmap[26], qpcMeta.bitmap[27], qpcMeta.bitmap[28], qpcMeta.bitmap[29], qpcMeta.bitmap[30], qpcMeta.bitmap[31]);
    }
    args->qp_num -= (args->batch_size - 1);
}

void NicCtrl::writeQpc(TypedBufferArg<kfd_ioctl_write_qpc_args> &args) {
    /* put QpcResc into mailbox */
    HanGuRnicDef::QpcResc qpcResc[MAX_QPC_BATCH]; // = (HanGuRnicDef::QpcResc *)mailbox.vaddr;
    memset(qpcResc, 0, sizeof(HanGuRnicDef::QpcResc) * args->batch_size);
    for (uint32_t i = 0; i < args->batch_size; ++i) {
        qpcResc[i].flag           = args->flag             [i];
        qpcResc[i].qpType         = args->type             [i];
        qpcResc[i].srcQpn         = args->src_qpn          [i];
        qpcResc[i].lLid           = args->llid             [i];
        qpcResc[i].cqn            = args->cq_num           [i];
        qpcResc[i].sndWqeBaseLkey = args->snd_wqe_base_lkey[i];
        qpcResc[i].sndWqeOffset   = args->snd_wqe_offset   [i];
        qpcResc[i].sqSizeLog      = args->sq_size_log      [i];
        qpcResc[i].rcvWqeBaseLkey = args->rcv_wqe_base_lkey[i];
        qpcResc[i].rcvWqeOffset   = args->rcv_wqe_offset   [i];
        qpcResc[i].rqSizeLog      = args->rq_size_log      [i];

        qpcResc[i].ackPsn  = args->ack_psn [i];
        qpcResc[i].sndPsn  = args->snd_psn [i];
        qpcResc[i].expPsn  = args->exp_psn [i];
        qpcResc[i].dLid    = args->dlid    [i];
        qpcResc[i].destQpn = args->dest_qpn[i];

        qpcResc[i].qkey    = args->qkey[i];

        DPRINTF(NicCtrl, " writeQpc: qpn: 0x%x\n", qpcResc[i].srcQpn);
    }
    DPRINTF(NicCtrl, " writeQpc: args->batch_size: %d\n", args->batch_size);
    memcpy(mailbox.data, qpcResc, sizeof(HanGuRnicDef::QpcResc) * args->batch_size);
    DPRINTF(NicCtrl, " writeQpc: args->batch_size1: %d\n", args->batch_size);
    postHcr((uint64_t)mailbox.addr, args->src_qpn[0], args->batch_size, HanGuRnicDef::WRITE_QPC);
}

/* -------------------------- QPC {end} ------------------------ */


/******************************* MemAllocator ***************************/

struct MemBlock MemAllocator::allocMem(size_t size) {
    Addr paddrStart, paddrEnd;
    Addr vaddrStart = (Addr) (new uint8_t[size]);
    Addr vaddrEnd = vaddrStart + size - 1;
    auto it = memMap.begin();

    for (it; it != memMap.end() - 1; it++) {
        if (memMap[it].paddr.end() + size < memMap[it+1].paddr.start()) {
            paddrStart = memMap[it].paddr.end() + 1;
            paddrEnd = paddrStart + size - 1;
            break;
        }
    }
    MemBlock memBlock = {
        .paddr = AddrRange(paddrStart, paddrEnd),
        .vaddr = AddrRange(vaddrStart, vaddrEnd)
    }
    memMap.insert(it+1, memBlock);

    return memBlock;
}

void MemAllocator::destroyMem(MemBlock memBlock) {
    for (auto it = memMap.begin(); it != memMap.end() - 1; it++) {
        if (memMap[it] == memBlock) {
            memMap.erase(it);
        }
    }
    return;
}

MemBlock MemAllocator::getPhyBlock(Addr paddr) {
    for (auto it = memMap.begin(); it != memMap.end() - 1; it++) {
        if (memMap[it].paddr.contains(paddr)) {
            return memMap[it];
        }
    }
    return NULL;
}

MemBlock MemAllocator::getVirBlock(Addr vaddr) {
    for (auto it = memMap.begin(); it != memMap.end() - 1; it++) {
        if (memMap[it].vaddr.contains(vaddr)) {
            return memMap[it];
        }
    }
    return NULL;
}

Addr MemAllocator::getPhyAddr(Addr vaddr) {
    for (auto it = memMap.begin(); it != memMap.end() - 1; it++) {
        if (memMap[it].vaddr.contains(vaddr)) {
            uint64_t idx = vaddr - memMap[it].vaddr.start();
            Addr paddr = memMap[it].paddr.start() + idx;
        }
    }
    return paddr;
}

Addr MemAllocator::getVirAddr(Addr paddr) {
    for (auto it = memMap.begin(); it != memMap.end() - 1; it++) {
        if (memMap[it].paddr.contains(paddr)) {
            uint64_t idx = paddr - memMap[it].paddr.start();
            Addr vaddr = memMap[it].vaddr.start() + idx;
        }
    }
    return vaddr;
}

/******************************* MemAllocator ***************************/
