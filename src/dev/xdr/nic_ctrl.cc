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
#include <memory>
#include <queue>

#include "base/addr_range.hh"
#include "base/inet.hh"
#include "base/trace.hh"
#include "base/random.hh"
#include "debug/Drain.hh"
#include "dev/net/etherpkt.hh"
#include "debug/XDR.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/NicCtrl.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

using namespace HanGuRnicDef;
using namespace Net;
using namespace std;

NicCtrl::NicCtrl(const Params *p)
	: rnic(&p->rnic),
	dmaReadDelay(p->dma_read_delay),
	dmaWriteDelay(p->dma_write_delay),
	pciBandwidth(p->pci_speed),
	etherBandwidth(p->ether_speed),
	LinkDelay     (p->link_delay),
	ethRxPktProcEvent([this]{ ethRxPktProc(); }, name()) {

	DPRINTF(NicCtrl, " qpc_cache_cap %d  reorder_cap %d cpuNum 0x%x\n", p->qpc_cache_cap, p->reorder_cap, p->cpu_num);

	mboxBuf = new uint8_t[4096];

	BARSize[0]  = (1 << 12);
	BARAddrs[0] = 0xc000000000000000;

	AddrRangeList addr_list = nic->getAddrRanges();
	AddrRange bar0 = addr_list.pop_front(); // Get BAR0
	hcrAddr = bar0.start();

	doorBell = hcrAddr + 0x18;
} // NicCtrl::NicCtrl

NicCtrl::~NicCtrl() {
}

void NicCtrl::init() {
	PciDevice::init();
}

///////////////////////////// NicCtrl::PIO relevant {begin}//////////////////////////////

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


Tick
HanGuRnic::read(PacketPtr pkt) {
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

Tick
HanGuRnic::write(PacketPtr pkt) {
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
		DPRINTF(PioEngine, " PioEngine.write: HCR, inparam: 0x%x\n", pkt->getLE<Hcr>().inParam_l);

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
		DPRINTF(HanGuRnic, " PioEngine.write: Doorbell, value %#X pio interval %ld\n", pkt->getLE<uint64_t>(), curTick() - this->tick); 

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

		DPRINTF(HanGuRnic, " PioEngine.write: qpn %d, opcode %x, num %d\n", 
				regs.db.qpn(), regs.db.opcode(), regs.db.num());
	} else if (daddr == 0x20 && pkt->getSize() == sizeof(uint32_t)) { /* latency sync */

		DPRINTF(HanGuRnic, " PioEngine.write: sync bit, value %#X, syncCnt %d\n", pkt->getLE<uint32_t>(), syncCnt); 

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

		DPRINTF(HanGuRnic, " PioEngine.write: sync bit end, value %#X, syncCnt %d\n", pkt->getLE<uint32_t>(), syncCnt); 
	} else {
		panic("Write request to unknown address : %#x && size 0x%x\n", daddr, pkt->getSize());
	}

	pkt->makeAtomicResponse();
	return pioDelay;
}
///////////////////////////// NicCtrl::PIO relevant {end}//////////////////////////////


///////////////////////////// NicCtrl::HCR relevant {begin}//////////////////////////////

uint8_t NicCtrl::checkHcr() {

	uint32_t goOp;
	// DPRINTF(NicCtrl, " Start read `GO`.\n");
	rnic->dmaRead(hcrAddr + (Addr)&(((HanGuRnicDef::Hcr*)0)->goOpcode), sizeof(goOp), nullptr, &goOp);

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
    // DPRINTF(NicCtrl, " inParam_l: 0x%x\n", hcr.inParam_l);
    // DPRINTF(NicCtrl, " inParam_h: 0x%x\n", hcr.inParam_h);
    // DPRINTF(NicCtrl, " inMod: 0x%x\n", hcr.inMod);
    // DPRINTF(NicCtrl, " outParam_l: 0x%x\n", hcr.outParam_l);
    // DPRINTF(NicCtrl, " outParam_h: 0x%x\n", hcr.outParam_h);
    // DPRINTF(NicCtrl, " goOpcode: 0x%x\n", hcr.goOpcode);

    rnic->dmaWrite(hcrAddr, sizeof(hcr), nullptr, &hcr);
}

///////////////////////////// NicCtrl::HCR relevant {end}//////////////////////////////


HanGuRnic *
HanGuRnicParams::create() {
	return new HanGuRnic(this);
}
