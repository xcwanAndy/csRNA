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

#include "dev/rdma/hangu_rnic_defs.hh"
#include "dev/rdma/hangu_rnic.hh"

#include "base/inet.hh"
#include "debug/EthernetDesc.hh"
#include "debug/EthernetIntr.hh"
#include "dev/net/etherdevice.hh"
#include "dev/net/etherint.hh"
#include "dev/net/etherpkt.hh"
#include "dev/net/pktfifo.hh"
#include "dev/pci/device.hh"
#include "params/NicCtrl.hh"

using namespace HanGuRnicDef;

class NicCtrl {
    private:
        HanGuRnic *rnic;
        Addr hcrAddr;
        Addr doorBell;

    public:
        typedef NicCtrlParams Params;
        const Params *params() const {
                return dynamic_cast<const Params *>(_params);
            }

        NicCtrl(const Params *params);
        ~NicCtrl();
        void init() override;

        // PIO Interface
        Tick writeConfig(PacketPtr pkt) override;
        Tick read(PacketPtr pkt) override;
        Tick write(PacketPtr pkt) override;

        // Addr mailbox;
        struct Mailbox {
            Addr addr;
            uint8_t *data;
        };
        Mailbox mailbox;
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

        // Driver functions
        uint8_t checkHcr(PortProxy& portProxy);
        void postHcr(uint64_t inParam,
            uint32_t inMod, uint64_t outParam, uint8_t opcode);

        std::unordered_map<Addr, Addr> icmAddrmap; // <icm vaddr page, icm paddr>
        void initIcm(uint8_t qpcNumLog, uint8_t cqcNumLog,
            uint8_t mptNumLog, uint8_t mttNumLog);
        uint8_t isIcmMapped(RescMeta &rescMeta, Addr index);
        Addr allocIcm(RescMeta &rescMeta, Addr index);
        void writeIcm(uint8_t rescType, RescMeta &rescMeta, Addr icmVPage);

        /* related to link delay processing */
        Tick LinkDelay;
};

#endif //__NIC_CTRL_HH__
