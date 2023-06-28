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

    public:
        typedef NicCtrlParams Params;
        const Params *
            params() const {
                return dynamic_cast<const Params *>(_params);
            }

        NicCtrl(const Params *params);
        ~NicCtrl();
        void init() override;

        // PIO Interface
        Tick writeConfig(PacketPtr pkt) override;
        Tick read(PacketPtr pkt) override;
        Tick write(PacketPtr pkt) override;

        // Driver functions
        uint8_t NicCtrl::checkHcr(PortProxy& portProxy);

        /* related to link delay processing */
        Tick LinkDelay;
        std::queue<std::pair<EthPacketPtr, Tick>> ethRxDelayFifo;

        void ethRxPktProc(); // When rx packet
        EventFunctionWrapper ethRxPktProcEvent;

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;

        DrainState drain() override;
        void drainResume() override;

};

#endif //__NIC_CTRL_HH__
