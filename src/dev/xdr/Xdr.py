#
# ======================= START OF LICENSE NOTICE =======================
#   Copyright (C) 2021 Kang Ning, NCIC, ICT, CAS.
#   All Rights Reserved.
#
#   NO WARRANTY. THE PRODUCT IS PROVIDED BY DEVELOPER "AS IS" AND ANY
#   EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
#   PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL DEVELOPER BE LIABLE FOR
#   ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
#   GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
#   IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
#   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THE PRODUCT, EVEN
#   IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ======================== END OF LICENSE NOTICE ========================
#   Primary Author: Kang Ning
#   <kangning18z@ict.ac.cn>
#

from m5.defines import buildEnv
from m5.SimObject import SimObject
from m5.params import *
from m5.proxy import *
from m5.objects.PciDevice import PciDevice
from m5.objects.Ethernet import *
from m5.objects.Process import EmulatedDriver
from m5.objects.Rnic import HanGuRnic


class NicCtrl(PciDevice):
    type = 'NicCtrl'
    cxx_header = "dev/xdr/nic_ctrl.hh"

    interface = EtherInt("Ethernet Interface")

    # rnic = HanGuRnic(pci_bus=0, pci_dev=0, pci_func=0)
    # rnic as a parameter
    rnic = Param.HanGuRnic("HanGu Rnic")

    # offpath
    is_onpath = Param.Bool(True, "If onpath or not")

    base_addr = Param.Addr(0xD000000000000000,
                          "Base Addr")

    mpt_cache_num = Param.Int(40000,
        "Number of mpt cache enteries")
    mtt_cache_num = Param.Int(50000,
        "Number of mtt cache enteries")
    qpc_cache_cap = Param.Int(100,
        "Number of qpc cache enteries")
    cqc_cache_num = Param.Int(2000,
        "Number of cqc cache enteries")

    VendorID = 0x8086
    DeviceID = 0x1075
    SubsystemID = 0x1008
    SubsystemVendorID = 0x8086
    Status = 0x0000
    SubClassCode = 0x00
    ClassCode = 0x02
    ProgIF = 0x00
    BAR0 = 0x00000000
    BAR1 = 0x00000000
    BAR2 = 0x00000000
    BAR3 = 0x00000000
    BAR4 = 0x00000000
    BAR5 = 0x00000000
    MaximumLatency = 0x00
    MinimumGrant = 0xff
    InterruptLine = 0x1e
    InterruptPin = 0x01
    BAR0Size = '128kB'

    dma_read_delay = Param.Latency('500ns', "delay after desc fetch occurs")
    dma_write_delay = Param.Latency('250ns', "delay after desc wb occurs")

    pci_speed = Param.NetworkBandwidth('128Gbps', "pci speed in bits per second")
    ether_speed = Param.NetworkBandwidth('100Gbps',
                                         "NIC speed in bits per second")

    reorder_cap = Param.Int(100,
                    "Number of concurrent request for one qpc req channel")

    link_delay = Param.Latency('1us', "ethernet link delay")
    cpu_num    = Param.Int(10, "Number of CPUs in this node")

class Ibv(SimObject):
    type = 'Ibv'
    cxx_header = "dev/xdr/libibv.hh"
    nic_ctrl = Param.NicCtrl("Rnic Controller")
    is_onpath = Param.Bool(True, "If onpath")

class Accel(SimObject):
    type = 'Accel'
    cxx_header = "dev/xdr/accel.hh"
    ibv = Param.Ibv("IB Verbs")
    is_onpath = Param.Bool(True, "If onpath")

class IbvTestClient(Accel):
    type = 'IbvTestClient'
    cxx_header = "dev/xdr/ibv_test_client.hh"

class IbvTestServer(Accel):
    type = 'IbvTestServer'
    cxx_header = "dev/xdr/ibv_test_server.hh"

class AccelDriver(EmulatedDriver):
    type = 'AccelDriver'
    # abstract = True
    cxx_header = 'dev/xdr/accel_driver.hh'
    accel = Param.Accel('Accelerater controlled by this driver')
