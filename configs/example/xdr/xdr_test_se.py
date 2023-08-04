#
# ====================== START OF LICENSE NOTICE =======================
#  Copyright (C) 2021 Kang Ning, NCIC, ICT, CAS.
#  All Rights Reserved.
#
#  NO WARRANTY. THE PRODUCT IS PROVIDED BY DEVELOPER "AS IS" AND ANY
#  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
#  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL DEVELOPER BE LIABLE FOR
#  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
#  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
#  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
#  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THE PRODUCT, EVEN
#  IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ======================= END OF LICENSE NOTICE ========================
#  Primary Author: Kang Ning
#  <kangning18z@ict.ac.cn>
#


from __future__ import print_function
from __future__ import absolute_import

import optparse
import sys
import os

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.params import NULL
from m5.util import addToPath, fatal, warn

from m5.objects.PciHost import *
from m5.objects.Rnic import HanGuRnic, HanGuDriver
from m5.objects.Xdr import NicCtrl, Ibv, IbvTestClient, IbvTestServer, AccelDriver, Accel


addToPath('../../')

from common import Options
from common import Simulation
from common import CacheConfig
from common import CpuConfig
from common import ObjectList
from common import MemConfig
from common.FileSystemConfig import config_filesystem
from common.Caches import *
from common.cpu2000 import *

CLIENT = "tests/test-progs/nic-ctrl/bin/client"
SERVER = "tests/test-progs/nic-ctrl/bin/server"


# def config_rnic(system, options, node_num):


def make_nic_system(options, CPUClass, test_mem_mode, is_client=False):
    system = System(cpu = [CPUClass(socket_id=i, cpu_id=i,
                                    numThreads=1, syscallRetryLatency=200)
                           for i in range(0,1)],
                    mem_mode = test_mem_mode,
                    mem_ranges = [AddrRange(options.mem_size)],
                    # cache_line_size = options.cacheline_size,
                    workload = NULL)

    # Create a top-level voltage domain
    system.voltage_domain = VoltageDomain(voltage = options.sys_voltage)

    # Create a source clock for the system and set the clock period
    system.clk_domain = SrcClockDomain(clock =  options.sys_clock,
                                    voltage_domain = system.voltage_domain)

    # Create a CPU voltage domain
    system.cpu_voltage_domain = VoltageDomain()

    # Create a separate clock domain for the CPUs
    system.cpu_clk_domain = SrcClockDomain(clock = options.cpu_clock,
                                        voltage_domain =
                                        system.cpu_voltage_domain)

    # All cpus belong to a common cpu_clk_domain, therefore running at a common
    # frequency.
    for cpu in system.cpu:
        cpu.clk_domain = system.cpu_clk_domain

    MemClass = Simulation.setMemClass(options)
    system.membus = SystemXBar()
    system.system_port = system.membus.cpu_side_ports
    CacheConfig.config_cache(options, system)
    MemConfig.config_mem(options, system)
    config_filesystem(system, options)

    # Configure RDMA NIC
    # config_rnic(system, options, node_num)
    """
    Add rdma nic to the system.
    """
    # North Bridge
    system.iobus  = IOXBar()
    system.iobridge = Bridge(delay='250ns')
    system.iobridge.master = system.iobus.slave
    system.iobridge.slave  = system.membus.master

    # Allow the bridge to pass through:
    #  1) kernel configured PCI device memory map address: address range
    #     [0xC0000000, 0xFFFF0000). (The upper 64kB are reserved for m5ops.)
    #  2) the bridge to pass through the IO APIC (two pages, already contained in 1),
    #  3) everything in the IO address range up to the local APIC, and
    #  4) then the entire PCI address space and beyond.
    IO_address_space_base = 0x8000000000000000
    pci_config_address_space_base = 0xc000000000000000
    interrupts_address_space_base = 0xa000000000000000
    system.iobridge.ranges = \
        [
        AddrRange(0xC0000000, 0xFFFF0000),
        AddrRange(IO_address_space_base,
                  interrupts_address_space_base - 1),
        AddrRange(pci_config_address_space_base,
                  Addr.max)
        ]
    system.dmabridge = Bridge(delay='250ns',
                       ranges = system.mem_ranges)
    system.dmabridge.slave = system.iobus.master
    system.dmabridge.master = system.membus.slave

    if (is_client):
        mac_addr = 0x11
        cmd = CLIENT
    else:
        mac_addr = 0x22
        cmd = SERVER

    system.accel = Accel()

    system.ibv = Ibv()
    system.accel.ibv = system.ibv

    # RNIC Platform
    system.nic_platform = RnicPlatform()
    system.nic_platform.rdma_nic.mac_addr = mac_addr
    system.nic_platform.rdma_nic.pci_speed     = options.pci_linkspeed
    system.nic_platform.rdma_nic.ether_speed   = options.ethernet_linkspeed
    system.nic_platform.rdma_nic.qpc_cache_cap = options.qpc_cache_cap
    system.nic_platform.rdma_nic.reorder_cap   = options.reorder_cap
    system.nic_platform.rdma_nic.cpu_num       = options.num_cpus
    # Attach RNIC to iobus
    system.nic_platform.attachIO(system.iobus)

    accel_driver = AccelDriver(filename="accel")
    accel_driver.accel = system.accel

    system.intrctrl = IntrControl()

    # NIC Controller Platform
    system.nic_platform.nic_ctrl.rnic = system.nic_platform.rdma_nic
    system.nic_platform.nic_ctrl.pci_speed = options.pci_linkspeed

    system.ibv.nic_ctrl = system.nic_platform.nic_ctrl
    # system.nic_platform.nic_ctrl.accel = system.accel

    # Set process
    process = Process(pid = 100)
    process.executable = cmd
    process.cwd = os.getcwd()
    process.cmd = [cmd]
    process.drivers = [accel_driver]
    system.cpu[0].workload = process
    system.cpu[0].createThreads()

    return system


def make_root_system(client, server, en_bw):
    self = Root(full_system = False)
    self.svrsys = server
    self.cltsys = client

    self.etherswitch = EtherSwitch()
    self.etherswitch.fabric_speed = en_bw
    self.etherswitch.delay = "0us" # We don't use the delay mechanism in etherswitch
    self.etherswitch.time_to_live = "100s" # don't consider MAC addr mapping TTL

    self.etherswitch.interface = client.nic_platform.rdma_nic.interface
    self.etherswitch.interface = server.nic_platform.rdma_nic.interface

    return self

def get_hangu_rnic_options():
    parser = optparse.OptionParser()
    Options.addCommonOptions(parser)
    Options.addSEOptions(parser)
    parser.add_option("--node-num", type="int", action="store", default=2,
                      help="""The options to pass to the binary, use " "
                              around the entire string""")
    parser.add_option("--pci-linkspeed", default="10Gbps",
                        action="store", type="string",
                        help="Link speed in bps\nDEFAULT: 10Gbps")
    parser.add_option("--qpc-cache-cap", default=50,
                        action="store", type="int",
                        help="capacity of qpc cache\nDEFAULT: 50 entries")
    parser.add_option("--reorder-cap", default=100,
                        action="store", type="int",
                        help="capacity of qpc cache\nDEFAULT: 50 entries")

    (options, args) = parser.parse_args()

    if args:
        print("Error: script doesn't take any positional arguments")
        sys.exit(1)

    print(options)
    return options


def main():
    # Add options
    options = get_hangu_rnic_options()

    (CPUClass, test_mem_mode, FutureClass) = Simulation.setCPUClass(options)

    client_system = make_nic_system(options, CPUClass, test_mem_mode, True)

    server_system = make_nic_system(options, CPUClass, test_mem_mode, False)

    # for cpu in system.cpu:
        # cpu.wait_for_remote_gdb = True

    root = make_root_system(client_system, server_system, options.ethernet_linkspeed)

    Simulation.run(options, root, server_system, FutureClass)

if __name__ == "__m5_main__":
    main()
