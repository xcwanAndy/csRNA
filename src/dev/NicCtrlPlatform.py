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
#   Primary Author: Kang Ning
#       <kangning18z@ict.ac.cn>
#   Date: 2021.07.08


from m5.params import *
from m5.proxy import *

from m5.objects.Xdr import NicCtrl
from m5.objects.Platform import Platform
from m5.objects.PciHost import GenericPciHost


class NicCtrlPciHost(GenericPciHost):
    conf_base = 0xE000000000000000
    conf_size = "16MB"
    pci_pio_base = 0x9000000000000000

class NicCtrlPlatform(Platform):
    type = 'NicCtrlPlatform'
    cxx_header = "dev/nicCtrlPlatform.hh"
    system = Param.System(Parent.any, "system")

    pci_host = NicCtrlPciHost()

    nic_ctrl = NicCtrl(pci_bus=1, pci_dev=1, pci_func=1)

    def attachIO(self, bus):
        self.pci_host.pio = bus.mem_side_ports
        self.nic_ctrl.pio = bus.mem_side_ports
        self.nic_ctrl.dma = bus.cpu_side_ports
