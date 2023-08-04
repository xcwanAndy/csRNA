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
 *  Date: 2021.07.08
 */

/**
 * @file
 * The AccelDriver implements an RnicDriver for an RDMA NIC
 * agent.
 */

#ifndef __ACCEL_DRIVER_HH__
#define __ACCEL_DRIVER_HH__

#include "dev/xdr/kfd_ioctl.hh"
#include "base/time.hh"

#include "dev/xdr/accel.hh"
#include "debug/XDR.hh"
#include "params/AccelDriver.hh"

#include "sim/proxy_ptr.hh"
#include "sim/process.hh"
#include "sim/system.hh"
#include "cpu/thread_context.hh"
#include "sim/syscall_emul_buf.hh"

#include "base/types.hh"
#include "sim/emul_driver.hh"


class PortProxy;
class ThreadContext;

//struct AccelDriverParams;

class AccelDriver final : public EmulatedDriver {
  public:
    typedef AccelDriverParams Params;
    AccelDriver(Params *p);

    int open(ThreadContext *tc, int mode, int flags);
    int ioctl(ThreadContext *tc, unsigned req, Addr ioc_buf) override;
    Addr mmap(ThreadContext *tc, Addr start, uint64_t length,
              int prot, int tgtFlags, int tgtFd, int offset);

  protected:
    Accel *accel;

    Addr hcrAddr;

    void configDevice();

  private:

    /* -------CPU_ID{begin}------- */
    uint8_t cpu_id;
    /* -------CPU_ID{end}------- */

    /* -------HCR {begin}------- */
    uint8_t checkHcr(PortProxy& portProxy);

    void postHcr(PortProxy& portProxy, 
            uint64_t inParam, uint32_t inMod, uint64_t outParam, uint8_t opcode);
    /* -------HCR {end}------- */
};

#endif // __ACCEL_DRIVER_HH__
