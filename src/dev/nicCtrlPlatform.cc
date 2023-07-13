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


/** @file
 * Implementation of platform with rdma nic.
 */

#include "dev/nicCtrlPlatform.hh"

#include <deque>
#include <string>
#include <vector>

#include "sim/system.hh"

NicCtrlPlatform::NicCtrlPlatform(const Params *p)
    : Platform(p), system(p->system)
{

}

void NicCtrlPlatform::init() {

}

void NicCtrlPlatform::postConsoleInt()
{
    // southBridge->ioApic->signalInterrupt(4);
    // southBridge->pic1->signalInterrupt(4);
}

void NicCtrlPlatform::clearConsoleInt()
{
    warn_once("Don't know what interrupt to clear for console.\n");
    //panic("Need implementation\n");
}

void NicCtrlPlatform::postPciInt(int line)
{
    // southBridge->ioApic->signalInterrupt(line);
}

void NicCtrlPlatform::clearPciInt(int line)
{
    warn_once("Tried to clear PCI interrupt %d\n", line);
}

NicCtrlPlatform* NicCtrlPlatformParams::create()
{
    return new NicCtrlPlatform(this);
}
