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

#include "dev/xdr/libibv.hh"

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
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/NicCtrl.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

#define SLEEP_CNT 1000

Ibv::Ibv(const Params *p)
    : SimObject(p),
    nicCtrl(&p->nicCtrl){
        DPRINTF(Ibv, " Initializing Ibv");
    }

Ibv::~Ibv(){}

void Ibv::wait(uint32_t n) {
    for (uint32_t i = 0; i < n; ++i);
}


uint8_t Ibv::write_cmd(unsigned long request, void *args) {
    nicCtrl->nicCtrlReq = request;
    if (!nicCtrl->nicCtrlEvent.scheduled()) {
        nicCtrl->schedule(nicCtrl->nicCtrlEvent, curTick() + nicCtrl->clockPeriod());
    }
    while (nicCtrl(request, (void *)args)) {
        DPRINTF(Ibv, " %ld ioctl failed try again\n", request);
        wait(SLEEP_CNT);
        // usleep(1);
    }
    do {
        wait(SLEEP_CNT);
    } while (nicCtrl(fd, HGKFD_IOC_CHECK_GO, NULL));

    return 0;
}


int Ibv::ibv_open_device(struct ibv_context *context, uint16_t lid) {

    context->lid = lid;

    /* Init fd */
    context->dvr = malloc(sizeof(struct hghca_context));
    struct hghca_context *dvr = (struct hghca_context *)context->dvr;

    /* get doorbell from nic controller */
    dvr->doorbell = nicCtrl->getDoorbell();
    dvr->sync = (Addr)((uint64_t)dvr->doorbell + 8);
    DRPINTF(Ibv, " get dvr->doorbell 0x%lx\n", (uint64_t)dvr->doorbell);

    /* Init ICM */
    struct kfd_ioctl_init_dev_args *args = 
            (struct kfd_ioctl_init_dev_args *)malloc(sizeof(struct kfd_ioctl_init_dev_args));
    args->qpc_num_log = 16; /* useless here */
    args->cqc_num_log = 16; /* useless here */
    args->mpt_num_log = 19; /* useless here */
    args->mtt_num_log = 19; /* useless here */
    write_cmd(dvr->fd, HGKFD_IOC_INIT_DEV, (void *)args);
    free(args);

    /* Init communication management */
    struct ibv_mr_init_attr mr_attr;
    mr_attr.length = PAGE_SIZE;
    mr_attr.flag = MR_FLAG_RD | MR_FLAG_WR | MR_FLAG_LOCAL;
    context->cm_mr = ibv_reg_mr(context, &mr_attr);

    struct ibv_cq_init_attr cq_attr;
    cq_attr.size_log = PAGE_SIZE_LOG;
    context->cm_cq = ibv_create_cq(context, &cq_attr);
    HGRNIC_PRINT(" ibv_open_device cq lkey: 0x%x, vaddr 0x%lx, mtt_index 0x%x, paddr 0x%lx\n", 
            context->cm_cq->mr->lkey, (uint64_t)context->cm_cq->mr->addr, context->cm_cq->mr->mtt->mtt_index, context->cm_cq->mr->mtt->paddr);

    struct ibv_qp_create_attr qp_attr;
    qp_attr.sq_size_log = PAGE_SIZE_LOG;
    qp_attr.rq_size_log = PAGE_SIZE_LOG;
    context->cm_qp = ibv_create_qp(context, &qp_attr);

    context->cm_qp->ctx = context;
    context->cm_qp->type = QP_TYPE_UD;
    context->cm_qp->cq = context->cm_cq;
    context->cm_qp->snd_wqe_offset = 0;
    context->cm_qp->rcv_wqe_offset = 0;
    context->cm_qp->lsubnet.llid = context->lid;
    context->cm_qp->qkey = QKEY_CM;
    ibv_modify_qp(context, context->cm_qp);

    context->cm_rcv_posted_off = RCV_WR_BASE;
    context->cm_rcv_acked_off  = RCV_WR_BASE;
    context->cm_snd_off        = SND_WR_BASE;
    context->cm_rcv_num        = 0;

    HGRNIC_PRINT(" Exit ibv_open_device: out!\n");
    return 0;
}

