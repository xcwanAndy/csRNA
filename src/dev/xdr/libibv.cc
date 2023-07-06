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
        memAlloc = nicCtrl->getMemAlloc();
    }

Ibv::~Ibv(){}

void Ibv::wait(uint32_t n) {
    for (uint32_t i = 0; i < n; ++i);
}


uint8_t Ibv::write_cmd(unsigned long request, void *args) {
    nicCtrl->nicCtrlReq = request;
    nicCtrl->ioc_buf = args;
    if (!nicCtrl->nicCtrlEvent.scheduled()) {
        nicCtrl->schedule(nicCtrl->nicCtrlEvent, curTick() + nicCtrl->clockPeriod());
    }
    //while (nicCtrl(request, (void *)args)) {
        //DPRINTF(Ibv, " %ld ioctl failed try again\n", request);
        //wait(SLEEP_CNT);
        //// usleep(1);
    //}
    //do {
        //wait(SLEEP_CNT);
    //} while (nicCtrl(fd, HGKFD_IOC_CHECK_GO, NULL));

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
    write_cmd(HGKFD_IOC_INIT_DEV, (void *)args);
    free(args);

    /* Init communication management */
    struct ibv_mr_init_attr mr_attr;
    mr_attr.length = PAGE_SIZE;
    mr_attr.flag = MR_FLAG_RD | MR_FLAG_WR | MR_FLAG_LOCAL;
    context->cm_mr = ibv_reg_mr(context, &mr_attr);

    struct ibv_cq_init_attr cq_attr;
    cq_attr.size_log = PAGE_SIZE_LOG;
    context->cm_cq = ibv_create_cq(context, &cq_attr);
    DPRINTF(Ibv, " ibv_open_device cq lkey: 0x%x, vaddr 0x%lx, mtt_index 0x%x, paddr 0x%lx\n", 
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

    DPRINTF(Ibv, " Exit ibv_open_device: out!\n");
    return 0;
}

struct ibv_mr* Ibv::ibv_reg_mr(struct ibv_context *context, struct ibv_mr_init_attr *mr_attr) {
    DPRINTF(Ibv, " ibv_reg_mr!\n");
    struct hghca_context *dvr = (struct hghca_context *)context->dvr;
    struct ibv_mr *mr =  (struct ibv_mr *)malloc(sizeof(struct ibv_mr));

    /* Calc needed number of pages */
    mr->num_mtt = (mr_attr->length >> 12) + (mr_attr->length & 0xFFF) ? 1 : 0;
    /* the num_mtt is always 1 */
    assert(mr->num_mtt == 1);

    /* !TODO: Now, we require allocated memory's start
     * vaddr is at the boundry of one page */
    //mr->addr   = memalign(PAGE_SIZE, mr_attr->length);
    /* This address will be translated into phys addr */
    mr->addr = (uint8_t *)memAlloc->allocMem(mr_attr->length).vaddr.start();
    memset(mr->addr, 0, mr_attr->length);
    mr->ctx = context;
    mr->flag   = mr_attr->flag;
    mr->length = mr_attr->length;
    mr->mtt    = (struct ibv_mtt *)malloc(sizeof(struct ibv_mtt) * mr->num_mtt);
    for (uint64_t i = 0; i < mr->num_mtt; ++i) {
        mr->mtt[i].vaddr = (void *)(mr->addr + (i << PAGE_SIZE_LOG));

        /* Init (Allocate and write) MTT */
        struct kfd_ioctl_init_mtt_args *mtt_args =
                (struct kfd_ioctl_init_mtt_args *)malloc(sizeof(struct kfd_ioctl_init_mtt_args));
        mtt_args->batch_size = 1;
        mtt_args->vaddr[0] = (uint8_t *)mr->mtt[i].vaddr;
        /* Translate virtual addr into physical addr */
        mtt_args->paddr[0] = memAlloc->getPhyAddr((Addr)mtt_args->vaddr[0]);
        write_cmd(HGKFD_IOC_ALLOC_MTT, (void *)mtt_args);
        mr->mtt[i].mtt_index = mtt_args->mtt_index;
        mr->mtt[i].paddr = mtt_args->paddr[0];
        write_cmd(HGKFD_IOC_WRITE_MTT, (void *)mtt_args);
        free(mtt_args);
    }

    /* Allocate MPT */
    struct kfd_ioctl_alloc_mpt_args *mpt_alloc_args =
            (struct kfd_ioctl_alloc_mpt_args *)malloc(sizeof(struct kfd_ioctl_alloc_mpt_args));
    mpt_alloc_args->batch_size = 1;
    write_cmd(HGKFD_IOC_ALLOC_MPT, (void *)mpt_alloc_args);
    mr->lkey = mpt_alloc_args->mpt_index;
    free(mpt_alloc_args);

    /* Write MPT */
    struct kfd_ioctl_write_mpt_args *mpt_args =
            (struct kfd_ioctl_write_mpt_args *)malloc(sizeof(struct kfd_ioctl_write_mpt_args));
    mpt_args->batch_size = 1;
    mpt_args->flag[0]      = mr->flag;
    mpt_args->addr[0]      = (uint64_t) mr->addr;
    mpt_args->length[0]    = mr->length;
    mpt_args->mtt_index[0] = mr->mtt[0].mtt_index;
    mpt_args->mpt_index[0] = mr->lkey;
    write_cmd(HGKFD_IOC_WRITE_MPT, (void *)mpt_args);
    free(mpt_args);

    DPRINTF(Ibv, " ibv_reg_mr: out!\n");
    return mr;
}

struct ibv_cq* Ibv::ibv_create_cq(struct ibv_context *context, struct ibv_cq_init_attr *cq_attr) {

    DPRINTF(Ibv, " enter ibv_create_cq!\n");
    struct hghca_context *dvr = (struct hghca_context *)context->dvr;
    struct ibv_cq *cq = (struct ibv_cq *)malloc(sizeof(struct ibv_cq));

    /* Allocate CQ */
    struct kfd_ioctl_alloc_cq_args *create_cq_args =
            (struct kfd_ioctl_alloc_cq_args *)malloc(sizeof(struct kfd_ioctl_alloc_cq_args));
    write_cmd(HGKFD_IOC_ALLOC_CQ, (void *)create_cq_args);
    cq->cq_num  = create_cq_args->cq_num;
    cq->ctx     = context;
    cq->offset  = 0;
    cq->cpl_cnt = 0;
    free(create_cq_args);

    /* Init (Allocate and write) MTT && MPT */
    struct ibv_mr_init_attr *mr_attr =
            (struct ibv_mr_init_attr *)malloc(sizeof(struct ibv_mr_init_attr));
    mr_attr->flag   = MR_FLAG_RD | MR_FLAG_LOCAL;
    mr_attr->length = (1 << cq_attr->size_log); // (PAGE_SIZE << 2); // !TODO: Now the size is a fixed number of 1 page
    cq->mr = ibv_reg_mr(context, mr_attr);
    free(mr_attr);

    /* write CQC */
    struct kfd_ioctl_write_cqc_args *write_cqc_args =
            (struct kfd_ioctl_write_cqc_args *)malloc(sizeof(struct kfd_ioctl_write_cqc_args));
    write_cqc_args->cq_num   = cq->cq_num;
    write_cqc_args->offset   = cq->offset;
    write_cqc_args->lkey     = cq->mr->lkey;
    write_cqc_args->size_log = PAGE_SIZE_LOG;
    write_cmd(HGKFD_IOC_WRITE_CQC, (void *)write_cqc_args);
    free(write_cqc_args);
    return cq;
}

