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

struct ibv_mr* Ibv::ibv_reg_batch_mr(struct ibv_context *context, struct ibv_mr_init_attr *mr_attr, uint32_t batch_size) {
    DPRINTF(Ibv, " ibv_reg_batch_mr!\n");
    struct hghca_context *dvr = (struct hghca_context *)context->dvr;
    struct ibv_mr *mr =  (struct ibv_mr *)malloc(sizeof(struct ibv_mr) * batch_size);

    uint32_t batch_cnt = 0;
    uint32_t batch_left = batch_size;
    struct kfd_ioctl_init_mtt_args *mtt_args =
                (struct kfd_ioctl_init_mtt_args *)malloc(sizeof(struct kfd_ioctl_init_mtt_args));
    struct kfd_ioctl_alloc_mpt_args *mpt_alloc_args =
                (struct kfd_ioctl_alloc_mpt_args *)malloc(sizeof(struct kfd_ioctl_alloc_mpt_args));
    struct kfd_ioctl_write_mpt_args *mpt_args =
                (struct kfd_ioctl_write_mpt_args *)malloc(sizeof(struct kfd_ioctl_write_mpt_args));
    while (batch_left > 0) {
        uint32_t sub_bsz = 0;
        sub_bsz = (batch_left > MAX_MR_BATCH) ? MAX_MR_BATCH : batch_left;

        /* Init (Allocate and write) MTT */
        for (uint32_t i = 0; i < sub_bsz; ++i) {
            /* Calc needed number of pages */
            mr[batch_cnt + i].num_mtt = (mr_attr->length >> 12) + (mr_attr->length & 0xFFF) ? 1 : 0;
            assert(mr[batch_cnt + i].num_mtt == 1);

            /* !TODO: Now, we require allocated memory's start
            * vaddr is at the boundry of one page */
            //mr[batch_cnt + i].addr   = memalign(PAGE_SIZE, mr_attr->length);
            /* Allloc memory using new api */
            mr[batch_cnt + i].addr = (uint8_t *)memAlloc->allocMem((Addr)mr_attr->length).vaddr.start();
            memset(mr[batch_cnt + i].addr, 0, mr_attr->length);
            mr[batch_cnt + i].ctx = context;
            mr[batch_cnt + i].flag   = mr_attr->flag;
            mr[batch_cnt + i].length = mr_attr->length;

            mr[batch_cnt + i].mtt    = (struct ibv_mtt *)malloc(sizeof(struct ibv_mtt) * mr->num_mtt);
            mr[batch_cnt + i].mtt[0].vaddr = (void *)(mr[batch_cnt + i].addr);

            mtt_args->vaddr[i] = (uint8_t *) mr[batch_cnt + i].mtt[0].vaddr;
            mtt_args->paddr[i] = memAlloc->getPhyAddr((Addr)mtt_args->vaddr[i]);
        }
        mtt_args->batch_size = sub_bsz;
        write_cmd(HGKFD_IOC_ALLOC_MTT, (void *)mtt_args);
        for (uint32_t i = 0; i < sub_bsz; ++i) {
            mr[batch_cnt + i].mtt[0].mtt_index = mtt_args->mtt_index + i;
            mr[batch_cnt + i].mtt[0].paddr = mtt_args->paddr[i];
        }
        mtt_args->batch_size = sub_bsz;
        write_cmd(HGKFD_IOC_WRITE_MTT, (void *)mtt_args);

        /* Allocate MPT */
        mpt_alloc_args->batch_size = sub_bsz;
        write_cmd(HGKFD_IOC_ALLOC_MPT, (void *)mpt_alloc_args);
        for (uint32_t i = 0; i < sub_bsz; ++i) {
            mr[batch_cnt + i].lkey = mpt_alloc_args->mpt_index + i;
            assert(mr[batch_cnt + i].lkey == mr[batch_cnt + i].mtt->mtt_index);
            // HGRNIC_PRINT(" ibv_reg_batch_mr: mpt_idx 0x%x mtt_idx 0x%x\n", mr[batch_cnt + i].lkey, mr[batch_cnt + i].mtt->mtt_index);
        }

        /* Write MPT */
        mpt_args->batch_size = sub_bsz;
        for (uint32_t i = 0; i < sub_bsz; ++i) {
            mpt_args->flag[i]      = mr[batch_cnt + i].flag;
            mpt_args->addr[i]      = (uint64_t) mr[batch_cnt + i].addr;
            mpt_args->length[i]    = mr[batch_cnt + i].length;
            mpt_args->mtt_index[i] = mr[batch_cnt + i].mtt[0].mtt_index;
            mpt_args->mpt_index[i] = mr[batch_cnt + i].lkey;
        }
        write_cmd(HGKFD_IOC_WRITE_MPT, (void *)mpt_args);

        /* update finished  */
        batch_left -= sub_bsz;
        batch_cnt += sub_bsz;
        assert(batch_cnt + batch_left == batch_size);
    }
    free(mtt_args);
    free(mpt_alloc_args);
    free(mpt_args);

    DPRINTF(Ibv, " ibv_reg_batch_mr!: out!\n");
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

/**
 * @note Now, SQ and RQ has their own MR respectively.
 */
struct ibv_qp* Ibv::ibv_create_qp(struct ibv_context *context, struct ibv_qp_create_attr *qp_attr) {

    DPRINTF(Ibv, " enter ibv_create_qp!\n");

    struct hghca_context *dvr = (struct hghca_context *)context->dvr;
    struct ibv_qp *qp = (struct ibv_qp *)malloc(sizeof(struct ibv_qp));
    memset(qp, 0, sizeof(struct ibv_qp));

    // allocate QP
    struct kfd_ioctl_alloc_qp_args *qp_args =
            (struct kfd_ioctl_alloc_qp_args *)malloc(sizeof(struct kfd_ioctl_alloc_qp_args));
    qp_args->batch_size = 1;
    write_cmd(HGKFD_IOC_ALLOC_QP, (void *)qp_args);
    qp->qp_num = qp_args->qp_num;
    DPRINTF(Ibv, " Get out of HGKFD_IOC_ALLOC_QP! qpn is : 0x%x\n", qp->qp_num);
    free(qp_args);

    // Init (Allocate and write) SQ MTT && MPT
    struct ibv_mr_init_attr *mr_attr =
            (struct ibv_mr_init_attr *)malloc(sizeof(struct ibv_mr_init_attr));
    mr_attr->flag   = MR_FLAG_WR | MR_FLAG_LOCAL;
    mr_attr->length = (1 << qp_attr->sq_size_log); // !TODO: Now the size is a fixed number of 1 page
    qp->snd_mr = ibv_reg_mr(context, mr_attr);

    // Init (Allocate and write) RQ MTT && MPT
    mr_attr->flag   = MR_FLAG_WR | MR_FLAG_LOCAL;
    mr_attr->length = (1 << qp_attr->rq_size_log); // !TODO: Now the size is a fixed number of 1 page
    qp->rcv_mr = ibv_reg_mr(context, mr_attr);
    DPRINTF(Ibv, " Get out of ibv_reg_mr in create_qp! qpn is : 0x%x\n", qp->qp_num);
    free(mr_attr);

    return qp;
}

/**
 * @note Allocate a batch of QP, with conntinuous qpn and the same qp_attr
 */
struct ibv_qp* Ibv::ibv_create_batch_qp(struct ibv_context *context, struct ibv_qp_create_attr *qp_attr, uint32_t batch_size) {

    DPRINTF(Ibv, " enter ibv_create_batch_qp!\n");

    struct hghca_context *dvr = (struct hghca_context *)context->dvr;
    struct ibv_qp *qp = (struct ibv_qp *)malloc(sizeof(struct ibv_qp) * batch_size);
    memset(qp, 0, sizeof(struct ibv_qp));

    /* allocate QP */
    uint32_t batch_cnt = 0;
    uint32_t batch_left = batch_size;
    struct kfd_ioctl_alloc_qp_args *qp_args =
            (struct kfd_ioctl_alloc_qp_args *)malloc(sizeof(struct kfd_ioctl_alloc_qp_args));
    while (batch_left > 0) {

        uint32_t sub_bsz = (batch_left > MAX_QPC_BATCH) ? MAX_QPC_BATCH : batch_left;

        qp_args->batch_size = sub_bsz;
        write_cmd(HGKFD_IOC_ALLOC_QP, (void *)qp_args);
        for (uint32_t i = 0; i < sub_bsz; ++i) {
            qp[batch_cnt + i].qp_num = qp_args->qp_num + i;
            // HGRNIC_PRINT(" Get out of HGKFD_IOC_ALLOC_QP! the %d-th qp, qpn is : 0x%x(%d)\n", batch_cnt + i, qp[batch_cnt + i].qp_num, qp[batch_cnt + i].qp_num&RESC_LIM_MASK);
        }

        batch_cnt  += sub_bsz;
        batch_left -= sub_bsz;
        assert(batch_cnt + batch_left == batch_size);
    }
    free(qp_args);

    // Init (Allocate and write) QP MTT && MPT
    struct ibv_mr_init_attr *mr_attr = 
            (struct ibv_mr_init_attr *)malloc(sizeof(struct ibv_mr_init_attr));
    mr_attr->flag   = MR_FLAG_WR | MR_FLAG_LOCAL;
    mr_attr->length = (1 << qp_attr->sq_size_log); // !TODO: Now the size is a fixed number of 1 page
    struct ibv_mr *tmp_mr = ibv_reg_batch_mr(context, mr_attr, batch_size * 2);

    for (uint32_t i = 0; i < batch_size; ++i) {
        qp[i].rcv_mr = &(tmp_mr[2 * i]);
        qp[i].snd_mr = &(tmp_mr[2 * i + 1]);
        DPRINTF(Ibv, " Get out of ibv_reg_batch_mr in create_qp! qpn is : 0x%x rcv_mr 0x%x snd_mr 0x%x\n", 
                qp[i].qp_num, qp[i].rcv_mr->lkey, qp[i].snd_mr->lkey);
    }
    free(mr_attr);

    return qp;
}



int Ibv::ibv_modify_batch_qp(struct ibv_context *context, struct ibv_qp *qp, uint32_t batch_size) {
    DPRINTF(Ibv, " enter ibv_modify_batch_qp!\n");
    struct hghca_context *dvr = (struct hghca_context *)context->dvr;

    /* write QP */
    struct kfd_ioctl_write_qpc_args *qpc_args =
            (struct kfd_ioctl_write_qpc_args *)malloc(sizeof(struct kfd_ioctl_write_qpc_args));
    memset(qpc_args, 0, sizeof(struct kfd_ioctl_write_qpc_args));
    uint32_t batch_cnt = 0;
    uint32_t batch_left = batch_size;
    while (batch_left > 0) {

        uint32_t sub_bsz = (batch_left > MAX_QPC_BATCH) ? MAX_QPC_BATCH : batch_left;
        DPRINTF(Ibv, " ibv_modify_batch_qp! batch_cnt %d batch_left %d sub_bsz %d\n", batch_cnt, batch_left, sub_bsz);

        qpc_args->batch_size = sub_bsz;
        for (int i = 0; i < sub_bsz; ++i) {
            qpc_args->flag    [i] = qp[batch_cnt + i].flag;
            qpc_args->type    [i] = qp[batch_cnt + i].type;
            qpc_args->llid    [i] = qp[batch_cnt + i].lsubnet.llid;
            qpc_args->dlid    [i] = qp[batch_cnt + i].dsubnet.dlid;
            qpc_args->src_qpn [i] = qp[batch_cnt + i].qp_num;
            qpc_args->dest_qpn[i] = qp[batch_cnt + i].dest_qpn;
            qpc_args->snd_psn [i] = qp[batch_cnt + i].snd_psn;
            qpc_args->ack_psn [i] = qp[batch_cnt + i].ack_psn;
            qpc_args->exp_psn [i] = qp[batch_cnt + i].exp_psn;
            qpc_args->cq_num  [i] = qp[batch_cnt + i].cq->cq_num;
            qpc_args->snd_wqe_base_lkey[i] = qp[batch_cnt + i].snd_mr->lkey;
            qpc_args->rcv_wqe_base_lkey[i] = qp[batch_cnt + i].rcv_mr->lkey;
            qpc_args->snd_wqe_offset   [i] = qp[batch_cnt + i].snd_wqe_offset;
            qpc_args->rcv_wqe_offset   [i] = qp[batch_cnt + i].rcv_wqe_offset;
            qpc_args->qkey       [i] = qp[batch_cnt + i].qkey;
            qpc_args->sq_size_log[i] = PAGE_SIZE_LOG; // qp->snd_mr->length;
            qpc_args->rq_size_log[i] = PAGE_SIZE_LOG; // qp->rcv_mr->length;

            // HGRNIC_PRINT(" ibv_modify_batch_qp! qpn 0x%x\n", qp[batch_cnt + i].qp_num);
        }
        write_cmd(HGKFD_IOC_WRITE_QPC, qpc_args);

        batch_cnt  += sub_bsz;
        batch_left -= sub_bsz;
        assert(batch_cnt + batch_left == batch_size);
    }
    free(qpc_args);

    DPRINTF(Ibv, " ibv_modify_batch_qp: out!\n");
    return 0;
}

