#include "base/statistics.hh"
#include "base/trace.hh"
#include "dev/rdma/hangu_rnic_defs.hh"
#include "ibv_test_base.hh"

IbvTestBase::IbvTestBase(const Params *p)
    : SimObject(p),
    ibv(p->ibv),
    pollCplEvent([this] { poll_cpl(); }, name()),
    rdmaWriteEvent([this] { rdma_write_pre(); }, name())
{
    has_rinfo = false;
    DPRINTF(IbvTestBase, "Initializing IbvTestBase\n");
} // IbvTestBase::IbvTestBase


/******************************* Init wqe *********************************/
struct ibv_wqe *IbvTestBase::init_rcv_wqe (struct ibv_context *ctx, int num) {
    struct ibv_wqe *wqe = (struct ibv_wqe *)malloc(sizeof(struct ibv_wqe) * num);

    for (int i = 0; i < num; ++i) {
        wqe[i].length = sizeof(rdma_cr);
        wqe[i].mr = ctx->cm_mr;
        wqe[i].offset = ctx->cm_rcv_posted_off;
        /* Add rcv element */
        wqe[i].trans_type = IBV_TYPE_RECV;
        DPRINTF(IbvTestBase, "[test requester] init_rcv_wqe: len is %d\n", wqe[i].length);

        ++ctx->cm_rcv_num;
        ctx->cm_rcv_posted_off += sizeof(struct rdma_cr);
        if (ctx->cm_rcv_posted_off + sizeof(struct rdma_cr) > RCV_WR_MAX * sizeof(struct rdma_cr)) {
            ctx->cm_rcv_posted_off = 0;
        }
    }
    return wqe;
}


struct ibv_wqe *IbvTestBase::init_snd_wqe (struct ibv_context *ctx, struct rdma_cr *cr_info, int num, uint16_t dlid) {

    struct ibv_wqe *wqe = (struct ibv_wqe *)malloc(sizeof(struct ibv_wqe) * num);
    struct rdma_cr *cr;

    if (num > SND_WR_MAX) {
        num = SND_WR_MAX;
        DPRINTF(IbvTestBase, "init_snd_wqe: There's not enough room for CM to post send!\n");
        assert(num < SND_WR_MAX);
    }

#define TRANS_SEND_DATA "hello RDMA Send!"

    for (int i = 0; i < num; ++i) {
        cr = (struct rdma_cr *)(ctx->cm_mr->addr + ctx->cm_snd_off);
        memcpy(cr, cr_info + i, sizeof(struct rdma_cr));

        //DPRINTF(IbvTestBase, "[test requester] init_snd_wqe: string is %s, string vaddr is 0x%lx, start vaddr is 0x%lx\n",
                //string, (uint64_t)string, (uint64_t)(&string));

        wqe[i].length = sizeof(struct rdma_cr);
        DPRINTF(IbvTestBase, "[test requester] init_snd_wqe: len is %d\n", wqe[i].length);
        wqe[i].mr = ctx->cm_mr;
        wqe[i].offset = ctx->cm_snd_off;

        /* Add Send element */
        wqe[i].trans_type = IBV_TYPE_SEND;
        wqe[i].send.dlid = dlid;
        wqe[i].send.dqpn = ctx->cm_qp->qp_num;
        wqe[i].send.qkey = QKEY_CM;

        ctx->cm_snd_off += sizeof(struct rdma_cr);
        if (ctx->cm_snd_off + sizeof(struct rdma_cr) > SND_WR_BASE + SND_WR_MAX * sizeof(struct rdma_cr)) {
            ctx->cm_snd_off = SND_WR_BASE;
        }
    }
    return wqe;
}


struct ibv_wqe *IbvTestBase::init_rdma_write_wqe (struct ibv_mr* lmr) {

    uint64_t raddr = res->rinfo->raddr;
    uint32_t rkey = res->rinfo->rkey;

    struct ibv_wqe *wqe = (struct ibv_wqe *)malloc(sizeof(struct ibv_wqe) * res->num_wqe);

#define RDMA_WRITE_DATA "hello, RDMA Write from server!"

    // Write data to mr
    uint32_t offset = 0;
    char *string = (char *)(lmr->addr + offset);
    memcpy(string, RDMA_WRITE_DATA, sizeof(RDMA_WRITE_DATA));
    DPRINTF(IbvTestBase, "init_rdma_write_wqe: string is %s, string vaddr is 0x%lx, start vaddr is 0x%lx\n",
                string, (uint64_t)string, (uint64_t)(lmr->addr + offset));

    for (int i = 0; i < res->num_wqe; ++i) {

        wqe[i].length = sizeof(RDMA_WRITE_DATA);
        wqe[i].mr = lmr;
        wqe[i].offset = offset;

        wqe[i].flag = WR_FLAG_SIGNALED; /* This flag is used to get cpl */

        // Add RDMA Write element
        wqe[i].trans_type = IBV_TYPE_RDMA_WRITE;
        wqe[i].rdma.raddr = raddr + (sizeof(RDMA_WRITE_DATA) -1) * i + offset;
        wqe[i].rdma.rkey  = rkey;
    }
    return wqe;
}

struct ibv_wqe *IbvTestBase::init_rdma_read_wqe (struct ibv_mr* req_mr, struct ibv_mr* rsp_mr, uint32_t qkey) {

    struct ibv_wqe *wqe = (struct ibv_wqe *)malloc(sizeof(struct ibv_wqe));

#define TRANS_RRDMA_DATA "hello RDMA Read!"

    // Write data to mr
    uint32_t offset = 0;
    char *string = (char *)(rsp_mr->addr + offset);
    memcpy(string, TRANS_RRDMA_DATA, sizeof(TRANS_RRDMA_DATA));

    DPRINTF(IbvTestBase, "[test requester] init_snd_wqe: string is %s, string vaddr is 0x%lx, start vaddr is 0x%lx\n",
            string, (uint64_t)string, (uint64_t)(rsp_mr->addr + offset));

    wqe->length = sizeof(TRANS_RRDMA_DATA);
    DPRINTF(IbvTestBase, "[test requester] init_snd_wqe: len is %d, addr: 0x%lx, key: %x\n", wqe->length, (uint64_t)rsp_mr->addr, rsp_mr->lkey);
    wqe->mr = req_mr;
    wqe->offset = offset;

    // Add RDMA Read element
    wqe->trans_type = IBV_TYPE_RDMA_READ;
    wqe->rdma.raddr = (uint64_t)rsp_mr->addr;
    wqe->rdma.rkey  = rsp_mr->lkey;

    // DPRINTF(IbvTestBase, "[test requester] init_snd_wqe: rKey: 0x%x, rAddr: 0x%lx\n", wqe->rdma.rkey, wqe->rdma.raddr);
    return wqe;
}
/******************************* Init wqe *********************************/


/******************************* Poll Cpl *********************************/
struct cpl_desc ** IbvTestBase::malloc_desc() {
    struct cpl_desc **desc = (struct cpl_desc **)malloc(sizeof(struct cpl_desc *) * MAX_CPL_NUM);
    for (int i = 0; i < MAX_CPL_NUM; ++i) {
        desc[i] = (struct cpl_desc *)malloc(sizeof(struct cpl_desc));
    }
    return desc;
}


void IbvTestBase::poll_cpl() {
    if (cplWaitingList.empty()) {
        return;
    }
    //DPRINTF(IbvTestBase, "[DEBUG] poll_cpl: Waiting for cpl ...\n");
    void (IbvTestBase::*deal_cpl)(struct cpl_desc *) = NULL;
    struct ibv_cq *cq;

    for (auto it = cplWaitingList.begin(); it != cplWaitingList.end(); it++) {
        cq = *it;

        assert(!cplWaitingList.empty());

        /* Determine the processing function */
        if (cq->cq_num == res->ctx->cm_cq->cq_num) {
            deal_cpl = &IbvTestBase::rdma_listen_post;
        } else {
            deal_cpl = &IbvTestBase::rdma_write_post;
        }

        int cpl_num = ibv->ibv_poll_cpl(cq, res->desc, MAX_CPL_NUM);
        if (cpl_num) {
            DPRINTF(IbvTestBase, "poll_cpl finish: cq_num: %d, return %d, cpl_cnt: %d\n", cq->cq_num, res, cq->cpl_cnt);
            for (int i=0; i<cpl_num; i++) {
                (this->*deal_cpl)(res->desc[i]);
            }
            break;
        }
    }
    /* Schedule itself */
    if (!pollCplEvent.scheduled()) {
        schedule(pollCplEvent, curTick() + 1000);
    }
}
/******************************* Poll Cpl *********************************/


/******************************* RDMA Ops *********************************/
void IbvTestBase::rdma_connect(struct rdma_resc *resc, uint16_t svr_lid) {
    struct ibv_context *ctx = resc->ctx;
    struct rdma_cr *cr_snd;
    uint16_t *dest_info = (uint16_t *)malloc(sizeof(uint16_t));
    int cm_req_num = 1;

    cr_snd = (struct rdma_cr *)malloc(sizeof(struct rdma_cr));
    memset(cr_snd, 0, sizeof(struct rdma_cr));

    cr_snd->flag    = CR_TYPE_REQ;
    cr_snd->src_lid = resc->ctx->lid;
    cr_snd->rkey    = resc->mr[0]->lkey; /* only use one mr in our design */
    cr_snd->raddr   = (uintptr_t)resc->mr[0]->addr; /* only use one mr in our design */
    cr_snd->src_qpn = resc->qp[0]->qp_num;
    dest_info[0]    = svr_lid;

    DPRINTF(IbvTestBase, "rdma_connect: send raddr %ld, rkey 0x%x\n", cr_snd->raddr, cr_snd->rkey);

    int i = 0;
    while (i < cm_req_num) {
        /* Post Same destination in one doorbell */
        DPRINTF(IbvTestBase, "rdma_connect: cm_post_send dest_info 0x%x cnt %d\n", dest_info[i], i);
        struct ibv_wqe *send_wqe = init_snd_wqe(ctx, cr_snd+i, 1, dest_info[i]);
        ibv->ibv_post_send(ctx, send_wqe, ctx->cm_qp, 1);
        i++;
    }
}


void IbvTestBase::rdma_listen_pre() {
    struct ibv_context *ctx = res->ctx;

    DPRINTF(IbvTestBase, "Put %d cq into waiting list\n", ctx->cm_cq->cq_num);
    cplWaitingList.insert(ctx->cm_cq);

    if (!pollCplEvent.scheduled()) {
        schedule(pollCplEvent, curTick());
    }
}


void IbvTestBase::rdma_listen_post(struct cpl_desc *desc) {
    struct ibv_context *ctx = res->ctx;
    struct rdma_cr *cr_info;

    DPRINTF(IbvTestBase, "rdma_listen_post: cpl_desc trans_type %d cq_num %d\n", desc->trans_type, desc->cq_num);

    if (desc->trans_type == IBV_TYPE_RECV) {
        DPRINTF(IbvTestBase, "rdma_listen_post: ibv_poll_cpl recv %d bytes CR.\n", desc->byte_cnt);
        /* Fetch rdma_cr from cm_mr */
        cr_info = (struct rdma_cr *)malloc(sizeof(struct rdma_cr));
        struct rdma_cr *cr_tmp = ((struct rdma_cr *)(ctx->cm_mr->addr + ctx->cm_rcv_acked_off));

        memcpy(cr_info, cr_tmp, sizeof(struct rdma_cr));

        /* DEBUG: print cr_info */
        uint8_t *u8_tmp = (uint8_t *)(cr_info);
        DPRINTF(IbvTestBase, "rdma_listen_post: flag: 0x%x, qp_type 0x%lx, raddr 0x%lx, rkey 0x%x, dst_qpn 0x%x\n",
                cr_info->flag, (uint64_t)cr_info->qp_type, (uint64_t)cr_info->raddr,
                cr_info->rkey, cr_info->dst_qpn);
        for (int j = 0; j < sizeof(struct rdma_cr); ++j) {
            DPRINTF(IbvTestBase, "rdma_listen_post:\t data[%d] 0x%x\n", j, u8_tmp[j]);
        }

        /* Clear cpl data */
        --ctx->cm_rcv_num;
        ctx->cm_rcv_acked_off += sizeof(struct rdma_cr);
        if (ctx->cm_rcv_acked_off + sizeof(struct rdma_cr) > RCV_WR_MAX * sizeof(struct rdma_cr)) {
            ctx->cm_rcv_acked_off = 0;
        }
    }

    /* Post CM recv to RQ */
    if (ctx->cm_rcv_num < RCV_WR_MAX / 2) {
        struct ibv_wqe *recv_wqe = init_rcv_wqe(ctx, RCV_WR_MAX);
        ibv->ibv_post_recv(ctx, recv_wqe, ctx->cm_qp, RCV_WR_MAX);
        //int rcv_wqe_num = cm_post_recv(ctx, RCV_WR_MAX);
        DPRINTF(IbvTestBase, "rdma_listen_post: Replenish %d Recv WQEs\n", RCV_WR_MAX);
    }

    /* get remote addr information */
    res->rinfo->dlid  = cr_info->src_lid; /* This is the dest MAC of subsequent WRITE */
    res->rinfo->raddr = cr_info->raddr;
    res->rinfo->rkey  = cr_info->rkey;
    res->rinfo->qpn   = cr_info->src_qpn; /* Used to specify client's qpn */
    has_rinfo = true;
    DPRINTF(IbvTestBase, "Received rinfo: raddr 0x%lx, rkey 0x%x\n", res->rinfo->raddr, res->rinfo->rkey);

    /* reconfig qp */
    config_rc_qp();

    /* remove cm_cq from waiting list */
    DPRINTF(IbvTestBase, "Removing cq %d from waiting list\n", ctx->cm_cq->cq_num);
    cplWaitingList.erase(ctx->cm_cq);

    /* schedule write operation */
    if (!rdmaWriteEvent.scheduled()) {
        schedule(rdmaWriteEvent, curTick());
    }
}


void IbvTestBase::rdma_write_pre() {
    struct ibv_wqe *wrdma_wqe = init_rdma_write_wqe(res->mr[0]);

    for (int i = 0; i < res->num_qp; ++i) {
        ibv->ibv_post_send(res->ctx, wrdma_wqe, res->qp[i], 1);
        DPRINTF(IbvTestBase, "IBV post write wqe to qp %d!\n", res->qp[i]->qp_num);

        cplWaitingList.insert(res->cq[i]);
        DPRINTF(IbvTestBase, "Put %d cq into waiting list\n", res->cq[i]->cq_num);
    }

    if (!pollCplEvent.scheduled()) {
        schedule(pollCplEvent, curTick() + 1000);
    }
}

void IbvTestBase::rdma_write_post(struct cpl_desc *desc) {

    if (desc->trans_type == IBV_TYPE_RDMA_WRITE) {
        DPRINTF(IbvTestBase, "rdma_write_post: ibv_poll_cpl write completion:\n");
        DPRINTF(IbvTestBase, "\t%d bytes transfered\n", desc->byte_cnt);
        DPRINTF(IbvTestBase, "\ttrans_type: %d, srv_type: %d\n", desc->trans_type, desc->srv_type);
        DPRINTF(IbvTestBase, "\tqp_num: %d, cq_num: %d.\n", desc->qp_num, desc->cq_num);
    }
}
/******************************* RDMA Ops *********************************/


/******************************* Config *********************************/
struct rdma_resc *IbvTestBase::resc_init(uint16_t llid, int num_qp, int num_mr, int num_cq, int num_wqe) {
    struct rdma_resc *res = (struct rdma_resc *)malloc(sizeof(struct rdma_resc));
    memset(res, 0, sizeof(struct rdma_resc));
    res->num_mr  = num_mr;
    res->num_cq  = num_cq;
    res->num_qp  = num_qp;
    res->mr = (struct ibv_mr **)malloc(sizeof(struct ibv_mr*) * num_mr);
    res->cq = (struct ibv_cq **)malloc(sizeof(struct ibv_cq*) * num_cq);
    res->qp = (struct ibv_qp **)malloc(sizeof(struct ibv_qp*) * num_qp);
    res->rinfo = (struct rem_info *)malloc(sizeof(struct rem_info));

    res->num_qp = num_qp;
    res->num_wqe = num_wqe;

    res->desc = malloc_desc();

    struct ibv_context *ctx = (struct ibv_context *)malloc(sizeof(struct ibv_context));
    res->ctx = ctx;
    ibv->ibv_open_device(res->ctx, llid);
    DPRINTF(IbvTestBase, "[test requester] ibv_open_device End. Doorbell addr 0x%lx\n", (long int)res->ctx->dvr);

    struct ibv_mr_init_attr mr_attr;
    mr_attr.length = 1 << 12;
    mr_attr.flag = (enum ibv_mr_flag)(MR_FLAG_RD | MR_FLAG_WR | MR_FLAG_LOCAL | MR_FLAG_REMOTE);
    res->mr[0] = ibv->ibv_reg_mr(res->ctx, &mr_attr);
    DPRINTF(IbvTestBase, "[test requester] ibv_reg_mr End! lkey %d, vaddr 0x%lx\n", res->mr[0]->lkey, (uint64_t)res->mr[0]->addr);

    struct ibv_cq_init_attr cq_attr;
    cq_attr.size_log = 12;
    struct ibv_cq *cq = ibv->ibv_create_cq(res->ctx, &cq_attr);
    DPRINTF(IbvTestBase, "[test requester] ibv_create_cq End! cqn %d\n", cq->cq_num);
    res->cq[0] = cq;

    struct ibv_qp_create_attr qp_attr;
    qp_attr.sq_size_log = 12;
    qp_attr.rq_size_log = 12;
    struct ibv_qp *qp = ibv->ibv_create_qp(res->ctx, &qp_attr);
    DPRINTF(IbvTestBase, "[test requester] ibv_create_qp end! qpn %d\n", qp->qp_num);
    res->qp[0] = qp;
    return res;
}

void IbvTestBase::config_rc_qp() {

    for (int i = 0; i < res->num_qp; ++i) {
        res->qp[i]->ctx = res->ctx;
        res->qp[i]->flag = 0;
        res->qp[i]->type = QP_TYPE_RC;
        res->qp[i]->cq = res->cq[i];
        res->qp[i]->snd_wqe_offset = 0;
        res->qp[i]->rcv_wqe_offset = 0;
        res->qp[i]->lsubnet.llid = res->ctx->lid;

        res->qp[i]->dest_qpn = res->rinfo->qpn;
        res->qp[i]->snd_psn = 0;
        res->qp[i]->ack_psn = 0;
        res->qp[i]->exp_psn = 0;
        res->qp[i]->dsubnet.dlid = res->rinfo->dlid;

        ibv->ibv_modify_qp(res->ctx, res->qp[i]);
        DPRINTF(IbvTestBase, "[test requester] ibv_modify_qp end!\n");
    }
}


void IbvTestBase::config_ud_qp (struct ibv_qp* qp, struct ibv_cq *cq, struct ibv_context *ctx, uint32_t qkey) {
    qp->ctx = ctx;
    qp->flag = 0;
    qp->type = QP_TYPE_UD;
    qp->cq = cq;
    qp->snd_wqe_offset = 0;
    qp->rcv_wqe_offset = 0;
    qp->lsubnet.llid = 0x0001;

    // qp->dest_qpn = 1;
    // qp->snd_psn = 0;
    // qp->ack_psn = 0;
    // qp->exp_psn = 0;

    // qp->dsubnet.dlid = 0x0000;

    qp->qkey = qkey;
}
/******************************* Config *********************************/


/* This function is compulsory */
IbvTestBase * IbvTestBaseParams::create() {
    return new IbvTestBase(this);
}
