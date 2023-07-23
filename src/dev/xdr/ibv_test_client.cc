#include "ibv_test_client.hh"
#include "base/statistics.hh"
#include "base/trace.hh"
#include "dev/xdr/libibv.hh"
#include <queue>

IbvTestClient::IbvTestClient(const Params *p)
    : SimObject(p),
    ibv(p->ibv),
    mainEvent([this]{ main(); }, name()),
    pollCplEvent([this] { pollCpl(); }, name()),
    readCplEvent([this] { readCpl(); }, name())
{
    DPRINTF(IbvTestClient, "Initializing IbvTestClient\n");
    desc = malloc_desc();
    if (!mainEvent.scheduled()) {
        DPRINTF(IbvTestClient, "mainEvent is being scheduled ...\n");
        schedule(mainEvent, curTick() + 1000);
    }
} // IbvTestClient::IbvTestClient


/******************************* Init wqe *********************************/
struct ibv_wqe *IbvTestClient::init_rcv_wqe (struct ibv_context *ctx, int num) {
    struct ibv_wqe *wqe = (struct ibv_wqe *)malloc(sizeof(struct ibv_wqe) * num);

    for (int i = 0; i < num; ++i) {
        wqe[i].length = sizeof(rdma_cr);
        wqe[i].mr = ctx->cm_mr;
        wqe[i].offset = ctx->cm_rcv_posted_off;
        /* Add rcv element */
        wqe[i].trans_type = IBV_TYPE_RECV;
        DPRINTF(IbvTestClient, "[test requester] init_rcv_wqe: len is %d\n", wqe[i].length);

        ++ctx->cm_rcv_num;
        ctx->cm_rcv_posted_off += sizeof(struct rdma_cr);
        if (ctx->cm_rcv_posted_off + sizeof(struct rdma_cr) > RCV_WR_MAX * sizeof(struct rdma_cr)) {
            ctx->cm_rcv_posted_off = 0;
        }
    }
    return wqe;
}


struct ibv_wqe *IbvTestClient::init_snd_wqe (struct ibv_context *ctx, struct rdma_cr *cr_info, int num, uint16_t dlid) {

    struct ibv_wqe *wqe = (struct ibv_wqe *)malloc(sizeof(struct ibv_wqe) * num);
    struct rdma_cr *cr;

    if (num > SND_WR_MAX) {
        num = SND_WR_MAX;
        DPRINTF(IbvTestClient, "cm_post_send: There's not enough room for CM to post send!\n");
        assert(num < SND_WR_MAX);
    }

#define TRANS_SEND_DATA "hello RDMA Send!"

    for (int i = 0; i < num; ++i) {
        cr = (struct rdma_cr *)(ctx->cm_mr->addr + ctx->cm_snd_off);
        memcpy(cr, cr_info + i, sizeof(struct rdma_cr));

        //DPRINTF(IbvTestClient, "[test requester] init_snd_wqe: string is %s, string vaddr is 0x%lx, start vaddr is 0x%lx\n",
                //string, (uint64_t)string, (uint64_t)(&string));

        wqe[i].length = sizeof(struct rdma_cr);
        DPRINTF(IbvTestClient, "[test requester] init_snd_wqe: len is %d\n", wqe[i].length);
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

struct ibv_wqe *IbvTestClient::init_rdma_write_wqe (struct rdma_resc *res, struct ibv_mr* lmr, uint64_t raddr, uint32_t rkey) {

    struct ibv_wqe *wqe = (struct ibv_wqe *)malloc(sizeof(struct ibv_wqe) * res->num_wqe);

#define RDMA_WRITE_DATA "hello RDMA Write!"

    // Write data to mr
    uint32_t offset = 0;
    char *string = (char *)(lmr->addr + offset);
    memcpy(string, RDMA_WRITE_DATA, sizeof(RDMA_WRITE_DATA));
    DPRINTF(IbvTestClient, "[test requester] init_snd_wqe: string is %s, string vaddr is 0x%lx, start vaddr is 0x%lx\n",
                string, (uint64_t)string, (uint64_t)(lmr->addr + offset));

    for (int i = 0; i < res->num_wqe; ++i) {

        wqe[i].length = sizeof(RDMA_WRITE_DATA);
        wqe[i].mr = lmr;
        wqe[i].offset = offset;

        // Add RDMA Write element
        wqe[i].trans_type = IBV_TYPE_RDMA_WRITE;
        wqe[i].rdma.raddr = raddr;
        wqe[i].rdma.rkey  = rkey;
    }
    return wqe;
}

struct ibv_wqe *IbvTestClient::init_rdma_read_wqe (struct ibv_mr* req_mr, struct ibv_mr* rsp_mr, uint32_t qkey) {

    struct ibv_wqe *wqe = (struct ibv_wqe *)malloc(sizeof(struct ibv_wqe));

#define TRANS_RRDMA_DATA "hello RDMA Read!"

    // Write data to mr
    uint32_t offset = 0;
    char *string = (char *)(rsp_mr->addr + offset);
    memcpy(string, TRANS_RRDMA_DATA, sizeof(TRANS_RRDMA_DATA));

    DPRINTF(IbvTestClient, "[test requester] init_snd_wqe: string is %s, string vaddr is 0x%lx, start vaddr is 0x%lx\n",
            string, (uint64_t)string, (uint64_t)(rsp_mr->addr + offset));

    wqe->length = sizeof(TRANS_RRDMA_DATA);
    DPRINTF(IbvTestClient, "[test requester] init_snd_wqe: len is %d, addr: 0x%lx, key: %x\n", wqe->length, (uint64_t)rsp_mr->addr, rsp_mr->lkey);
    wqe->mr = req_mr;
    wqe->offset = offset;

    // Add RDMA Read element
    wqe->trans_type = IBV_TYPE_RDMA_READ;
    wqe->rdma.raddr = (uint64_t)rsp_mr->addr;
    wqe->rdma.rkey  = rsp_mr->lkey;

    // DPRINTF(IbvTestClient, "[test requester] init_snd_wqe: rKey: 0x%x, rAddr: 0x%lx\n", wqe->rdma.rkey, wqe->rdma.raddr);
    return wqe;
}
/******************************* Init wqe *********************************/


/******************************* Poll Cpl *********************************/
struct cpl_desc ** IbvTestClient::malloc_desc() {
    struct cpl_desc **desc = (struct cpl_desc **)malloc(sizeof(struct cpl_desc *) * MAX_CPL_NUM);
    for (int i = 0; i < MAX_CPL_NUM; ++i) {
        desc[i] = (struct cpl_desc *)malloc(sizeof(struct cpl_desc));
    }
    return desc;
}

void IbvTestClient::pollCpl() {
    for (struct ibv_cq *cq : cplWaitingList) {
        int res = ibv->ibv_poll_cpl(cq, desc, MAX_CPL_NUM);
        if (res) {
            DPRINTF(IbvTestClient, "poll_cpl finish: cq_num: %d, return %d, cpl_cnt: %d\n", cq->cq_num, res, cq->cpl_cnt);
            for (int i=0; i<res; i++) {
                rdma_listen_post(desc[i]);
            }
        }
    }
    /* Schedule itself */
    if (!cplWaitingList.empty() && !pollCplEvent.scheduled()) {
        schedule(pollCplEvent, curTick() + 1000);
    }
    return;
}

void IbvTestClient::readCpl() {}

/******************************* Poll Cpl *********************************/


void IbvTestClient::config_rc_qp(struct rdma_resc *res) {

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
        DPRINTF(IbvTestClient, "[test requester] ibv_modify_qp end!\n");
    }
}


void IbvTestClient::config_ud_qp (struct ibv_qp* qp, struct ibv_cq *cq, struct ibv_context *ctx, uint32_t qkey) {
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


void IbvTestClient::rdma_connect(struct rdma_resc *resc, uint16_t svr_lid) {
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
    dest_info[0]    = svr_lid;

    DPRINTF(IbvTestClient, "rdma_connect: send raddr %ld, rkey 0x%x\n", cr_snd->raddr, cr_snd->rkey);

    int i = 0;
    while (i < cm_req_num) {
        /* Post Same destination in one doorbell */
        DPRINTF(IbvTestClient, "rdma_connect: cm_post_send dest_info 0x%x cnt %d\n", dest_info[i], i);
        struct ibv_wqe *send_wqe = init_snd_wqe(ctx, cr_snd+i, 1, dest_info[i]);
        ibv->ibv_post_send(ctx, send_wqe, ctx->cm_qp, 1);
    }
}

void IbvTestClient::rdma_listen_pre() {
    struct ibv_context *ctx = res->ctx;

    DPRINTF(IbvTestClient, "Scheduling pollCplEvent from rdma_listen_pre\n");

    cplWaitingList.insert(ctx->cm_cq);
    if (!pollCplEvent.scheduled()) {
        schedule(pollCplEvent, curTick());
    }
}

struct rdma_cr *IbvTestClient::rdma_listen_post(struct cpl_desc *desc) {
    struct ibv_context *ctx = res->ctx;
    struct rdma_cr *cr_info;

    DPRINTF(IbvTestClient, "rdma_listen_post: cpl_desc trans_type %d cq_num %d\n", desc->trans_type, desc->cq_num);

    if (desc->trans_type == IBV_TYPE_RECV && desc->cq_num == ctx->cm_cq->cq_num) {
        DPRINTF(IbvTestClient, "rdma_listen: ibv_poll_cpl recv %d bytes CR.\n", desc->byte_cnt);
        /* Fetch rdma_cr from cm_mr */
        cr_info = (struct rdma_cr *)malloc(sizeof(struct rdma_cr));
        struct rdma_cr *cr_tmp = ((struct rdma_cr *)(ctx->cm_mr->addr + ctx->cm_rcv_acked_off));

        memcpy(cr_info, cr_tmp, sizeof(struct rdma_cr));

        /* DEBUG: print cr_info */
        uint8_t *u8_tmp = (uint8_t *)(cr_info);
        DPRINTF(IbvTestClient, "rdma_listen: flag: 0x%x, base_addr 0x%lx, acked_off 0x%lx, src_qpn 0x%x, dst_qpn 0x%x\n",
                cr_info->flag, (uint64_t)ctx->cm_mr->addr, (uint64_t)ctx->cm_rcv_acked_off,
                cr_info->src_qpn, cr_info->dst_qpn);
        for (int j = 0; j < sizeof(struct rdma_cr); ++j) {
            DPRINTF(IbvTestClient, "rdma_listen: data[%d] 0x%x\n", j, u8_tmp[j]);
        }

        /* Clear cpl data */
        --ctx->cm_rcv_num;
        ctx->cm_rcv_acked_off += sizeof(struct rdma_cr);
        if (ctx->cm_rcv_acked_off + sizeof(struct rdma_cr) > RCV_WR_MAX * sizeof(struct rdma_cr)) {
            ctx->cm_rcv_acked_off = 0;
        }
    }

    /* Post CM recv to RQ */
    if (ctx->cm_rcv_num < RCV_WR_MAX) {
        struct ibv_wqe *recv_wqe = init_rcv_wqe(ctx, RCV_WR_MAX);
        ibv->ibv_post_recv(ctx, recv_wqe, ctx->cm_qp, RCV_WR_MAX);
        //int rcv_wqe_num = cm_post_recv(ctx, RCV_WR_MAX);
        DPRINTF(IbvTestClient, "rdma_listen: Replenish %d Recv WQEs\n", RCV_WR_MAX);
    }

    /* get remote addr information */
    res->rinfo->dlid  = svr_lid;
    res->rinfo->raddr = cr_info->raddr;
    res->rinfo->rkey  = cr_info->rkey;
    DPRINTF(IbvTestClient, "Received rinfo: raddr %ld, rkey 0x%x\n", res->rinfo->raddr, res->rinfo->rkey);

    return cr_info;
}


struct rdma_resc *IbvTestClient::resc_init(uint16_t llid, int num_qp, int num_mr, int num_cq) {
    res = (struct rdma_resc *)malloc(sizeof(struct rdma_resc));
    memset(res, 0, sizeof(struct rdma_resc));
    res->num_mr  = num_mr;
    res->num_cq  = num_cq;
    res->num_qp  = num_qp;
    res->mr = (struct ibv_mr **)malloc(sizeof(struct ibv_mr*) * num_mr);
    res->cq = (struct ibv_cq **)malloc(sizeof(struct ibv_cq*) * num_cq);
    res->qp = (struct ibv_qp **)malloc(sizeof(struct ibv_qp*) * num_qp);

    res->num_qp = num_qp;
    //res->num_wqe = num_wqe;

    struct ibv_context *ctx = (struct ibv_context *)malloc(sizeof(struct ibv_context));
    res->ctx = ctx;
    ibv->ibv_open_device(res->ctx, llid);
    DPRINTF(IbvTestClient, "[test requester] ibv_open_device End. Doorbell addr 0x%lx\n", (long int)res->ctx->dvr);

    struct ibv_mr_init_attr mr_attr;
    mr_attr.length = 1 << 12;
    mr_attr.flag = (enum ibv_mr_flag)(MR_FLAG_RD | MR_FLAG_WR | MR_FLAG_LOCAL | MR_FLAG_REMOTE);
    res->mr[0] = ibv->ibv_reg_mr(res->ctx, &mr_attr);
    DPRINTF(IbvTestClient, "[test requester] ibv_reg_mr End! lkey %d, vaddr 0x%lx\n", res->mr[0]->lkey, (uint64_t)res->mr[0]->addr);

    struct ibv_cq_init_attr cq_attr;
    cq_attr.size_log = 12;
    struct ibv_cq * cq = ibv->ibv_create_cq(res->ctx, &cq_attr);
    DPRINTF(IbvTestClient, "[test requester] ibv_create_cq End! cqn %d\n", cq->cq_num);
    res->cq[0] = cq;

    struct ibv_qp_create_attr qp_attr;
    qp_attr.sq_size_log = 12;
    qp_attr.rq_size_log = 12;
    struct ibv_qp *qp = ibv->ibv_create_qp(res->ctx, &qp_attr);
    DPRINTF(IbvTestClient, "[test requester] ibv_create_qp end! qpn %d\n", qp->qp_num);
    res->qp = (ibv_qp **)malloc(res->num_qp * sizeof(ibv_qp *));
    res->qp[0] = qp;
    return res;
}

/*****************************************************
 *********************** Entrance ********************
 *****************************************************/
int IbvTestClient::main () {
    int rtn;
    clt_lid=100;
    svr_lid=200;
    sprintf(id_name, "%d", 999);

    int num_qp = 1, num_mr = 1, num_cq = 1;
    res = resc_init(clt_lid, num_qp, num_mr, num_cq);

    DPRINTF(IbvTestClient, "main function executing ...\n");

    /********************* receive ********************/
    struct ibv_wqe *recv_wqe = init_rcv_wqe(res->ctx, RCV_WR_MAX);
    ibv->ibv_post_recv(res->ctx, recv_wqe, res->ctx->cm_qp, RCV_WR_MAX);

    rdma_connect(res, svr_lid);

    config_rc_qp(res);

    return 0;

    /********************* write ********************/
    struct ibv_wqe *wrdma_wqe = init_rdma_write_wqe(res, res->mr[0], res->rinfo->raddr, res->rinfo->rkey);
    for (int i = 0; i < res->num_qp; ++i) {
        ibv->ibv_post_send(res->ctx, wrdma_wqe, res->qp[i], 1);
    }
    DPRINTF(IbvTestClient, "[test requester] ibv_post_send!\n");

    int sum = 0;
    struct cpl_desc **desc;
    for (int i = 0; i < 100; ++i) {
        // usleep(1000);

        desc = (struct cpl_desc **)malloc(sizeof(struct cpl_desc *) * MAX_CPL_NUM);
        for (int i = 0; i < MAX_CPL_NUM; ++i) {
            desc[i] = (struct cpl_desc *)malloc(sizeof(struct cpl_desc));
        }
        rtn = ibv->ibv_poll_cpl(res->cq[0], desc, MAX_CPL_NUM);

        DPRINTF(IbvTestClient, "[test requester] %d ibv_poll_cpl (CM) finish ! return is %d\n", i, rtn);

        if (rtn) {
            for (int j  = 0; j < rtn; ++j) {
                DPRINTF(IbvTestClient, "[test requester] ibv_poll_cpl (CM) finish! recv %d bytes, trans type is %d.\n", (*desc)[j].byte_cnt, (*desc)[j].trans_type);
            }
            sum += rtn;
            if (sum >= (1 * 2)) {
                break;
            }
        }
    }

    // for (int i = 0; i < res->num_wqe; ++i) {
    //     DPRINTF(IbvTestClient, "[test requester] CM Recv addr is 0x%lx, send data is : %s\n",
    //         (uint64_t)ctx.cm_mr->addr + 100 + i * 17, (char *)(ctx.cm_mr->addr + 100 + i * 17));
    // }

    return 0;
}


/* This function is compulsory */
IbvTestClient * IbvTestClientParams::create() {
    return new IbvTestClient(this);
}
