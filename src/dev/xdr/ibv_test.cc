#include "ibv_test.hh"
#include "base/statistics.hh"

IbvTest::IbvTest(const Params *p)
    : SimObject(p),
    ibv(p->ibv),
    mainEvent([this]{ main(); }, name())
{
        DPRINTF(IbvTest, "Initializing IbvTest\n");
    if (!mainEvent.scheduled()) {
        DPRINTF(IbvTest, "mainEvent is being scheduled ...\n");
        schedule(mainEvent, curTick() + 1000);
    }
} // IbvTest::IbvTest

struct ibv_wqe *IbvTest::init_rcv_wqe (struct ibv_mr* mr, int num) {
    struct ibv_wqe *wqe = (struct ibv_wqe *)malloc(sizeof(struct ibv_wqe) * num);

    for (int i = 0; i < num; ++i) {
        /* Write data to mr */
        uint32_t offset = 100 + i * 17;
        char *string = (char *)(mr->addr + offset);
        DPRINTF(IbvTest, "[test requester] init_rcv_wqe: string is %s, string vaddr is 0x%lx, start vaddr is 0x%lx\n", string, (uint64_t)string, (uint64_t)(mr->addr + offset));
        DPRINTF(IbvTest, "[test requester] init_rcv_wqe: data[0] is %c, data[1] is %c\n", (char)*(mr->addr + offset), (char)*(mr->addr + offset + 1));


        wqe[i].length = 17;
        wqe[i].mr = mr;
        wqe[i].offset = offset;

        DPRINTF(IbvTest, "[test requester] init_rcv_wqe: len is %d\n", wqe[i].length);

        /* Add rcv element */
        wqe[i].trans_type = IBV_TYPE_RECV;
    }

    return wqe;

}

struct ibv_wqe *IbvTest::init_snd_wqe (struct ibv_mr* mr, uint32_t qkey, int num) {

    struct ibv_wqe *wqe = (struct ibv_wqe *)malloc(sizeof(struct ibv_wqe) * num);

#define TRANS_SEND_DATA "hello RDMA Send!"

    // Write data to mr
    uint32_t offset = 0;
    char *string = (char *)(mr->addr + offset);
    memcpy(string, TRANS_SEND_DATA, sizeof(TRANS_SEND_DATA));

    DPRINTF(IbvTest, "[test requester] init_snd_wqe: string is %s, string vaddr is 0x%lx, start vaddr is 0x%lx\n",
            string, (uint64_t)string, (uint64_t)(mr->addr + offset));

    for (int i = 0; i < num; ++i) {
        wqe[i].length = sizeof(TRANS_SEND_DATA);
        DPRINTF(IbvTest, "[test requester] init_snd_wqe: len is %d\n", wqe[i].length);
        wqe[i].mr = mr;
        wqe[i].offset = offset;

        /* Add Send element */
        wqe[i].trans_type = IBV_TYPE_SEND;
        wqe[i].send.dlid = 0x01;
        wqe[i].send.dqpn = 0;
        wqe[i].send.qkey = qkey;
    }


    return wqe;
}

struct ibv_wqe *IbvTest::init_rdma_write_wqe (struct Resource *res, struct ibv_mr* lmr, uint64_t raddr, uint32_t rkey) {

    struct ibv_wqe *wqe = (struct ibv_wqe *)malloc(sizeof(struct ibv_wqe) * res->num_wqe);

#define RDMA_WRITE_DATA "hello RDMA Write!"

    // Write data to mr
    uint32_t offset = 0;
    char *string = (char *)(lmr->addr + offset);
    memcpy(string, RDMA_WRITE_DATA, sizeof(RDMA_WRITE_DATA));
    DPRINTF(IbvTest, "[test requester] init_snd_wqe: string is %s, string vaddr is 0x%lx, start vaddr is 0x%lx\n",
                string, (uint64_t)string, (uint64_t)(lmr->addr + offset));

    for (int i = 0; i < res->num_wqe; ++i, wqe = (wqe + 1)) {

        wqe->length = sizeof(RDMA_WRITE_DATA);
        wqe->mr = lmr;
        wqe->offset = offset;

        // Add RDMA Write element
        wqe->trans_type = IBV_TYPE_RDMA_WRITE;
        wqe->rdma.raddr = raddr;
        wqe->rdma.rkey  = rkey;
    }
    return wqe;
}

struct ibv_wqe *IbvTest::init_rdma_read_wqe (struct ibv_mr* req_mr, struct ibv_mr* rsp_mr, uint32_t qkey) {

    struct ibv_wqe *wqe = (struct ibv_wqe *)malloc(sizeof(struct ibv_wqe));

#define TRANS_RRDMA_DATA "hello RDMA Read!"

    // Write data to mr
    uint32_t offset = 0;
    char *string = (char *)(rsp_mr->addr + offset);
    memcpy(string, TRANS_RRDMA_DATA, sizeof(TRANS_RRDMA_DATA));

    DPRINTF(IbvTest, "[test requester] init_snd_wqe: string is %s, string vaddr is 0x%lx, start vaddr is 0x%lx\n",
            string, (uint64_t)string, (uint64_t)(rsp_mr->addr + offset));

    wqe->length = sizeof(TRANS_RRDMA_DATA);
    DPRINTF(IbvTest, "[test requester] init_snd_wqe: len is %d, addr: 0x%lx, key: %x\n", wqe->length, (uint64_t)rsp_mr->addr, rsp_mr->lkey);
    wqe->mr = req_mr;
    wqe->offset = offset;

    // Add RDMA Write element
    wqe->trans_type = IBV_TYPE_RDMA_READ;
    wqe->rdma.raddr = (uint64_t)rsp_mr->addr;
    wqe->rdma.rkey  = rsp_mr->lkey;

    // DPRINTF(IbvTest, "[test requester] init_snd_wqe: rKey: 0x%x, rAddr: 0x%lx\n", wqe->rdma.rkey, wqe->rdma.raddr);
    return wqe;
}

void IbvTest::config_rc_qp(struct Resource *res) {

    for (int i = 0; i < res->num_qp; ++i) {
        res->qp[i]->ctx = &(res->ctx);
        res->qp[i]->flag = 0;
        res->qp[i]->type = QP_TYPE_RC;
        res->qp[i]->cq = res->cq;
        res->qp[i]->snd_wqe_offset = 0;
        res->qp[i]->rcv_wqe_offset = 0;
        res->qp[i]->lsubnet.llid = res->ctx.lid;

        res->qp[i]->dest_qpn = res->rinfo->qpn;
        res->qp[i]->snd_psn = 0;
        res->qp[i]->ack_psn = 0;
        res->qp[i]->exp_psn = 0;
        res->qp[i]->dsubnet.dlid = res->rinfo->dlid;

        ibv->ibv_modify_qp(&(res->ctx), res->qp[i]);
        DPRINTF(IbvTest, "[test requester] ibv_modify_qp end!\n");
    }
}


void IbvTest::config_ud_qp (struct ibv_qp* qp, struct ibv_cq *cq, struct ibv_context *ctx, uint32_t qkey) {
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

int IbvTest::exchange_rc_info() {
    return 0;
}

struct Resource *IbvTest::resc_init(uint16_t llid, int msg_size, int num_qp, int num_wqe) {
    struct Resource *res = (struct Resource *)malloc(sizeof(struct Resource));

    res->num_qp = num_qp;
    res->num_wqe = num_wqe;

    ibv->ibv_open_device(&(res->ctx), llid);
    DPRINTF(IbvTest, "[test requester] ibv_open_device End. Doorbell addr 0x%lx\n", (long int)res->ctx.dvr);

    struct ibv_mr_init_attr mr_attr;
    mr_attr.length = msg_size;
    mr_attr.flag = (enum ibv_mr_flag)(MR_FLAG_RD | MR_FLAG_WR | MR_FLAG_LOCAL | MR_FLAG_REMOTE);
    res->mr = ibv->ibv_reg_mr(&(res->ctx), &mr_attr);
    DPRINTF(IbvTest, "[test requester] ibv_reg_mr End! lkey %d, vaddr 0x%lx\n", res->mr->lkey, (uint64_t)res->mr->addr);

    struct ibv_cq_init_attr cq_attr;
    cq_attr.size_log = 12;
    struct ibv_cq * cq = ibv->ibv_create_cq(&(res->ctx), &cq_attr);
    DPRINTF(IbvTest, "[test requester] ibv_create_cq End! cqn %d\n", cq->cq_num);

    struct ibv_qp_create_attr qp_attr;
    qp_attr.sq_size_log = 12;
    qp_attr.rq_size_log = 12;
    struct ibv_qp * qp = ibv->ibv_create_qp(&(res->ctx), &qp_attr);
    DPRINTF(IbvTest, "[test requester] ibv_create_qp end! qpn %d\n", qp->qp_num);
    return res;
}

int IbvTest::main () {
    int rtn;
    char is_server = 0;
    uint16_t llid=100, dlid=200;

    num_client = 1;
    cpu_id     = 0;
    sprintf(id_name, "%d", 999);

    struct Resource *res = resc_init(llid, 1, 1, 1);

    DPRINTF(IbvTest, "main function executing ...\n");

    exchange_rc_info();

    config_rc_qp(res);

    struct ibv_wqe *wrdma_wqe = init_rdma_write_wqe(res, res->mr, res->rinfo->raddr, res->rinfo->rkey);

    for (int i = 0; i < res->num_qp; ++i) {
        ibv->ibv_post_send(&(res->ctx), wrdma_wqe, res->qp[i], res->num_wqe);
    }
    DPRINTF(IbvTest, "[test requester] ibv_post_send!\n");

    int sum = 0;
    struct cpl_desc **desc;
    for (int i = 0; i < 100; ++i) {
        // usleep(1000);

        desc = (struct cpl_desc **)malloc(sizeof(struct cpl_desc *) * MAX_CPL_NUM);
        rtn = ibv->ibv_poll_cpl(res->cq, desc, MAX_CPL_NUM);

        DPRINTF(IbvTest, "[test requester] %d ibv_poll_cpl (CM) finish ! return is %d\n", i, rtn);

        if (rtn) {
            for (int j  = 0; j < rtn; ++j) {
                DPRINTF(IbvTest, "[test requester] ibv_poll_cpl (CM) finish! recv %d bytes, trans type is %d.\n", (*desc)[j].byte_cnt, (*desc)[j].trans_type);
            }
            sum += rtn;
            if (sum >= (res->num_wqe * 2)) {
                break;
            }
        }
    }

    // for (int i = 0; i < res->num_wqe; ++i) {
    //     DPRINTF(IbvTest, "[test requester] CM Recv addr is 0x%lx, send data is : %s\n",
    //         (uint64_t)ctx.cm_mr->addr + 100 + i * 17, (char *)(ctx.cm_mr->addr + 100 + i * 17));
    // }

    return 0;

}

/* This function is compulsory */
IbvTest * IbvTestParams::create() {
    return new IbvTest(this);
}
