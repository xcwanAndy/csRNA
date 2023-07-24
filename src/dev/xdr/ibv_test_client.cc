#include "ibv_test_client.hh"
#include "base/statistics.hh"
#include "base/trace.hh"
#include "dev/xdr/libibv.hh"
#include <queue>

IbvTestClient::IbvTestClient(const Params *p)
    : IbvTestBase(p),
    mainEvent([this]{ main(); }, name())
{
    DPRINTF(IbvTestClient, "Initializing IbvTestClient\n");
    if (!mainEvent.scheduled()) {
        DPRINTF(IbvTestClient, "mainEvent is being scheduled ...\n");
        schedule(mainEvent, curTick() + 1000);
    }
} // IbvTestClient::IbvTestClient



/*****************************************************
 *********************** Entrance ********************
 *****************************************************/
int IbvTestClient::main () {
    int rtn;
    num_client = 1;
    clt_lid=0x11; /* client's MAC */
    svr_lid=0x22; /* server's MAC */
    sprintf(id_name, "%d", 999);

    int num_qp = 1, num_mr = 1, num_cq = 1;
    res = resc_init(clt_lid, num_qp, num_mr, num_cq);

    DPRINTF(IbvTestClient, "main function executing ...\n");

    /********************* receive ********************/
    struct ibv_wqe *recv_wqe = init_rcv_wqe(res->ctx, RCV_WR_MAX);
    ibv->ibv_post_recv(res->ctx, recv_wqe, res->ctx->cm_qp, RCV_WR_MAX);

    rdma_connect(res, svr_lid);

    config_rc_qp();

    return 0;

    /********************* write ********************/
    struct ibv_wqe *wrdma_wqe = init_rdma_write_wqe(res->mr[0]);
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
