#include "ibv_test_server.hh"
#include "base/statistics.hh"


IbvTestServer::IbvTestServer(const Params *p)
    : IbvTestBase(p),
    mainEvent([this]{ main(); }, name())
{
    DPRINTF(IbvTestServer, "Initializing IbvTestServer\n");
    if (!mainEvent.scheduled()) {
        DPRINTF(IbvTestServer, "mainEvent is being scheduled ...\n");
        schedule(mainEvent, curTick() + 1000);
    }
} // IbvTestServer::IbvTestServer


/*****************************************************
 *********************** Entrance ********************
 *****************************************************/
int IbvTestServer::main () {
    int rtn;
    num_client = 1;
    clt_lid=0x11; /* client's MAC */
    svr_lid=0x22; /* server's MAC */
    sprintf(id_name, "%d", 999);

    int num_qp = 1, num_mr = 1, num_cq = 1;
    res = resc_init(clt_lid, num_qp, num_mr, num_cq);

    DPRINTF(IbvTestServer, "server main function executing ...\n");

    /********************* receive ********************/
    struct ibv_wqe *recv_wqe = init_rcv_wqe(res->ctx, RCV_WR_MAX);
    ibv->ibv_post_recv(res->ctx, recv_wqe, res->ctx->cm_qp, RCV_WR_MAX);

    //rdma_connect(res, svr_lid);

    rdma_listen_pre();

    return 0;

    int sum = 0;
    struct cpl_desc **desc;
    for (int i = 0; i < 100; ++i) {
        // usleep(1000);

        desc = (struct cpl_desc **)malloc(sizeof(struct cpl_desc *) * MAX_CPL_NUM);
        for (int i = 0; i < MAX_CPL_NUM; ++i) {
            desc[i] = (struct cpl_desc *)malloc(sizeof(struct cpl_desc));
        }
        rtn = ibv->ibv_poll_cpl(res->cq[0], desc, MAX_CPL_NUM);

        DPRINTF(IbvTestServer, "[test requester] %d ibv_poll_cpl (CM) finish ! return is %d\n", i, rtn);

        if (rtn) {
            for (int j  = 0; j < rtn; ++j) {
                DPRINTF(IbvTestServer, "[test requester] ibv_poll_cpl (CM) finish! recv %d bytes, trans type is %d.\n", (*desc)[j].byte_cnt, (*desc)[j].trans_type);
            }
            sum += rtn;
            if (sum >= (1 * 2)) {
                break;
            }
        }
    }

    // for (int i = 0; i < res->num_wqe; ++i) {
    //     DPRINTF(IbvTestServer, "[test requester] CM Recv addr is 0x%lx, send data is : %s\n",
    //         (uint64_t)ctx.cm_mr->addr + 100 + i * 17, (char *)(ctx.cm_mr->addr + 100 + i * 17));
    // }

    return 0;
}


/* This function is compulsory */
IbvTestServer * IbvTestServerParams::create() {
    return new IbvTestServer(this);
}
