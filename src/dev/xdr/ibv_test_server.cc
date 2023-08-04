#include "ibv_test_server.hh"
#include "base/statistics.hh"


IbvTestServer::IbvTestServer(const Params *p)
    : Accel(p),
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
    num_client = 1;
    clt_lid=0x11; /* client's MAC */
    svr_lid=0x22; /* server's MAC */
    sprintf(id_name, "%d", 999);

    int num_qp = 1, num_mr = 1, num_cq = 1, num_wqe = 1;
    /* The first parameter is local lid */
    res = resc_init(svr_lid, num_qp, num_mr, num_cq, num_wqe);

    res->ibv_type = IBV_TYPE_RDMA_WRITE;
    //res->ibv_type = IBV_TYPE_RDMA_READ;

    DPRINTF(IbvTestServer, "server main function executing ...\n");

    /********************* receive ********************/
    struct ibv_wqe *recv_wqe = init_rcv_wqe(res->ctx, RCV_WR_MAX);
    ibv->ibv_post_recv(res->ctx, recv_wqe, res->ctx->cm_qp, RCV_WR_MAX);

    //rdma_connect(res, svr_lid);

    rdma_listen_pre();

    return 0;
}


/* This function is compulsory */
IbvTestServer * IbvTestServerParams::create() {
    return new IbvTestServer(this);
}
