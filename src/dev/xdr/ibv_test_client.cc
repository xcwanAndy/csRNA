#include "ibv_test_client.hh"
#include "base/statistics.hh"
#include "base/trace.hh"

IbvTestClient::IbvTestClient(const Params *p)
    : IbvTestBase(p),
    mainEvent([this]{ main(); }, name()),
    loopEvent([this] { loop(); }, name())
{
    DPRINTF(IbvTestClient, "Initializing IbvTestClient\n");
    if (!mainEvent.scheduled()) {
        DPRINTF(IbvTestClient, "mainEvent is being scheduled ...\n");
        schedule(mainEvent, curTick() + 1000);
    }
} // IbvTestClient::IbvTestClient

void IbvTestClient::loop() {
    if (!loopEvent.scheduled()) {
        schedule(loopEvent, curTick() + 2000);
    }
}

/*****************************************************
 *********************** Entrance ********************
 *****************************************************/
int IbvTestClient::main () {
    num_client = 1;
    clt_lid=0x11; /* client's MAC */
    svr_lid=0x22; /* server's MAC */
    sprintf(id_name, "%d", 999);

    int num_qp = 1, num_mr = 1, num_cq = 1, num_wqe = 1;
    res = resc_init(clt_lid, num_qp, num_mr, num_cq, num_wqe);

    fill_read_mr(res->mr[0]);

    DPRINTF(IbvTestClient, "main function executing ...\n");

    /********************* receive ********************/
    struct ibv_wqe *recv_wqe = init_rcv_wqe(res->ctx, RCV_WR_MAX);
    ibv->ibv_post_recv(res->ctx, recv_wqe, res->ctx->cm_qp, RCV_WR_MAX);

    rdma_connect(res, svr_lid);

    res->rinfo->dlid = svr_lid; /* Set mannually */
    res->rinfo->qpn = 1; /* Set mannually for server res->qp[0] */

    config_rc_qp();

    if (!loopEvent.scheduled()) {
        schedule(loopEvent, curTick() + 2000);
    }

    return 0;
}


/* This function is compulsory */
IbvTestClient * IbvTestClientParams::create() {
    return new IbvTestClient(this);
}
