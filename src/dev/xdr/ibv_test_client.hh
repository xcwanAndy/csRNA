#ifndef __IBV_TEST_CLIENT_H__
#define __IBV_TEST_CLIENT_H__

#include "debug/XDR.hh"
#include "params/IbvTestClient.hh"
#include "sim/eventq.hh"
#include "ibv_test_base.hh"

class IbvTestClient : public IbvTestBase {
    public:
        typedef IbvTestClientParams Params;
        const Params *params() const {
                return dynamic_cast<const Params *>(_params);
            }
        IbvTestClient(const Params *params);
        ~IbvTestClient(){};

        //Ibv *ibv;
        //struct rdma_resc *res;
        //uint16_t clt_lid, svr_lid;

        //struct cpl_desc **desc;
        //struct cpl_desc **malloc_desc();

        EventFunctionWrapper mainEvent;
        int main ();

        //EventFunctionWrapper pollCplEvent;
        //void poll_cpl();
        //std::unordered_set<struct ibv_cq *> cplWaitingList;

        ////EventFunctionWrapper readCplEvent;
        ////void readCpl();

        //char id_name[10];
        //uint8_t  cpu_id;
        //uint32_t num_client;

        //void rdma_connect(struct rdma_resc *resc, uint16_t svr_lid);
        //void rdma_listen_pre();
        //void rdma_listen_post(struct cpl_desc *desc);
        //struct ibv_wqe *init_rcv_wqe (struct ibv_mr* mr, int num);
        //struct ibv_wqe *init_rcv_wqe (struct ibv_context *ctx, int num);
        //struct ibv_wqe *init_snd_wqe (struct ibv_context *ctx, struct rdma_cr *cr_info, int num, uint16_t dlid);
        //struct ibv_wqe *init_rdma_write_wqe (struct rdma_resc *res, struct ibv_mr* lmr, uint64_t raddr, uint32_t rkey);
        //struct ibv_wqe *init_rdma_read_wqe (struct ibv_mr* req_mr, struct ibv_mr* rsp_mr, uint32_t qkey);
        //void config_rc_qp(struct rdma_resc *res);
        //void config_ud_qp (struct ibv_qp* qp, struct ibv_cq *cq, struct ibv_context *ctx, uint32_t qkey);
        //int exchange_rc_info(struct rdma_resc *resc, uint16_t svr_lid);
        //struct rdma_resc *resc_init(uint16_t llid, int num_qp, int num_mr, int num_cq);
};

#endif /* __IBV_TEST_CLIENT_H__ */
