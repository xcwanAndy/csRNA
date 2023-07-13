#ifndef __IBV_TEST_H__
#define __IBV_TEST_H__

#include <stdio.h>
#include "debug/XDR.hh"
#include "params/IbvTest.hh"
#include "libibv.hh"

struct rem_info {
    uint64_t raddr;
    uint32_t rkey;
    uint32_t qpn;
    uint16_t dlid;
};

struct Resource {
    struct ibv_context ctx;;
    struct ibv_mr *mr;
    struct ibv_cq *cq;

    struct ibv_qp **qp;
    int num_qp;

    struct ibv_wqe **wr;
    int num_wqe;

    struct rem_info *rinfo; /* an struct pointer to store remote information */
};

class IbvTest : public SimObject {
    public:
        typedef IbvTestParams Params;
        const Params *params() const {
                return dynamic_cast<const Params *>(_params);
            }
        IbvTest(const Params *params);
        ~IbvTest(){};

        Ibv *ibv;
        EventFunctionWrapper mainEvent;

        char id_name[10];
        uint8_t  cpu_id;
        uint32_t num_client;

        struct ibv_wqe *init_rcv_wqe (struct ibv_mr* mr, int num);
        struct ibv_wqe *init_snd_wqe (struct ibv_mr* mr, uint32_t qkey, int num);
        struct ibv_wqe *init_rdma_write_wqe (struct Resource *res, struct ibv_mr* lmr, uint64_t raddr, uint32_t rkey);
        struct ibv_wqe *init_rdma_read_wqe (struct ibv_mr* req_mr, struct ibv_mr* rsp_mr, uint32_t qkey);
        void config_rc_qp(struct Resource *res);
        void config_ud_qp (struct ibv_qp* qp, struct ibv_cq *cq, struct ibv_context *ctx, uint32_t qkey);
        int exchange_rc_info();
        struct Resource *resc_init(uint16_t llid, int msg_size, int num_qp, int num_wqe);
        int main ();
};

#endif /* __IBV_TEST_H__ */
