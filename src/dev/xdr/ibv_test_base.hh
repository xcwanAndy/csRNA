#ifndef __IBV_TEST_BASE_H__
#define __IBV_TEST_BASE_H__

#include <queue>
#include <stdio.h>
#include <unordered_map>
#include "debug/XDR.hh"
#include "params/IbvTestBase.hh"
#include "libibv.hh"
#include "sim/eventq.hh"

/* used to store remote information */
struct rem_info {
    uint64_t raddr;
    uint32_t rkey;
    uint32_t qpn;
    uint16_t dlid;

    int start_off; /* qp start offset in rdma_resc */
    int sum; /* number of qp connected for one client */

    /**
     * == 1 if recved sync req
     */
    uint8_t sync_flag;
};
//struct rem_info {
    //uint64_t raddr;
    //uint32_t rkey;
    //uint32_t qpn;
    //uint16_t dlid;
//};

struct rdma_resc {
    struct ibv_context *ctx;

    uint8_t ibv_type;

    int num_mr;
    int num_cq;
    int num_qp; /* number of qps per client */
    int num_rem; /* number of remote client (or server) */
    int num_wqe;

    struct ibv_mr **mr;
    struct ibv_cq **cq;
    struct ibv_qp **qp;
    //struct ibv_cq *cq;

    struct rem_info *rinfo;

    // /**
    //  * true if recved sync req,
    //  * size is equal to num_rem.
    //  */
    // uint8_t *sync_flag;

    struct cpl_desc **desc;
};

/* Connection Request Type */
enum rdma_cr_type {
    CR_TYPE_NULL= (uint8_t)0x00,
    CR_TYPE_REQ = (uint8_t)0x01,
    CR_TYPE_ACK = (uint8_t)0x02,
    CR_TYPE_NAK = (uint8_t)0x04,
    CR_TYPE_RKEY= (uint8_t)0x10,
    CR_TYPE_SYNC= (uint8_t)0x20
};


/**
 * @note This struct is transmitted through QP0.
 *       It is known as Connection Requester.
 */
struct rdma_cr {

    enum rdma_cr_type flag;
    enum ibv_qp_type qp_type;

    uint16_t src_lid; /* client's LID */
    uint32_t src_qpn; /* client's QPN */
    uint32_t dst_qpn; /* server's QPN */

    uint32_t rkey; /* client's or server's MR rkey */
    uint64_t raddr; /* client's or server's remote Addr */

    /* Valid in RC trans type */
    uint32_t snd_psn; /* send PSN number in Requester */

    /* Valid in UD trans type */
    uint32_t src_qkey; /* qkey in Requester */
};


class IbvTestBase : public SimObject {
    public:
        typedef IbvTestBaseParams Params;
        const Params *params() const {
                return dynamic_cast<const Params *>(_params);
            }
        IbvTestBase(const Params *params);
        ~IbvTestBase(){};

        Ibv *ibv;
        struct rdma_resc *res;
        uint16_t clt_lid, svr_lid;

        struct cpl_desc **desc;
        struct cpl_desc **malloc_desc();

        //EventFunctionWrapper mainEvent;
        //int main ();

        EventFunctionWrapper pollCplEvent;
        void poll_cpl();
        std::unordered_set<struct ibv_cq *> cplWaitingList;

        EventFunctionWrapper rdmaOpEvent;

        char id_name[10];
        uint8_t  cpu_id;
        uint32_t num_client;
        bool has_rinfo;

        void rdma_connect(struct rdma_resc *resc, uint16_t svr_lid);
        void rdma_listen_pre();
        void rdma_listen_post(struct cpl_desc *desc);
        void rdma_op_pre();
        void rdma_op_post(struct cpl_desc *desc);
        struct ibv_wqe *init_rcv_wqe (struct ibv_context *ctx, int num);
        struct ibv_wqe *init_snd_wqe (struct ibv_context *ctx, struct rdma_cr *cr_info, int num, uint16_t dlid);
        struct ibv_wqe *init_rdma_write_wqe (struct ibv_mr* lmr);
        struct ibv_wqe *init_rdma_read_wqe (struct ibv_mr* mr);
        struct ibv_wqe *init_rdma_read_wqe (struct ibv_mr* req_mr, struct ibv_mr* rsp_mr, uint32_t qkey);
        void config_rc_qp();
        void config_ud_qp (struct ibv_qp* qp, struct ibv_cq *cq, struct ibv_context *ctx, uint32_t qkey);
        int exchange_rc_info(struct rdma_resc *resc, uint16_t svr_lid);
        void fill_read_mr(struct ibv_mr* mr);
        struct rdma_resc *resc_init(uint16_t llid, int num_qp, int num_mr, int num_cq, int num_wqe);
};

#endif /* __IBV_TEST_BASE_H__ */
