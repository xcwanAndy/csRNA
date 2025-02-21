/*
 *======================= START OF LICENSE NOTICE =======================
 *  Copyright (C) 2021 Kang Ning, NCIC, ICT, CAS.
 *  All Rights Reserved.
 *
 *  NO WARRANTY. THE PRODUCT IS PROVIDED BY DEVELOPER "AS IS" AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL DEVELOPER BE LIABLE FOR
 *  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 *  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THE PRODUCT, EVEN
 *  IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *======================== END OF LICENSE NOTICE ========================
 *  Primary Author: Kang Ning
 *  <kangning18z@ict.ac.cn>
 *
 *  Copyright (C) 2023 Xinyu Yang, HKUST.
 */


#ifndef __LIBIBV_HH__
#define __LIBIBV_HH__

#include "dev/xdr/accel_defs.hh"
#include "dev/rdma/hangu_rnic.hh"
#include "dev/rdma/kfd_ioctl.h"
#include "dev/xdr/nic_ctrl.hh"

#include "base/inet.hh"
#include "dev/pci/device.hh"
#include "sim/eventq.hh"
#include "sim/sim_object.hh"
#include "sim/syscall_emul_buf.hh"
#include "params/NicCtrl.hh"
#include "params/Ibv.hh"
#include <cstdint>

extern char id_name[10];
extern uint8_t  cpu_id;
extern uint32_t num_client;

/* -------Useful API{begin}-------- */
/* Useful defines */
#define QKEY_CM 0xFF

// CM QP realted parameter
#define RCV_WR_BASE 0
#define SND_WR_BASE 3000
#define RCV_WR_MAX 70
#define __SND_WR_MAX ((RCV_WR_MAX / num_client) - 1)
#define SND_WR_MAX ( ( __SND_WR_MAX < 25 ) ? __SND_WR_MAX : 25 )

#define MAX_CPL_NUM 100


/* --------Useful API{end}-------- */


/* -------Interact with device{begin}------- */
/**
 * All the code in this block needs to hold 
 * consistance with hangu_rnic_def.hh
 */

/* Duplicate with hangu_rnic_def
 */
// We just implement RC and UD
//enum ibv_qp_type {
    //QP_TYPE_RC,
    //QP_TYPE_UC,
    //QP_TYPE_RD,
    //QP_TYPE_UD
//};

enum ibv_mr_flag {
    MR_FLAG_RD     = (1 << 0),
    MR_FLAG_WR     = (1 << 1),
    MR_FLAG_LOCAL  = (1 << 2),
    MR_FLAG_REMOTE = (1 << 3)
};

// Command Opcode for Doorbell command
enum ibv_trans_func {
    IBV_TYPE_NULL       = (uint8_t)0x00,
    IBV_TYPE_SEND       = (uint8_t)0x01,
    IBV_TYPE_RECV       = (uint8_t)0x02,
    IBV_TYPE_RDMA_WRITE = (uint8_t)0x03,
    IBV_TYPE_RDMA_READ  = (uint8_t)0x04,
};

/* This one has been defined in hangu_rnic_def.h */
//enum ibv_wqe_flags {
    //WR_FLAG_SIGNALED  = (1 << 31),
//};


struct Doorbell {
    uint8_t  opcode;
    uint8_t  num;
    uint32_t qpn;
    uint32_t offset;
};

// Send descriptor struct
struct send_desc {
    /* data unit */
    uint32_t len;
    uint32_t lkey;
    uint64_t lVaddr;

    union {
        struct {
            uint32_t dlid;
            uint32_t qkey;
            uint32_t dest_qpn;
        } send_type; /* UD unit */
        struct {
            uint32_t rkey;
            uint32_t rVaddr_l;
            uint32_t rVaddr_h;
        } rdma_type; /* RDMA unit */
    };

    /* Base unit */
    // uint8_t opcode;
    /**
     * @brief Base unit, inncluding wqe flags & opcode
     * ------------------------------
     * |    31    | 30 - 8 | 7 : 0  |
     * | signaled |        | opcode |
     * ------------------------------
     * signaled :  enum ibv_wqe_flags
     * opcode   :  enum ibv_trans_func
     */
    union {
        uint32_t flags;
        uint8_t opcode;
    };
};

// Receive Descriptor struct
struct recv_desc {
    uint32_t len;
    uint32_t lkey;
    uint64_t lVaddr;
};

// Completion Descriptor struct
struct cpl_desc {
    uint8_t  srv_type  ; // enum ibv_qp_type
    uint8_t  trans_type; // enum ibv_trans_func
    uint16_t byte_cnt  ;
    uint32_t qp_num    ;
    uint32_t cq_num    ;
};
/* -------Interact with device{end}------- */


/* -------Interact with kernel upper layer verbs lib{begin}------- */
struct ibv_context {

    uint16_t   lid;  /* Local lid */

    void *dvr; // User id of this user

    /* memory for Communication management */
    struct ibv_mr *cm_mr;
    struct ibv_cq *cm_cq;
    struct ibv_qp *cm_qp;

    uint32_t cm_snd_off;

    uint32_t cm_rcv_posted_off; /* non-received offset */
    uint32_t cm_rcv_acked_off; /* Posted offset */
    uint32_t cm_rcv_num; /* number of RCV WR posted to cm_qp, outstanding */

};

struct ibv_mtt {
    uint32_t mtt_index;
    void    *vaddr;
    uint64_t paddr;
};

struct ibv_mr {
    struct ibv_context *ctx; /* What does it used for? */

    uint32_t         lkey   ; /* aka. mpt index */
    enum ibv_mr_flag flag   ;
    uint8_t          *addr  ;
    uint64_t         length ;
    uint32_t         num_mtt; /* Number of MTT struct for this MR */
    struct ibv_mtt  *mtt;
};

struct ibv_qp {

    struct ibv_context *ctx; /* not used now */

    // Basic QP attribute and state
    uint32_t         qp_num; // Local qpn, allocated by ibv_create_qp
    uint8_t          flag  ; // not used now.
    enum ibv_qp_type type  ; // type of QP

    struct ibv_cq *cq;

    // Queue relevant
    struct ibv_mr *snd_mr; // (Fixed at 4KB now) allocated by ibv_create_qp
    struct ibv_mr *rcv_mr; // (Fixed at 4KB now) allocated by ibv_create_qp
    uint16_t       snd_wqe_offset;
    uint16_t       rcv_wqe_offset;

    // subnet ID
    union {
        uint16_t llid; // Local LID
        uint64_t lmac; // Local MAC, unused now
    } lsubnet;

    // For RC type
    uint32_t dest_qpn; // Dest qpn (useful only in RC type)
    uint32_t snd_psn ; // next send psn
    uint32_t ack_psn ; // last acked psn
    uint32_t exp_psn ; // next receive (expect) psn
    union {
        uint16_t dlid ; // Dest  LID
        uint64_t dmac ; // Dest  MAC, unused now
    } dsubnet;

    // For UD type
    uint32_t qkey;
};


struct ibv_cq {
    struct ibv_context *ctx; // What does it used for?

    uint32_t       cq_num;

    struct ibv_mr *mr    ; // (Fixed at 4KB now)
    uint32_t       offset; // The offset of the CQ

    uint32_t       cpl_cnt;
};


struct ibv_wqe {
    enum ibv_trans_func trans_type;

    /**
     * @note WQE attribute
     * ---------------------
     * |    31    | 30 : 0 |
     * | signaled |        |
     * ---------------------
     * signaled: 1 means post cpl to cq when completion,
     *           only valid in send/rdma write/rdma read
     */
    uint32_t flag; /* enum ibv_wqe_flags */

    /**
     * This mr is the data to be transed for send / RDMA write.
     * This mr is the space to recv data for Recv / RDMA Read.
     */
    struct ibv_mr *mr;
    uint32_t offset; // local mr addr offset
    uint32_t length; // actual length of transform data

    union {
        struct {
            uint32_t dlid;
            uint32_t qkey;
            uint32_t dqpn;
        } send; /* For UD Send */

        struct {
            uint32_t rkey ;
            uint64_t raddr; // remote virtual addr
        } rdma; /* For RC RDMA Write/Read */
    };
};


struct ibv_cq_init_attr {
    uint32_t         size_log; /* Cq size in log(in byte), 1 page maximum */
};

struct ibv_qp_create_attr {
    uint32_t         sq_size_log; /* SQ size in log(in byte), 1 page maximum */
    uint32_t         rq_size_log; /* SQ size in log(in byte), 1 page maximum */
};

struct ibv_mr_init_attr {
    enum ibv_mr_flag flag  ;
    uint64_t         length; /* 0-4096(in bytes) now */
    uint64_t         vaddr;
    uint64_t         paddr;
};
/* -------Interact with kernel upper layer verbs lib{end}------- */


/* -------Interact with kernel space driver{begin}------- */

#define KERNEL_FILE_NAME "/dev/hangu_rnic"

#define DB_LEN 0x1000

#define PAGE_SIZE_LOG 12
#define PAGE_SIZE (1 << PAGE_SIZE_LOG)

struct hghca_context {
    //uint32_t fd; // kernel file handler
    volatile Addr doorbell; // doorbell address
    volatile Addr sync; // address to sync
};


struct hghca_mtt {
    uint32_t mtt_index;
    uint64_t vAddr;
    uint64_t pAddr;
};

struct hghca_mpt {
    uint32_t mpt_index; // Key in other words
    // uint32_t key;
    uint32_t flag;
    uint64_t startVAddr;
    uint64_t length;
    uint64_t mtt_start;
};

struct hghca_qp {
    uint8_t  flag; // not used now.
    uint8_t  type;


    uint16_t local_lid; // Local LID
    uint32_t local_qpn; // Local qpn

    uint32_t cq_num;

    // For RC only
    uint16_t dest_lid ; // Dest  LID
    uint32_t dest_qpn ; // Dest qpn
    uint32_t snd_psn; // next send psn
    uint32_t ack_psn; // last acked psn
    uint32_t exp_psn; // next receive (expect) psn

    // Queue relevant
    uint16_t snd_wqe_offset;
    uint32_t snd_wqe_lkey  ;
    uint64_t snd_wqe_base  ;
    uint16_t rcv_wqe_offset;
    uint32_t rcv_wqe_lkey  ;
    uint64_t rcv_wqe_base  ;

    // !TODO We don't implement now
    uint8_t  sq_sz_log; // The size of SQ (It is now fixed at 4KB)
    uint8_t  rq_sz_log; // The size of RQ (It is now fixed at 4KB)


    uint32_t qkey;
};


struct hghca_cq {
    uint32_t cq_num;
    uint32_t offset; // The offset of the CQ
    uint32_t lkey;   // lkey of the CQ

    // !TODO We don't implement it now.
    uint32_t sz_log; // The size of CQ. (It is now fixed at 4KB)
};


class Ibv : public SimObject{
    public:
        typedef IbvParams Params;
        const Params *params() const {
                return dynamic_cast<const Params *>(_params);
            }

        Ibv(const Params *params);
        ~Ibv();

        bool is_onpath;
        NicCtrl *nicCtrl;

        MemAllocator *memAlloc;
        MemAllocator *dataMRAlloc;

        typedef struct {
            Addr addr;
            size_t size;
            uint64_t doorbell;
        } DbellElem;
        std::queue<DbellElem> dbellFifo;
        EventFunctionWrapper sendDooebellEvent;
        void sendDoorbell();

        EventFunctionWrapper waitMailReplyEvent;
        void waitMailReply();

        uint8_t write_cmd(unsigned long request, void *args);

        /* Infiniband verbs
         */
        int ibv_modify_batch_qp(struct ibv_context *context, struct ibv_qp *qp, uint32_t batch_size);
        struct ibv_qp * ibv_create_batch_qp(struct ibv_context *context, struct ibv_qp_create_attr *qp_attr, uint32_t batch_size);
        struct ibv_mr * ibv_reg_batch_mr(struct ibv_context *context, struct ibv_mr_init_attr *mr_attr, uint32_t batch_size);

        int ibv_open_device(struct ibv_context *context, uint16_t llid);
        struct ibv_cq * ibv_create_cq(struct ibv_context *context, struct ibv_cq_init_attr *cq_attr);
        struct ibv_qp * ibv_create_qp(struct ibv_context *context, struct ibv_qp_create_attr *qp_attr);
        int ibv_modify_qp(struct ibv_context *context, struct ibv_qp *qp);
        struct ibv_mr * ibv_reg_mr(struct ibv_context *context, struct ibv_mr_init_attr *mr_attr, bool is_onpath=true, bool is_datamr=false);
        int ibv_post_send(struct ibv_context *context, struct ibv_wqe *wqe, struct ibv_qp *qp, uint8_t num);
        int ibv_post_recv(struct ibv_context *context, struct ibv_wqe *wqe, struct ibv_qp *qp, uint8_t num);

        int ibv_poll_cpl(struct ibv_cq *cq, struct cpl_desc **desc, int max_num);

        int cpu_sync(struct ibv_context *context);

        void trans_wait(struct ibv_context *context);


};

#endif //__LIBIBV_HH__
