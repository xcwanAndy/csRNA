#ifndef __TEST_H__
#define __TEST_H__

#include <stdio.h>
#include "libhgrnic.h"

extern char id_name[10];
extern uint8_t  cpu_id;
extern uint32_t num_client;

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

#endif /* __TEST_H__ */