#ifndef __IBV_TEST_SERVER_H__
#define __IBV_TEST_SERVER_H__

#include "params/IbvTestServer.hh"
#include "ibv_test_base.hh"

class IbvTestServer : public IbvTestBase {
    public:
        typedef IbvTestServerParams Params;
        const Params *params() const {
                return dynamic_cast<const Params *>(_params);
            }
        IbvTestServer(const Params *params);
        ~IbvTestServer(){};

        EventFunctionWrapper mainEvent;
        int main();

        //EventFunctionWrapper rdmaWriteEvent;

        //char id_name[10];
        //uint8_t  cpu_id;
        //uint32_t num_client;

};

#endif /* __IBV_TEST_SERVER_H__*/
