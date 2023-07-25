#ifndef __IBV_TEST_CLIENT_H__
#define __IBV_TEST_CLIENT_H__

#include "params/IbvTestClient.hh"
#include "ibv_test_base.hh"

class IbvTestClient : public IbvTestBase {
    public:
        typedef IbvTestClientParams Params;
        const Params *params() const {
                return dynamic_cast<const Params *>(_params);
            }
        IbvTestClient(const Params *params);
        ~IbvTestClient(){};

        EventFunctionWrapper mainEvent;
        int main ();

        EventFunctionWrapper loopEvent;
        void loop();
};

#endif /* __IBV_TEST_CLIENT_H__ */
