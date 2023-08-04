#ifndef __IBV_TEST_CLIENT_H__
#define __IBV_TEST_CLIENT_H__

#include "params/IbvTestClient.hh"
#include "accel.hh"

class IbvTestClient : public Accel {
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
