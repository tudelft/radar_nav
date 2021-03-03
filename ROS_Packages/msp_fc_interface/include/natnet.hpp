#pragma once

#include <thread>

class NatNet {
    private:
        std::thread natnet_thread_;      
    public:    
        NatNet();
        ~NatNet();
        void natnet_rx();
        void sample_data();
        void velocity_thread();
};