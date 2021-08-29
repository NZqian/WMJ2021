#pragma once
#include "WMJProtocol.h"

namespace wmj
{
    class Port
    {
    public:
        Port() {}
        virtual ~Port() {}
        virtual bool sendFrame(Buffer&) = 0 ;
        virtual Buffer readFrame(int) = 0 ;
        static void checkPort();
    private:
    };
}
