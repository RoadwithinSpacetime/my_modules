#include "../mp_sdk_audio.h"
#include "CyclePeakLookahead.h"

SE_DECLARE_INIT_STATIC_FILE(CyclePeakLookahead);

namespace se_sdk_init
{
    void InitDll()
    {
        REGISTER_PLUGIN(CyclePeakLookahead, L"CyclePeakLookahead");
    }
}
