#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/exception.hpp>

#include "usrp_utils.h"

// adds a double offset to a uhd::time_spec_t
// useful for calculating pulse times for a pulse sequence
uhd::time_spec_t offset_time_spec(uhd::time_spec_t t0, double toffset)
{
    uhd::time_spec_t t1;
    
    time_t full_sec = t0.get_full_secs();
    double frac_sec = t0.get_frac_secs();

    frac_sec += toffset;
    
    full_sec += floor(frac_sec);
    frac_sec -= floor(frac_sec);


    t1 = uhd::time_spec_t(full_sec, frac_sec);
    
    return t1;
}
