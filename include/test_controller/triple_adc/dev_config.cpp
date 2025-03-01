#include "test_controller/triple_adc/dev_config.h"
#include <fcntl.h>
#include <time.h>
#include <errno.h>


/**
 * delay x ms
**/
void DEV_Delay_ms(UDOUBLE msec)
{
    struct timespec ts;
    int res;

    if (msec < 0)
    {
        errno = EINVAL;
    }

    ts.tv_sec = msec / 1000;
    ts.tv_nsec = (msec % 1000) * 1000000;

    do {
        res = nanosleep(&ts, &ts);
    } while (res && errno == EINTR);

}

