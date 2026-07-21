/*
 * nc_acq_policy.c — subscription-gated acquisition demand.
 * Rules documented in nc_acq_policy.h (handoff §5.4, DECIDED).
 */
#include "narbis/nc_acq_policy.h"
#include "narbis/proto.h"   /* NC_STREAM_MASK_* */

void nc_acq_eval(const nc_acq_in_t *in, nc_acq_out_t *out)
{
    out->ppg_on = false;
    out->accel_on = false;
    out->dsp_on = false;

    /* stream_on_usb=0 while USB present: hard-off, beats everything */
    if (!in->usb_stream_ok) return;

    bool start_ppg = (in->ctrl_start_mask & NC_STREAM_MASK_PPG) != 0;
    bool start_ibi = (in->ctrl_start_mask & NC_STREAM_MASK_IBI) != 0;
    bool start_acc = (in->ctrl_start_mask & NC_STREAM_MASK_ACCEL) != 0;
    bool stop_ppg  = (in->ctrl_stop_mask & NC_STREAM_MASK_PPG) != 0;
    bool stop_ibi  = (in->ctrl_stop_mask & NC_STREAM_MASK_IBI) != 0;
    bool stop_acc  = (in->ctrl_stop_mask & NC_STREAM_MASK_ACCEL) != 0;

    /* Per-source demand; a stop bit suppresses only its own source and
     * persists until the caller clears it (re-subscribe / re-start).
     * sub_event and the EVENT mask bits start nothing by design. */
    bool ppg_src = (in->sub_ppg || start_ppg) && !stop_ppg;
    bool ibi_src = (in->sub_ibi || in->sub_hrs || start_ibi) && !stop_ibi;
    bool ppg_demand = ppg_src || ibi_src;

    bool acc_explicit = (in->sub_accel || start_acc) && !stop_acc;

    /* wear_paused suppresses the AFE but not the gate/motion-wake
     * accel coupling (accel must keep running off-ear to re-wake) */
    out->ppg_on   = ppg_demand && !in->wear_paused;
    out->accel_on = acc_explicit || (in->gate_en && ppg_demand);
    out->dsp_on   = out->ppg_on;
}
