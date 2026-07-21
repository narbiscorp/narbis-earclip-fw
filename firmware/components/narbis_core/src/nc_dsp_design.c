/*
 * nc_dsp_design.c — see header. The pipeline is scipy's, transcribed:
 *
 *   butter(2, [lo, hi], 'bandpass', fs=fs, output='zpk')
 *     = analog prototype (N=2: poles exp(j*3pi/4), exp(-j*3pi/4), k=1)
 *       -> pre-warp corners: w = 2*fs*tan(pi*f/fs)
 *       -> lp2bp_zpk:  p_lp = p*BW/2;  p_bp = p_lp +- sqrt(p_lp^2 - w0^2)
 *          (BW = wh-wl, w0 = sqrt(wl*wh)), zeros: 2 at s=0, k *= BW^2
 *       -> bilinear_zpk (fs2 = 2*fs): x_z = (fs2+x)/(fs2-x), 2 extra
 *          zeros at z=-1, k *= Re( prod(fs2 - z_s) / prod(fs2 - p_s) )
 *   zpk2sos(..., pairing='nearest'): sec1 gets the pole pair nearest
 *   the unit circle + the zeros at +1; sec0 the other pair + zeros at
 *   -1 (verified against the committed tables' sign pattern).
 *   Overall gain split sqrt(k) into each section's numerator.
 */
#include "narbis/nc_dsp_design.h"
#include <math.h>
#include <complex.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define Q30F 1073741824.0   /* 2^30 */
#define POLE_RADIUS_MAX 0.9995

static int32_t q30(double x, bool *ok)
{
    double v = round(x * Q30F);
    if (v <= -2147483648.0 || v >= 2147483648.0) {
        *ok = false;
        return 0;
    }
    return (int32_t)v;
}

/* max |root| of z^2 + a1 z + a2 (real coefficients) */
static double pole_radius(double a1, double a2)
{
    double disc = a1 * a1 - 4.0 * a2;
    if (disc >= 0.0) {
        double r1 = fabs((-a1 + sqrt(disc)) * 0.5);
        double r2 = fabs((-a1 - sqrt(disc)) * 0.5);
        return (r1 > r2) ? r1 : r2;
    }
    return sqrt(a2);   /* conjugate pair: radius = sqrt(|a2|) */
}

bool nc_dsp_design_bp(uint16_t fs_sps, uint16_t lo_x100, uint16_t hi_x100,
                      nc_bq_coeff_t out[2])
{
    const double fs = (double)fs_sps;
    double lo = lo_x100 / 100.0;
    double hi = hi_x100 / 100.0;

    /* Clamp to sane digital territory; reject degenerate afterwards. */
    const double hi_max = 0.45 * fs;
    if (hi > hi_max) hi = hi_max;
    if (lo < 0.05) lo = 0.05;
    if (!(lo < hi * 0.95)) {
        return false;
    }

    /* Pre-warped analog corners. */
    const double wl = 2.0 * fs * tan(M_PI * lo / fs);
    const double wh = 2.0 * fs * tan(M_PI * hi / fs);
    const double bw = wh - wl;
    const double w0 = sqrt(wl * wh);

    /* Prototype poles (N=2 Butterworth), then lp2bp. */
    const double complex proto[2] = {
        cexp(I * (3.0 * M_PI / 4.0)),
        cexp(-I * (3.0 * M_PI / 4.0)),
    };
    double complex p_bp[4];
    for (int i = 0; i < 2; i++) {
        double complex plp = proto[i] * (bw / 2.0);
        double complex root = csqrt(plp * plp - w0 * w0);
        p_bp[i] = plp + root;
        p_bp[i + 2] = plp - root;
    }
    double k = bw * bw;   /* k_lp(=1) * BW^degree, degree 2 */

    /* Bilinear. Analog zeros: two at s=0. */
    const double fs2 = 2.0 * fs;
    double complex p_z[4];
    double complex num = fs2 * fs2;        /* prod(fs2 - 0) over 2 zeros */
    double complex den = 1.0;
    for (int i = 0; i < 4; i++) {
        p_z[i] = (fs2 + p_bp[i]) / (fs2 - p_bp[i]);
        den *= (fs2 - p_bp[i]);
    }
    k *= creal(num / den);
    /* digital zeros: {+1, +1} (from s=0) and {-1, -1} (degree fill) */

    /* Pair the pole conjugate pairs: p_z[0]/p_z[1] came from proto[0]/
     * proto[1] with '+' root, p_z[2]/p_z[3] with '-'. Regroup into
     * conjugate pairs by matching conj(). */
    double complex pairs[2][2];
    /* p_z[0] pairs with whichever of the others is (numerically) its
     * conjugate; the remaining two form the second pair. */
    int mate = 1;
    double best = 1e300;
    for (int i = 1; i < 4; i++) {
        double d = cabs(p_z[i] - conj(p_z[0]));
        if (d < best) { best = d; mate = i; }
    }
    pairs[0][0] = p_z[0];
    pairs[0][1] = p_z[mate];
    int others[2], no = 0;
    for (int i = 1; i < 4; i++) {
        if (i != mate) others[no++] = i;
    }
    pairs[1][0] = p_z[others[0]];
    pairs[1][1] = p_z[others[1]];

    /* 'nearest' pairing: the pair closest to the unit circle takes the
     * zeros at +1 and lands in sec1; the other pair + zeros{-1,-1} is
     * sec0 (matches the committed tables). */
    double r0 = cabs(pairs[0][0]);
    double r1 = cabs(pairs[1][0]);
    int near = (r0 >= r1) ? 0 : 1;
    int far = 1 - near;

    const double g = sqrt(fabs(k));   /* sqrt(k) per section; k>0 for BP */

    bool ok = true;
    /* sec0: zeros {-1,-1} -> b = g*(1, 2, 1); poles = far pair */
    {
        double a1 = -2.0 * creal(pairs[far][0]);
        double a2 = creal(pairs[far][0] * pairs[far][1]);
        if (pole_radius(a1, a2) >= POLE_RADIUS_MAX) return false;
        out[0].b0 = q30(g, &ok);
        out[0].b1 = q30(2.0 * g, &ok);
        out[0].b2 = q30(g, &ok);
        out[0].a1 = q30(a1, &ok);
        out[0].a2 = q30(a2, &ok);
    }
    /* sec1: zeros {+1,+1} -> b = g*(1, -2, 1); poles = near pair */
    {
        double a1 = -2.0 * creal(pairs[near][0]);
        double a2 = creal(pairs[near][0] * pairs[near][1]);
        if (pole_radius(a1, a2) >= POLE_RADIUS_MAX) return false;
        out[1].b0 = q30(g, &ok);
        out[1].b1 = q30(-2.0 * g, &ok);
        out[1].b2 = q30(g, &ok);
        out[1].a1 = q30(a1, &ok);
        out[1].a2 = q30(a2, &ok);
    }
    return ok;
}

int32_t nc_dsp_design_alpha_q31(uint16_t fs_sps, uint16_t fc_x100)
{
    double fc = fc_x100 / 100.0;
    double a = 2147483648.0 * (1.0 - exp(-2.0 * M_PI * fc / (double)fs_sps));
    if (a < 1.0) a = 1.0;
    if (a > 2147483647.0) a = 2147483647.0;
    return (int32_t)llround(a);
}
