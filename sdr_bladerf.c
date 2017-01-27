// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// bladerf.c: bladeRF support code
//

#include "dump1090.h"
#include "sdr_bladerf.h"

#include <libbladeRF.h>

static struct {
    const char *device_str;
    const char *fpga_path;
    unsigned decimation;
    bladerf_lpf_mode lpf_mode;
    unsigned lpf_bandwidth;

    struct bladerf *device;
} BladeRF;

void bladeRFInitConfig()
{
    BladeRF.device_str = NULL;
    BladeRF.fpga_path = NULL;
    BladeRF.decimation = 1;
    BladeRF.lpf_mode = BLADERF_LPF_NORMAL;
    BladeRF.lpf_bandwidth = 1750000;
    BladeRF.device = NULL;
}

bool bladeRFHandleOption(int argc, char **argv, int *jptr)
{
    int j = *jptr;
    bool more = (j+1 < argc);
    if (!strcmp(argv[j], "--bladerf-fpga") && more) {
        BladeRF.fpga_path = strdup(argv[++j]);
    } else if (!strcmp(argv[j], "--bladerf-decimation") && more) {
        BladeRF.decimation = atoi(argv[++j]);
    } else if (!strcmp(argv[j], "--bladerf-bandwidth") && more) {
        ++j;
        if (!strcasecmp(argv[j], "bypass")) {
            BladeRF.lpf_mode = BLADERF_LPF_BYPASSED;
        } else {
            BladeRF.lpf_mode = BLADERF_LPF_NORMAL;
            BladeRF.lpf_bandwidth = atoi(argv[j]);
        }
    } else {
        return false;
    }

    *jptr = j;
    return true;
}

void bladeRFShowHelp()
{
    printf("      bladeRF-specific options (use with --device-type bladerf)\n");
    printf("\n");
    printf("--device <ident>         select device by bladeRF 'device identifier'\n");
    printf("--bladerf-fpga <path>    use alternative FPGA bitstream ('' to disable FPGA load)\n");
    printf("--bladerf-decimation <N> assume FPGA decimates by a factor of N\n");
    printf("--bladerf-bandwidth <hz> set LPF bandwidth ('bypass' to bypass the LPF)\n");
    printf("\n");
}

static int lna_gain_db(bladerf_lna_gain gain)
{
    switch (gain) {
    case BLADERF_LNA_GAIN_BYPASS:
        return 0;
    case BLADERF_LNA_GAIN_MID:
        return BLADERF_LNA_GAIN_MID_DB;
    case BLADERF_LNA_GAIN_MAX:
        return BLADERF_LNA_GAIN_MAX_DB;
    default:
        return -1;
    }
}

static void show_config()
{
    int status;

    unsigned rate;
    unsigned freq;
    bladerf_lpf_mode lpf_mode;
    unsigned lpf_bw;
    bladerf_lna_gain lna_gain;
    int rxvga1_gain;
    int rxvga2_gain;
    int16_t lms_dc_i, lms_dc_q;
    int16_t fpga_phase, fpga_gain;
    struct bladerf_lms_dc_cals dc_cals;

    if ((status = bladerf_get_sample_rate(BladeRF.device, BLADERF_MODULE_RX, &rate)) < 0 ||
        (status = bladerf_get_frequency(BladeRF.device, BLADERF_MODULE_RX, &freq)) < 0 ||
        (status = bladerf_get_lpf_mode(BladeRF.device, BLADERF_MODULE_RX, &lpf_mode)) < 0 ||
        (status = bladerf_get_bandwidth(BladeRF.device, BLADERF_MODULE_RX, &lpf_bw)) < 0 ||
        (status = bladerf_get_lna_gain(BladeRF.device, &lna_gain)) < 0 ||
        (status = bladerf_get_rxvga1(BladeRF.device, &rxvga1_gain)) < 0 ||
        (status = bladerf_get_rxvga2(BladeRF.device, &rxvga2_gain)) < 0 ||
        (status = bladerf_get_correction(BladeRF.device, BLADERF_MODULE_RX, BLADERF_CORR_LMS_DCOFF_I, &lms_dc_i)) < 0 ||
        (status = bladerf_get_correction(BladeRF.device, BLADERF_MODULE_RX, BLADERF_CORR_LMS_DCOFF_Q, &lms_dc_q)) < 0 ||
        (status = bladerf_get_correction(BladeRF.device, BLADERF_MODULE_RX, BLADERF_CORR_FPGA_PHASE, &fpga_phase)) < 0 ||
        (status = bladerf_get_correction(BladeRF.device, BLADERF_MODULE_RX, BLADERF_CORR_FPGA_GAIN, &fpga_gain)) < 0 ||
        (status = bladerf_lms_get_dc_cals(BladeRF.device, &dc_cals)) < 0) {
        fprintf(stderr, "bladeRF: couldn't read back device configuration\n");
        return;
    }

    fprintf(stderr, "bladeRF: sampling rate: %.1f MHz\n", rate/1e6);
    fprintf(stderr, "bladeRF: frequency:     %.1f MHz\n", freq/1e6);
    fprintf(stderr, "bladeRF: LNA gain:      %ddB\n", lna_gain_db(lna_gain));
    fprintf(stderr, "bladeRF: RXVGA1 gain:   %ddB\n", rxvga1_gain);
    fprintf(stderr, "bladeRF: RXVGA2 gain:   %ddB\n", rxvga2_gain);

    switch (lpf_mode) {
    case BLADERF_LPF_NORMAL:
        fprintf(stderr, "bladeRF: LPF bandwidth: %.2f MHz\n", lpf_bw/1e6);
        break;
    case BLADERF_LPF_BYPASSED:
        fprintf(stderr, "bladeRF: LPF bypassed\n");
        break;
    case BLADERF_LPF_DISABLED:
        fprintf(stderr, "bladeRF: LPF disabled\n");
        break;
    default:
        fprintf(stderr, "bladeRF: LPF in unknown state\n");
        break;
    }

    fprintf(stderr, "bladeRF: calibration settings:\n");
    fprintf(stderr, "  LMS DC adjust:     I=%d Q=%d\n", lms_dc_i, lms_dc_q);
    fprintf(stderr, "  FPGA phase adjust: %+.3f degrees\n", fpga_phase * 10.0 / 4096);
    fprintf(stderr, "  FPGA gain adjust:  %+.3f\n", fpga_gain * 1.0 / 4096);
    fprintf(stderr, "  LMS LPF tuning:    %d\n", dc_cals.lpf_tuning);
    fprintf(stderr, "  LMS RX LPF filter: I=%d Q=%d\n", dc_cals.rx_lpf_i, dc_cals.rx_lpf_q);
    fprintf(stderr, "  LMS RXVGA2 DC ref: %d\n", dc_cals.dc_ref);
    fprintf(stderr, "  LMS RXVGA2A:       I=%d Q=%d\n", dc_cals.rxvga2a_i, dc_cals.rxvga2a_q);
    fprintf(stderr, "  LMS RXVGA2B:       I=%d Q=%d\n", dc_cals.rxvga2b_i, dc_cals.rxvga2b_q);

}

bool bladeRFOpen()
{
    if (BladeRF.device) {
        return true;
    }

    int status;

    bladerf_set_usb_reset_on_open(true);
    if ((status = bladerf_open(&BladeRF.device, Modes.dev_name)) < 0) {
        fprintf(stderr, "Failed to open bladeRF: %s\n", bladerf_strerror(status));
        goto error;
    }

    const char *fpga_path;
    if (BladeRF.fpga_path) {
        fpga_path = BladeRF.fpga_path;
    } else {
        bladerf_fpga_size size;
        if ((status = bladerf_get_fpga_size(BladeRF.device, &size)) < 0) {
            fprintf(stderr, "bladerf_get_fpga_size failed: %s\n", bladerf_strerror(status));
            goto error;
        }

        switch (size) {
        case BLADERF_FPGA_40KLE:
            fpga_path = "/usr/share/Nuand/bladeRF/hostedx40.rbf";
            break;
        case BLADERF_FPGA_115KLE:
            fpga_path = "/usr/share/Nuand/bladeRF/hostedx115.rbf";
            break;
        default:
            fprintf(stderr, "bladerf: unknown FPGA size, skipping FPGA load");
            fpga_path = NULL;
            break;
        }
    }

    if (fpga_path && fpga_path[0]) {
        fprintf(stderr, "bladerf: loading FPGA bitstream from %s\n", fpga_path);
        if ((status = bladerf_load_fpga(BladeRF.device, fpga_path)) < 0) {
            fprintf(stderr, "bladerf_load_fpga() failed: %s\n", bladerf_strerror(status));
        }
    }

    if ((status = bladerf_set_sample_rate(BladeRF.device, BLADERF_MODULE_RX, Modes.sample_rate * BladeRF.decimation, NULL)) < 0) {
        fprintf(stderr, "bladerf_set_sample_rate failed: %s\n", bladerf_strerror(status));
        goto error;
    }

    if ((status = bladerf_set_frequency(BladeRF.device, BLADERF_MODULE_RX, Modes.freq)) < 0) {
        fprintf(stderr, "bladerf_set_frequency failed: %s\n", bladerf_strerror(status));
        goto error;
    }

    if ((status = bladerf_set_lpf_mode(BladeRF.device, BLADERF_MODULE_RX, BladeRF.lpf_mode)) < 0) {
        fprintf(stderr, "bladerf_set_lpf_mode failed: %s\n", bladerf_strerror(status));
        goto error;
    }

    if ((status = bladerf_set_bandwidth(BladeRF.device, BLADERF_MODULE_RX, BladeRF.lpf_bandwidth, NULL)) < 0) {
        fprintf(stderr, "bladerf_set_lpf_bandwidth failed: %s\n", bladerf_strerror(status));
        goto error;
    }

    /* turn the tx gain right off, just in case */
    if ((status = bladerf_set_gain(BladeRF.device, BLADERF_MODULE_TX, -100)) < 0) {
        fprintf(stderr, "bladerf_set_gain(TX) failed: %s\n", bladerf_strerror(status));
        goto error;
    }

    if ((status = bladerf_set_gain(BladeRF.device, BLADERF_MODULE_RX, Modes.gain / 10.0)) < 0) {
        fprintf(stderr, "bladerf_set_gain(RX) failed: %s\n", bladerf_strerror(status));
        goto error;
    }

    if ((status = bladerf_set_loopback(BladeRF.device, BLADERF_LB_NONE)) < 0) {
        fprintf(stderr, "bladerf_set_loopback() failed: %s\n", bladerf_strerror(status));
        goto error;
    }

    if ((status = bladerf_calibrate_dc(BladeRF.device, BLADERF_DC_CAL_LPF_TUNING)) < 0) {
        fprintf(stderr, "bladerf_calibrate_dc(LPF_TUNING) failed: %s\n", bladerf_strerror(status));
        goto error;
    }

    if ((status = bladerf_calibrate_dc(BladeRF.device, BLADERF_DC_CAL_RX_LPF)) < 0) {
        fprintf(stderr, "bladerf_calibrate_dc(RX_LPF) failed: %s\n", bladerf_strerror(status));
        goto error;
    }

    if ((status = bladerf_calibrate_dc(BladeRF.device, BLADERF_DC_CAL_RXVGA2)) < 0) {
        fprintf(stderr, "bladerf_calibrate_dc(RXVGA2) failed: %s\n", bladerf_strerror(status));
        goto error;
    }

    show_config();
    return true;

 error:
    if (BladeRF.device) {
        bladerf_close(BladeRF.device);
        BladeRF.device = NULL;
    }
    return false;
}

void bladeRFRun()
{
    if (!BladeRF.device) {
        return;
    }

    int buflen = MODES_MAG_BUF_SAMPLES;

    unsigned ms_per_transfer = 1000 * buflen * 4 / Modes.sample_rate;
    unsigned transfers = 4;

    int status;
    if ((status = bladerf_sync_config(BladeRF.device, BLADERF_MODULE_RX,
                                      BLADERF_FORMAT_SC16_Q11,
                                      /* num_buffers */ transfers * 2,
                                      /* buffer_size */ buflen * 4,
                                      /* num_transfers */ transfers,
                                      /* stream_timeout, ms */ ms_per_transfer * (transfers + 2))) < 0) {
        fprintf(stderr, "bladerf_sync_config() failed: %s\n", bladerf_strerror(status));
        return;
    }

    if ((status = bladerf_enable_module(BladeRF.device, BLADERF_MODULE_RX, true) < 0)) {
        fprintf(stderr, "bladerf_enable_module(RX, true) failed: %s\n", bladerf_strerror(status));
        return;
    }

    void *buf = malloc(buflen * 4);
    if (!buf) {
        fprintf(stderr, "failed to allocate bladeRF sample buffer\n");
        return;
    }

    struct converter_state *converter_state;
    iq_convert_fn converter = init_converter(INPUT_SC16Q11,
                                             Modes.sample_rate,
                                             Modes.dc_filter,
                                             &converter_state);
    if (!converter) {
        fprintf(stderr, "can't initialize sample converter\n");
        free(buf);
        return;
    }

    struct bladerf_metadata metadata;
    bool dropping = false;

    struct timespec thread_cpu;
    start_cpu_timing(&thread_cpu);

    uint64_t sampleCounter = 0;
    pthread_mutex_lock(&Modes.data_mutex);
    while (!Modes.exit) {
        pthread_mutex_unlock(&Modes.data_mutex);

        metadata.timestamp = 0;
        metadata.flags = BLADERF_META_FLAG_RX_NOW;
        status = bladerf_sync_rx(BladeRF.device,
                                 buf, buflen,
                                 &metadata,
                                 1000);

        pthread_mutex_lock(&Modes.data_mutex);

        if (status < 0) {
            fprintf(stderr, "bladerf_sync_rx() failed: %s\n", bladerf_strerror(status));
            if (status == BLADERF_ERR_TIMEOUT) {
                continue;
            }

            break;
        }

        if (metadata.status & BLADERF_META_STATUS_OVERRUN) {
            fprintf(stderr, "bladerf_sync_rx(): overrun detected\n");
        }

        unsigned next_free_buffer = (Modes.first_free_buffer + 1) % MODES_MAG_BUFFERS;
        struct mag_buf *outbuf = &Modes.mag_buffers[Modes.first_free_buffer];
        struct mag_buf *lastbuf = &Modes.mag_buffers[(Modes.first_free_buffer + MODES_MAG_BUFFERS - 1) % MODES_MAG_BUFFERS];
        unsigned free_bufs = (Modes.first_filled_buffer - next_free_buffer + MODES_MAG_BUFFERS) % MODES_MAG_BUFFERS;

        if (free_bufs == 0 || (dropping && free_bufs < MODES_MAG_BUFFERS/2)) {
            // FIFO is full. Drop this block.
            sampleCounter += metadata.actual_count;
            outbuf->dropped += metadata.actual_count;
            dropping = true;
            continue;
        }

        // Compute the sample timestamp and system timestamp for the start of the block
        outbuf->sampleTimestamp = sampleCounter * 12e6 / Modes.sample_rate;
        unsigned block_duration = 1e9 * metadata.actual_count / Modes.sample_rate;
        sampleCounter += metadata.actual_count;

        // Get the approx system time for the start of this block
        clock_gettime(CLOCK_REALTIME, &outbuf->sysTimestamp);
        outbuf->sysTimestamp.tv_nsec -= block_duration;
        normalize_timespec(&outbuf->sysTimestamp);

        dropping = false;
        pthread_mutex_unlock(&Modes.data_mutex);

        // Copy trailing data from last block (or reset if not valid)
        if (outbuf->dropped == 0) {
            memcpy(outbuf->data, lastbuf->data + lastbuf->length, Modes.trailing_samples * sizeof(uint16_t));
        } else {
            memset(outbuf->data, 0, Modes.trailing_samples * sizeof(uint16_t));
        }

        // Convert the new data
        outbuf->length = metadata.actual_count;
        converter(buf, &outbuf->data[Modes.trailing_samples], metadata.actual_count, converter_state, &outbuf->mean_level, &outbuf->mean_power);

        // Push the new data to the demodulation thread
        pthread_mutex_lock(&Modes.data_mutex);

        // accumulate CPU while holding the mutex, and restart measurement
        end_cpu_timing(&thread_cpu, &Modes.reader_cpu_accumulator);
        start_cpu_timing(&thread_cpu);

        Modes.mag_buffers[next_free_buffer].dropped = 0;
        Modes.mag_buffers[next_free_buffer].length = 0;  // just in case
        Modes.first_free_buffer = next_free_buffer;

        pthread_cond_signal(&Modes.data_cond);
    }

    pthread_mutex_unlock(&Modes.data_mutex);
    cleanup_converter(converter_state);
    free(buf);

    if ((status = bladerf_enable_module(BladeRF.device, BLADERF_MODULE_RX, false) < 0)) {
        fprintf(stderr, "bladerf_enable_module(RX, false) failed: %s\n", bladerf_strerror(status));
    }
}

void bladeRFClose()
{
    if (!BladeRF.device) {
        return;
    }

    bladerf_close(BladeRF.device);
    BladeRF.device = NULL;
}
