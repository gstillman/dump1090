#include "dump1090.h"

static void **testdata_uc8;
static void **testdata_sc16;
static void **testdata_sc16q11;
static uint16_t *outdata;

// SC16Q11_TABLE_BITS notes:

// 11 bits (8MB) gives you full precision, but a large table that doesn't fit in cache
// 9 bits (512kB) will fit in the Pi 2/3's shared L2 cache
//   (but there will be contention from other cores)
// 8 bits (128kB) will fit in the Pi 1's L2 cache
// 7 bits (32kB) will fit in the Pi 1/2/3's L1 cache

// Sample results for "SC16Q11, no DC":

// Core i7, SC16Q11_TABLE_BITS undefined (floating point path):
//  764.15M samples in 5.001021 seconds
//  152.80M samples/second
// Core i7, SC16Q11_TABLE_BITS=11
//  507.25M samples in 5.011417 seconds
//  101.22M samples/second
// Core i7, SC16Q11_TABLE_BITS=9
//  1216.35M samples in 5.004822 seconds
//  243.04M samples/second
// Core i7, SC16Q11_TABLE_BITS=8
//  1584.66M samples in 5.001499 seconds
//  316.84M samples/second
// Core i7, SC16Q11_TABLE_BITS=7
//  1879.57M samples in 5.002829 seconds
//  375.70M samples/second

// Pi3B, SC16Q11_TABLE_BITS undefined (floating point path):
//  111.41M samples in 5.020529 seconds
//  22.19M samples/second
// Pi3B, SC16Q11_TABLE_BITS=11
//  30.15M samples in 5.147597 seconds
//  5.86M samples/second
// Pi3B, SC16Q11_TABLE_BITS=9
//  96.99M samples in 5.018548 seconds
//  19.33M samples/second
// Pi3B, SC16Q11_TABLE_BITS=8
//  167.77M samples in 5.008305 seconds
//  33.50M samples/second
// Pi3B, SC16Q11_TABLE_BITS=7
//  300.15M samples in 5.021375 seconds
//  59.78M samples/second



void prepare()
{
    srand(1);

    testdata_uc8 = calloc(10, sizeof(void*));
    testdata_sc16 = calloc(10, sizeof(void*));
    testdata_sc16q11 = calloc(10, sizeof(void*));
    outdata = calloc(MODES_MAG_BUF_SAMPLES, sizeof(uint16_t));

    for (int buf = 0; buf < 10; ++buf) {
        uint8_t *uc8 = calloc(MODES_MAG_BUF_SAMPLES, 2);
        testdata_uc8[buf] = uc8;;
        uint16_t *sc16 = calloc(MODES_MAG_BUF_SAMPLES, 4);
        testdata_sc16[buf] = sc16;
        uint16_t *sc16q11 = calloc(MODES_MAG_BUF_SAMPLES, 4);
        testdata_sc16q11[buf] = sc16q11;

        for (unsigned i = 0; i < MODES_MAG_BUF_SAMPLES; ++i) {
            double I = 2.0 * rand() / (RAND_MAX + 1.0) - 1.0;
            double Q = 2.0 * rand() / (RAND_MAX + 1.0) - 1.0;

            uc8[i*2] = (uint8_t) (I * 128 + 128);
            uc8[i*2+1] = (uint8_t) (Q * 128 + 128);

            sc16[i*2] = htole16( (int16_t) (I * 32768.0) );
            sc16[i*2+1] = htole16( (int16_t) (Q * 32768.0) );

            sc16q11[i*2] = htole16( (int16_t) (I * 2048.0) );
            sc16q11[i*2+1] = htole16( (int16_t) (Q * 2048.0) );
        }
    }
}

void test(const char *what, input_format_t format, void **data, double sample_rate, bool filter_dc) {
    fprintf(stderr, "Benchmarking: %s ", what);

    struct converter_state *state;
    iq_convert_fn converter = init_converter(format, sample_rate, filter_dc, &state);
    if (!converter) {
        fprintf(stderr, "Can't initialize converter\n");
        return;
    }

    struct timespec total = { 0, 0 };
    int iterations = 0;

    // Run it once to force init.
    converter(data[0], outdata, MODES_MAG_BUF_SAMPLES, state, NULL, NULL);

    while (total.tv_sec < 5) {
        fprintf(stderr, ".");

        struct timespec start;
        start_cpu_timing(&start);

        for (int i = 0; i < 10; ++i) {
            converter(data[i], outdata, MODES_MAG_BUF_SAMPLES, state, NULL, NULL);
        }

        end_cpu_timing(&start, &total);
        iterations++;
    }

    fprintf(stderr, "\n");
    cleanup_converter(state);

    double samples = 10.0 * iterations * MODES_MAG_BUF_SAMPLES;
    double nanos = total.tv_sec * 1e9 + total.tv_nsec;
    fprintf(stderr, "  %.2fM samples in %.6f seconds\n",
            samples / 1e6, nanos / 1e9);
    fprintf(stderr, "  %.2fM samples/second\n",
            samples / nanos * 1e3);
}

int main(int argc, char **argv)
{
    MODES_NOTUSED(argc);
    MODES_NOTUSED(argv);

    prepare();

    test("SC16Q11, DC", INPUT_SC16Q11, testdata_sc16q11, 2400000, true);
    test("SC16Q11, no DC", INPUT_SC16Q11, testdata_sc16q11, 2400000, false);

    test("UC8, DC", INPUT_UC8, testdata_uc8, 2400000, true);
    test("UC8, no DC", INPUT_UC8, testdata_uc8, 2400000, false);

    test("SC16, DC", INPUT_SC16, testdata_sc16, 2400000, true);
    test("SC16, no DC", INPUT_SC16, testdata_sc16, 2400000, false);
}
