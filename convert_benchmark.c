#include "dump1090.h"

static void **testdata_uc8;
static void **testdata_sc16;
static void **testdata_sc16q11;
static uint16_t *outdata;

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
