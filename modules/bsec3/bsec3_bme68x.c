/*
 * bsec3_bme68x.c
 *
 * MicroPython C module: BME68X sensor driver integrated with BSEC3.
 *
 * Provides a single Python type  bme68x.BME68X(i2c, addr)  with one
 * measurement method:
 *
 *   sleep_us, outputs = sensor.run_cycle(bsec_inst_id)
 *
 * run_cycle() performs the full BSEC measurement step in C:
 *   bsec_sensor_control → configure BME68x → forced-mode measurement →
 *   bme68x_get_data → bsec_do_steps → return (sleep_us, outputs_or_None)
 *
 * I2C is accessed by calling machine.I2C methods on the Python object
 * supplied to the constructor, so this module is portable to any
 * MicroPython build that provides machine.I2C.
 *
 * State save/load stays in MicroPython via the existing bsec3 module
 * (bsec_wrapper.c).  Subscription and initialisation also remain there.
 *
 * BSD-3-Clause — see Bosch header files for licence text.
 */

#include <string.h>

#include "py/obj.h"
#include "py/objarray.h"   /* mp_obj_new_bytearray_by_ref */
#include "py/runtime.h"
#include "py/mphal.h"
#include "py/nlr.h"

#include "bme68x.h"
#include "bsec_datatypes.h"
#include "bsec_interface.h"

/* Pico SDK 64-bit timer — needed for a stable ns-resolution timestamp */
#include "hardware/timer.h"

/* Clamp sleep_us (int64_t) to mp_int_t range for mp_obj_new_int().
   300s ULP mode fits in 32-bit, but clamp defensively. */
static inline mp_int_t clamp_sleep_us(int64_t us) {
    if (us < 0) return 0;
    if (us > 0x7FFFFFFF) return 0x7FFFFFFF;
    return (mp_int_t)us;
}

#include "bsec3_common.h"

static void *resolve_bsec_instance(mp_obj_t inst_id_obj) {
    int id = mp_obj_get_int(inst_id_obj);
    if (id < 0 || id >= BSEC3_MAX_INSTANCES) {
        mp_raise_ValueError(MP_ERROR_TEXT("BSEC instance id out of range"));
    }
    void *inst = bsec3_instances[id];
    if (inst == NULL) {
        mp_raise_ValueError(MP_ERROR_TEXT("BSEC instance not created"));
    }
    return inst;
}

/* -------------------------------------------------------------------------
 * BME68X Python object
 * ------------------------------------------------------------------------- */
typedef struct {
    mp_obj_base_t    base;
    struct bme68x_dev dev;      /* Bosch device struct (owns the callbacks) */
    mp_obj_t         i2c_obj;   /* machine.I2C passed from Python           */
    uint8_t          i2c_addr;
} bme68x_obj_t;

/* -------------------------------------------------------------------------
 * I2C read callback
 *
 * Calls  i2c.readfrom_mem_into(addr, reg_addr, buf)  on the Python I2C
 * object.  Uses nlr_push/nlr_pop so that any MicroPython exception is
 * caught and converted to a BME68X_E_COM_FAIL return code rather than
 * unwinding through Bosch C library stack frames.
 *
 * mp_obj_new_bytearray_by_ref() creates a bytearray that references
 * 'data' directly — the I2C driver writes the result into data[].
 * ------------------------------------------------------------------------- */
static BME68X_INTF_RET_TYPE bme68x_i2c_read_cb(
    uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    bme68x_obj_t *self = (bme68x_obj_t *)intf_ptr;
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t buf = mp_obj_new_bytearray_by_ref(len, data);
        mp_obj_t dest[5];
        mp_load_method(self->i2c_obj, MP_QSTR_readfrom_mem_into, dest);
        dest[2] = mp_obj_new_int(self->i2c_addr);
        dest[3] = mp_obj_new_int(reg_addr);
        dest[4] = buf;
        mp_call_method_n_kw(3, 0, dest);
        nlr_pop();
        return BME68X_INTF_RET_SUCCESS;
    } else {
        return BME68X_E_COM_FAIL;
    }
}

/* -------------------------------------------------------------------------
 * I2C write callback
 *
 * Calls  i2c.writeto_mem(addr, reg_addr, bytes)  on the Python I2C object.
 * mp_obj_new_bytes() makes an immutable copy matching the const qualifier.
 * ------------------------------------------------------------------------- */
static BME68X_INTF_RET_TYPE bme68x_i2c_write_cb(
    uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    bme68x_obj_t *self = (bme68x_obj_t *)intf_ptr;
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t dest[5];
        mp_load_method(self->i2c_obj, MP_QSTR_writeto_mem, dest);
        dest[2] = mp_obj_new_int(self->i2c_addr);
        dest[3] = mp_obj_new_int(reg_addr);
        dest[4] = mp_obj_new_bytes(data, len);
        mp_call_method_n_kw(3, 0, dest);
        nlr_pop();
        return BME68X_INTF_RET_SUCCESS;
    } else {
        return BME68X_E_COM_FAIL;
    }
}

/* -------------------------------------------------------------------------
 * Delay callback — delegates to MicroPython HAL (portable across builds)
 * ------------------------------------------------------------------------- */
static void bme68x_delay_us_cb(uint32_t period, void *intf_ptr) {
    (void)intf_ptr;
    mp_hal_delay_us(period);
}

/* -------------------------------------------------------------------------
 * Constructor: BME68X(i2c, addr)
 *
 * Stores the machine.I2C object, populates the Bosch dev struct with
 * callbacks, and calls bme68x_init() (reads chip-id, loads calibration).
 * Raises RuntimeError on any Bosch API failure.
 * ------------------------------------------------------------------------- */
static mp_obj_t bme68x_make_new(const mp_obj_type_t *type,
    size_t n_args, size_t n_kw, const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 2, 2, false);

    bme68x_obj_t *self = mp_obj_malloc(bme68x_obj_t, type);
    self->i2c_obj  = args[0];
    self->i2c_addr = (uint8_t)mp_obj_get_int(args[1]);

    /* Populate Bosch dev struct */
    memset(&self->dev, 0, sizeof(self->dev));
    self->dev.intf     = BME68X_I2C_INTF;
    self->dev.intf_ptr = self;              /* callbacks cast intf_ptr → self */
    self->dev.read     = bme68x_i2c_read_cb;
    self->dev.write    = bme68x_i2c_write_cb;
    self->dev.delay_us = bme68x_delay_us_cb;
    self->dev.amb_temp = 25;               /* standard ambient; BSEC provides
                                              heat-compensated temperature     */

    int8_t brc = bme68x_init(&self->dev);
    if (brc != BME68X_OK) {
        mp_raise_msg_varg(&mp_type_RuntimeError,
            MP_ERROR_TEXT("bme68x_init failed: %d"), (int)brc);
    }

    return MP_OBJ_FROM_PTR(self);
}

/* -------------------------------------------------------------------------
 * run_cycle(inst_id) -> (sleep_us, outputs) | (sleep_us, None)
 *
 * Executes one complete BSEC measurement step:
 *
 *  1. Capture current timestamp (ns) via Pico SDK time_us_64()
 *  2. bsec_sensor_control() — decide whether to measure and get settings
 *  3. If trigger_measurement == 0: return (sleep_us, None) immediately
 *  4. bme68x_set_conf()  — set oversampling for this cycle
 *  5. bme68x_set_heatr_conf() — set heater temp/duration
 *  6. bme68x_set_op_mode(FORCED) — start measurement
 *  7. Wait bme68x_get_meas_dur() + heater_dur microseconds
 *  8. bme68x_get_data() — read raw T/P/H/gas
 *  9. Build bsec_input_t array (respecting process_data bitmask)
 * 10. bsec_do_steps() — BSEC output computation
 * 11. Compute sleep_us from next_call (after all processing)
 * 12. Return (sleep_us, [(sensor_id, signal, accuracy), ...])
 *
 * Returns (sleep_us, None) when:
 *   - BSEC says no measurement needed this cycle
 *   - Sensor produced 0 data fields (transient; BSEC recovers timing)
 *   - BSEC produced 0 outputs (can happen during warm-up)
 *
 * Raises RuntimeError if any Bosch API or BSEC call returns an error.
 * Raises RuntimeError if the I2C bus fails (propagated from callbacks).
 * ------------------------------------------------------------------------- */
static mp_obj_t bme68x_run_cycle(mp_obj_t self_in, mp_obj_t inst_id_obj) {
    bme68x_obj_t *self = MP_OBJ_TO_PTR(self_in);
    void *inst = resolve_bsec_instance(inst_id_obj);

    /* --- 1. Nanosecond timestamp ---------------------------------------- */
    int64_t ts_ns = (int64_t)time_us_64() * 1000LL;

    /* --- 2. BSEC sensor control ----------------------------------------- */
    bsec_bme_settings_t settings;
    bsec_library_return_t rc = bsec_sensor_control(inst, ts_ns, &settings);
    if (rc < 0) {
        mp_raise_msg_varg(&mp_type_RuntimeError,
            MP_ERROR_TEXT("bsec_sensor_control failed: %d"), (int)rc);
    }

    /* --- 3. No measurement needed this cycle ----------------------------- */
    if (!settings.trigger_measurement) {
        int64_t sleep_us = (settings.next_call / 1000LL) - (int64_t)time_us_64();
        if (sleep_us < 0) sleep_us = 0;
        mp_obj_t ret[2] = { mp_obj_new_int(clamp_sleep_us(sleep_us)), mp_const_none };
        return mp_obj_new_tuple(2, ret);
    }

    /* --- 4. Oversampling configuration ------------------------------------ */
    /* bme68x_set_conf() does a register read-modify-write internally;
       we supply filter=OFF and odr=NONE which are correct for forced mode.   */
    struct bme68x_conf conf = {
        .os_hum  = settings.humidity_oversampling,
        .os_temp = settings.temperature_oversampling,
        .os_pres = settings.pressure_oversampling,
        .filter  = BME68X_FILTER_OFF,
        .odr     = BME68X_ODR_NONE,
    };
    int8_t brc = bme68x_set_conf(&conf, &self->dev);
    if (brc != BME68X_OK) {
        mp_raise_msg_varg(&mp_type_RuntimeError,
            MP_ERROR_TEXT("bme68x_set_conf failed: %d"), (int)brc);
    }

    /* --- 5. Heater configuration ----------------------------------------- */
    /* Forced mode uses single heater step; profile pointers left NULL
       (zero-initialised) which is correct — bme68x_set_heatr_conf only
       dereferences them in sequential/parallel mode. */
    struct bme68x_heatr_conf hcfg = {
        .enable    = settings.run_gas ? BME68X_ENABLE : BME68X_DISABLE,
        .heatr_temp = settings.heater_temperature,   /* °C */
        .heatr_dur  = settings.heater_duration,      /* ms */
    };
    brc = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &hcfg, &self->dev);
    if (brc != BME68X_OK) {
        mp_raise_msg_varg(&mp_type_RuntimeError,
            MP_ERROR_TEXT("bme68x_set_heatr_conf failed: %d"), (int)brc);
    }

    /* --- 6. Trigger forced-mode measurement ------------------------------ */
    brc = bme68x_set_op_mode(BME68X_FORCED_MODE, &self->dev);
    if (brc != BME68X_OK) {
        mp_raise_msg_varg(&mp_type_RuntimeError,
            MP_ERROR_TEXT("bme68x_set_op_mode failed: %d"), (int)brc);
    }

    /* --- 7. Wait for measurement + heater -------------------------------- */
    /* bme68x_get_meas_dur() returns µs; heatr_dur is in ms → × 1000       */
    uint32_t del_us = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &self->dev)
                      + (uint32_t)hcfg.heatr_dur * 1000U;
    mp_hal_delay_us(del_us);

    /* --- 8. Read sensor data --------------------------------------------- */
    struct bme68x_data data[3];  /* forced mode yields at most 1 field       */
    uint8_t n_fields = 0;
    brc = bme68x_get_data(BME68X_FORCED_MODE, data, &n_fields, &self->dev);
    if (brc < BME68X_OK) {
        mp_raise_msg_varg(&mp_type_RuntimeError,
            MP_ERROR_TEXT("bme68x_get_data failed: %d"), (int)brc);
    }
    if (n_fields == 0) {
        /* No new data — BSEC timing will recover on the next call */
        int64_t sleep_us = (settings.next_call / 1000LL) - (int64_t)time_us_64();
        if (sleep_us < 0) sleep_us = 0;
        mp_obj_t ret[2] = { mp_obj_new_int(clamp_sleep_us(sleep_us)), mp_const_none };
        return mp_obj_new_tuple(2, ret);
    }

    /* Update timestamp to actual measurement time for BSEC inputs */
    ts_ns = (int64_t)time_us_64() * 1000LL;

    /* --- 9. Build BSEC inputs -------------------------------------------- */
    /* Use the first field only (forced mode always delivers field 0).
       BSEC_INPUT_HEATSOURCE is omitted — home environment, no external heat.
       BSEC_INPUT_PROFILE_PART signal = 0 for forced mode (single heater
       step, always at profile index 0).                                       */
    bsec_input_t inputs[5];
    uint8_t n_inputs = 0;

    if (settings.process_data & BSEC_PROCESS_TEMPERATURE) {
        inputs[n_inputs].sensor_id  = BSEC_INPUT_TEMPERATURE;
        inputs[n_inputs].signal     = data[0].temperature;
        inputs[n_inputs].time_stamp = ts_ns;
        n_inputs++;
    }
    if (settings.process_data & BSEC_PROCESS_PRESSURE) {
        inputs[n_inputs].sensor_id  = BSEC_INPUT_PRESSURE;
        inputs[n_inputs].signal     = data[0].pressure;
        inputs[n_inputs].time_stamp = ts_ns;
        n_inputs++;
    }
    if (settings.process_data & BSEC_PROCESS_HUMIDITY) {
        inputs[n_inputs].sensor_id  = BSEC_INPUT_HUMIDITY;
        inputs[n_inputs].signal     = data[0].humidity;
        inputs[n_inputs].time_stamp = ts_ns;
        n_inputs++;
    }
    if ((settings.process_data & BSEC_PROCESS_GAS) &&
            (data[0].status & BME68X_GASM_VALID_MSK)) {
        inputs[n_inputs].sensor_id  = BSEC_INPUT_GASRESISTOR;
        inputs[n_inputs].signal     = data[0].gas_resistance;
        inputs[n_inputs].time_stamp = ts_ns;
        n_inputs++;
    }
    /* Profile part: forced mode → always slot 0 */
    if ((settings.process_data & BSEC_PROCESS_PROFILE_PART) &&
            (data[0].status & BME68X_GASM_VALID_MSK)) {
        inputs[n_inputs].sensor_id  = BSEC_INPUT_PROFILE_PART;
        inputs[n_inputs].signal     = 0.0f;
        inputs[n_inputs].time_stamp = ts_ns;
        n_inputs++;
    }

    if (n_inputs == 0) {
        /* No data matched the process_data bitmask */
        int64_t sleep_us = (settings.next_call / 1000LL) - (int64_t)time_us_64();
        if (sleep_us < 0) sleep_us = 0;
        mp_obj_t ret[2] = { mp_obj_new_int(clamp_sleep_us(sleep_us)), mp_const_none };
        return mp_obj_new_tuple(2, ret);
    }

    /* --- 10. BSEC processing -------------------------------------------- */
    bsec_output_t bsec_outputs[BSEC_NUMBER_OUTPUTS];
    uint8_t n_outputs = BSEC_NUMBER_OUTPUTS;
    rc = bsec_do_steps(inst, inputs, n_inputs, bsec_outputs, &n_outputs);
    if (rc < 0) {
        mp_raise_msg_varg(&mp_type_RuntimeError,
            MP_ERROR_TEXT("bsec_do_steps failed: %d"), (int)rc);
    }

    /* --- 11. Compute sleep time AFTER all processing -------------------- */
    /* Subtract measurement + processing time already elapsed from next_call */
    int64_t sleep_us = (settings.next_call / 1000LL) - (int64_t)time_us_64();
    if (sleep_us < 0) sleep_us = 0;

    if (n_outputs == 0) {
        /* BSEC produced no outputs (can happen during warm-up) */
        mp_obj_t ret[2] = { mp_obj_new_int(clamp_sleep_us(sleep_us)), mp_const_none };
        return mp_obj_new_tuple(2, ret);
    }

    /* --- 12. Build outputs list: [(sensor_id, signal, accuracy), ...] --- */
    mp_obj_t outputs_list = mp_obj_new_list(n_outputs, NULL);
    for (uint8_t i = 0; i < n_outputs; i++) {
        mp_obj_t tup[3] = {
            mp_obj_new_int(bsec_outputs[i].sensor_id),
            mp_obj_new_float(bsec_outputs[i].signal),
            mp_obj_new_int(bsec_outputs[i].accuracy),
        };
        mp_obj_list_store(outputs_list, MP_OBJ_NEW_SMALL_INT(i),
                          mp_obj_new_tuple(3, tup));
    }

    mp_obj_t ret[2] = { mp_obj_new_int(clamp_sleep_us(sleep_us)), outputs_list };
    return mp_obj_new_tuple(2, ret);
}
static MP_DEFINE_CONST_FUN_OBJ_2(bme68x_run_cycle_obj, bme68x_run_cycle);

/* -------------------------------------------------------------------------
 * Type definition
 * ------------------------------------------------------------------------- */
static const mp_rom_map_elem_t bme68x_locals_table[] = {
    { MP_ROM_QSTR(MP_QSTR_run_cycle), MP_ROM_PTR(&bme68x_run_cycle_obj) },
};
static MP_DEFINE_CONST_DICT(bme68x_locals_dict, bme68x_locals_table);

MP_DEFINE_CONST_OBJ_TYPE(
    bme68x_type,
    MP_QSTR_BME68X,
    MP_TYPE_FLAG_NONE,
    make_new, bme68x_make_new,
    locals_dict, &bme68x_locals_dict
);

/* -------------------------------------------------------------------------
 * Module definition
 * ------------------------------------------------------------------------- */
static const mp_rom_map_elem_t bme68x_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_bme68x) },
    { MP_ROM_QSTR(MP_QSTR_BME68X),  MP_ROM_PTR(&bme68x_type)    },
};
static MP_DEFINE_CONST_DICT(bme68x_module_globals, bme68x_module_globals_table);

const mp_obj_module_t bme68x_module = {
    .base    = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&bme68x_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_bme68x, bme68x_module);
