#include <string.h>
#include "py/obj.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "py/misc.h"
#include "bsec_datatypes.h"
#include "bsec_interface.h"

// ---------------------------------------------------------------------------
// Multi-instance management
// ---------------------------------------------------------------------------
#define BSEC3_MAX_INSTANCES 4
void *bsec3_instances[BSEC3_MAX_INSTANCES];

// Resolve an integer instance id to the allocated instance pointer.
static void *resolve_instance(mp_obj_t inst_id_obj) {
    int inst_id = mp_obj_get_int(inst_id_obj);
    if (inst_id < 0 || inst_id >= BSEC3_MAX_INSTANCES) {
        mp_raise_ValueError(MP_ERROR_TEXT("Instance id out of range"));
    }
    void *inst = bsec3_instances[inst_id];
    if (inst == NULL) {
        mp_raise_ValueError(MP_ERROR_TEXT("Instance not created"));
    }
    return inst;
}

// ---------------------------------------------------------------------------
// 64-bit timestamp split/combine helpers (MicroPython has no int64 literal)
// ---------------------------------------------------------------------------
static int64_t from_uint_high_low(uint32_t high, uint32_t low) {
    return ((int64_t)high << 32) | (uint32_t)low;
}

static uint32_t to_uint_high(int64_t value) {
    return (uint32_t)((value >> 32) & 0xFFFFFFFF);
}

static uint32_t to_uint_low(int64_t value) {
    return (uint32_t)(value & 0xFFFFFFFF);
}

// ---------------------------------------------------------------------------
// Result tuple helpers
// ---------------------------------------------------------------------------
static mp_obj_t to_result_tuple(bsec_library_return_t result, mp_obj_t value) {
    mp_obj_t items[2];
    items[0] = mp_obj_new_int(result);
    items[1] = value;
    return mp_obj_new_tuple(2, items);
}

static mp_obj_t to_result_tuple_none(bsec_library_return_t result) {
    return to_result_tuple(result, mp_const_none);
}

// ---------------------------------------------------------------------------
// Sensor configuration conversion helpers
// ---------------------------------------------------------------------------
static void tuple_to_sensor_configuration(mp_obj_t tuple, bsec_sensor_configuration_t *config) {
    mp_obj_t *items;
    size_t len;
    mp_obj_get_array(tuple, &len, &items);
    if (len != 2) {
        mp_raise_ValueError(MP_ERROR_TEXT("Sensor config tuple length must be 2"));
    }
    config->sample_rate = mp_obj_get_float(items[0]);
    config->sensor_id   = mp_obj_get_int(items[1]);
}

static mp_obj_t sensor_configuration_to_tuple(const bsec_sensor_configuration_t *config) {
    mp_obj_t items[2];
    items[0] = mp_obj_new_float(config->sample_rate);
    items[1] = mp_obj_new_int(config->sensor_id);
    return mp_obj_new_tuple(2, items);
}

// ---------------------------------------------------------------------------
// BME settings struct -> Python dict
// ---------------------------------------------------------------------------
static mp_obj_t sensor_settings_to_dict(const bsec_bme_settings_t *settings) {
    mp_obj_t dict = mp_obj_new_dict(0);
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_next_call_high),            mp_obj_new_int_from_uint(to_uint_high(settings->next_call)));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_next_call_low),             mp_obj_new_int_from_uint(to_uint_low(settings->next_call)));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_process_data),              mp_obj_new_int(settings->process_data));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_heater_temperature),        mp_obj_new_int(settings->heater_temperature));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_heater_duration),           mp_obj_new_int(settings->heater_duration));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_trigger_measurement),       mp_obj_new_int(settings->trigger_measurement));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_op_mode),                   mp_obj_new_int(settings->op_mode));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_run_gas),                   mp_obj_new_int(settings->run_gas));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_pressure_oversampling),     mp_obj_new_int(settings->pressure_oversampling));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_temperature_oversampling),  mp_obj_new_int(settings->temperature_oversampling));
    mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_humidity_oversampling),     mp_obj_new_int(settings->humidity_oversampling));
    return dict;
}

// ---------------------------------------------------------------------------
// create_instance() -> int
// Allocates a BSEC instance and returns its slot index (0-3).
// ---------------------------------------------------------------------------
static mp_obj_t bsec3_create_instance_wrapper(void) {
    for (int i = 0; i < BSEC3_MAX_INSTANCES; i++) {
        if (bsec3_instances[i] == NULL) {
            size_t inst_size = bsec_get_instance_size();
            bsec3_instances[i] = m_malloc(inst_size);
            memset(bsec3_instances[i], 0, inst_size);
            return mp_obj_new_int(i);
        }
    }
    mp_raise_ValueError(MP_ERROR_TEXT("No free instance slots"));
}
static MP_DEFINE_CONST_FUN_OBJ_0(bsec3_create_instance_obj, bsec3_create_instance_wrapper);

// ---------------------------------------------------------------------------
// destroy_instance(inst_id) -> None
// ---------------------------------------------------------------------------
static mp_obj_t bsec3_destroy_instance_wrapper(mp_obj_t inst_id_obj) {
    int inst_id = mp_obj_get_int(inst_id_obj);
    if (inst_id < 0 || inst_id >= BSEC3_MAX_INSTANCES) {
        mp_raise_ValueError(MP_ERROR_TEXT("Instance id out of range"));
    }
    if (bsec3_instances[inst_id] != NULL) {
        m_free(bsec3_instances[inst_id]);
        bsec3_instances[inst_id] = NULL;
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(bsec3_destroy_instance_obj, bsec3_destroy_instance_wrapper);

// ---------------------------------------------------------------------------
// init(inst_id) -> int
// ---------------------------------------------------------------------------
static mp_obj_t bsec3_init_wrapper(mp_obj_t inst_id_obj) {
    void *inst = resolve_instance(inst_id_obj);
    bsec_library_return_t result = bsec_init(inst);
    return mp_obj_new_int(result);
}
static MP_DEFINE_CONST_FUN_OBJ_1(bsec3_init_obj, bsec3_init_wrapper);

// ---------------------------------------------------------------------------
// get_version(inst_id) -> (rc, (major, minor, major_bugfix, minor_bugfix))
// ---------------------------------------------------------------------------
static mp_obj_t bsec3_get_version_wrapper(mp_obj_t inst_id_obj) {
    void *inst = resolve_instance(inst_id_obj);
    bsec_version_t version;
    bsec_library_return_t result = bsec_get_version(inst, &version);
    if (result != BSEC_OK) {
        return to_result_tuple_none(result);
    }
    mp_obj_t ver[4];
    ver[0] = mp_obj_new_int(version.major);
    ver[1] = mp_obj_new_int(version.minor);
    ver[2] = mp_obj_new_int(version.major_bugfix);
    ver[3] = mp_obj_new_int(version.minor_bugfix);
    return to_result_tuple(result, mp_obj_new_tuple(4, ver));
}
static MP_DEFINE_CONST_FUN_OBJ_1(bsec3_get_version_obj, bsec3_get_version_wrapper);

// ---------------------------------------------------------------------------
// set_configuration(inst_id, config_bytes) -> int
// ---------------------------------------------------------------------------
static mp_obj_t bsec3_set_configuration_wrapper(mp_obj_t inst_id_obj, mp_obj_t config_obj) {
    void *inst = resolve_instance(inst_id_obj);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(config_obj, &bufinfo, MP_BUFFER_READ);
    if (bufinfo.len > BSEC_MAX_PROPERTY_BLOB_SIZE) {
        mp_raise_ValueError(MP_ERROR_TEXT("Configuration size too large"));
    }
    uint8_t work_buffer[BSEC_MAX_WORKBUFFER_SIZE];
    bsec_library_return_t result = bsec_set_configuration(
        inst,
        (const uint8_t *)bufinfo.buf, bufinfo.len,
        work_buffer, BSEC_MAX_WORKBUFFER_SIZE
    );
    return mp_obj_new_int(result);
}
static MP_DEFINE_CONST_FUN_OBJ_2(bsec3_set_configuration_obj, bsec3_set_configuration_wrapper);

// ---------------------------------------------------------------------------
// get_state(inst_id) -> (rc, bytes)
// ---------------------------------------------------------------------------
static mp_obj_t bsec3_get_state_wrapper(mp_obj_t inst_id_obj) {
    void *inst = resolve_instance(inst_id_obj);
    uint8_t state_buffer[BSEC_MAX_STATE_BLOB_SIZE];
    uint8_t work_buffer[BSEC_MAX_WORKBUFFER_SIZE];
    uint32_t n_serialized_state;
    bsec_library_return_t result = bsec_get_state(
        inst, 0,
        state_buffer, BSEC_MAX_STATE_BLOB_SIZE,
        work_buffer,  BSEC_MAX_WORKBUFFER_SIZE,
        &n_serialized_state
    );
    if (result != BSEC_OK) {
        return to_result_tuple_none(result);
    }
    return to_result_tuple(result, mp_obj_new_bytes(state_buffer, n_serialized_state));
}
static MP_DEFINE_CONST_FUN_OBJ_1(bsec3_get_state_obj, bsec3_get_state_wrapper);

// ---------------------------------------------------------------------------
// set_state(inst_id, state_bytes) -> int
// ---------------------------------------------------------------------------
static mp_obj_t bsec3_set_state_wrapper(mp_obj_t inst_id_obj, mp_obj_t state_obj) {
    void *inst = resolve_instance(inst_id_obj);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(state_obj, &bufinfo, MP_BUFFER_READ);
    if (bufinfo.len > BSEC_MAX_STATE_BLOB_SIZE) {
        mp_raise_ValueError(MP_ERROR_TEXT("State size too large"));
    }
    uint8_t work_buffer[BSEC_MAX_WORKBUFFER_SIZE];
    bsec_library_return_t result = bsec_set_state(
        inst,
        (const uint8_t *)bufinfo.buf, bufinfo.len,
        work_buffer, BSEC_MAX_WORKBUFFER_SIZE
    );
    return mp_obj_new_int(result);
}
static MP_DEFINE_CONST_FUN_OBJ_2(bsec3_set_state_obj, bsec3_set_state_wrapper);

// ---------------------------------------------------------------------------
// update_subscription(inst_id, [(sample_rate, sensor_id), ...])
//   -> (rc, [(sample_rate, sensor_id), ...])
// ---------------------------------------------------------------------------
static mp_obj_t bsec3_update_subscription_wrapper(mp_obj_t inst_id_obj, mp_obj_t requested_obj) {
    void *inst = resolve_instance(inst_id_obj);
    mp_obj_t *requested_items;
    size_t n_requested;
    mp_obj_get_array(requested_obj, &n_requested, &requested_items);

    bsec_sensor_configuration_t requested_configs[n_requested];
    bsec_sensor_configuration_t required_configs[BSEC_MAX_PHYSICAL_SENSOR];

    for (size_t i = 0; i < n_requested; i++) {
        tuple_to_sensor_configuration(requested_items[i], &requested_configs[i]);
    }

    uint8_t n_required = BSEC_MAX_PHYSICAL_SENSOR;
    bsec_library_return_t result = bsec_update_subscription(
        inst,
        requested_configs, (uint8_t)n_requested,
        required_configs,  &n_required
    );
    // Negative rc = error (return None); zero or positive = OK/warning (still
    // return the required-sensor list, which may be empty).
    if (result < 0) {
        return to_result_tuple_none(result);
    }

    mp_obj_t required_list = mp_obj_new_list(n_required, NULL);
    for (size_t i = 0; i < n_required; i++) {
        mp_obj_list_store(MP_OBJ_FROM_PTR(required_list), MP_OBJ_NEW_SMALL_INT(i),
                          sensor_configuration_to_tuple(&required_configs[i]));
    }
    return to_result_tuple(result, required_list);
}
static MP_DEFINE_CONST_FUN_OBJ_2(bsec3_update_subscription_obj, bsec3_update_subscription_wrapper);

// ---------------------------------------------------------------------------
// sensor_control(inst_id, ts_high, ts_low) -> (rc, settings_dict)
// ---------------------------------------------------------------------------
static mp_obj_t bsec3_sensor_control_wrapper(mp_obj_t inst_id_obj, mp_obj_t ts_high_obj, mp_obj_t ts_low_obj) {
    void *inst = resolve_instance(inst_id_obj);
    uint32_t ts_high = mp_obj_get_int_truncated(ts_high_obj);
    uint32_t ts_low  = mp_obj_get_int_truncated(ts_low_obj);
    bsec_bme_settings_t sensor_settings;
    bsec_library_return_t result = bsec_sensor_control(inst, from_uint_high_low(ts_high, ts_low), &sensor_settings);
    if (result != BSEC_OK) {
        return to_result_tuple_none(result);
    }
    return to_result_tuple(result, sensor_settings_to_dict(&sensor_settings));
}
static MP_DEFINE_CONST_FUN_OBJ_3(bsec3_sensor_control_obj, bsec3_sensor_control_wrapper);

// ---------------------------------------------------------------------------
// do_steps(inst_id, [(sensor_id, signal, ts_high, ts_low), ...])
//   -> (rc, [{'sensor_id', 'signal', 'accuracy', 'time_stamp_high', 'time_stamp_low'}, ...])
// ---------------------------------------------------------------------------
static mp_obj_t bsec3_do_steps_wrapper(mp_obj_t inst_id_obj, mp_obj_t inputs_obj) {
    void *inst = resolve_instance(inst_id_obj);
    mp_obj_t *input_items;
    size_t n_inputs;
    mp_obj_get_array(inputs_obj, &n_inputs, &input_items);

    bsec_input_t bsec_inputs[n_inputs];
    for (size_t i = 0; i < n_inputs; i++) {
        mp_obj_t *fields;
        size_t len;
        mp_obj_get_array(input_items[i], &len, &fields);
        if (len != 4) {
            mp_raise_ValueError(MP_ERROR_TEXT("Input tuple length must be 4"));
        }
        bsec_inputs[i].sensor_id  = mp_obj_get_int(fields[0]);
        bsec_inputs[i].signal     = mp_obj_get_float(fields[1]);
        bsec_inputs[i].time_stamp = from_uint_high_low(
            mp_obj_get_int_truncated(fields[2]),
            mp_obj_get_int_truncated(fields[3])
        );
    }

    bsec_output_t outputs[BSEC_NUMBER_OUTPUTS];
    uint8_t n_outputs = BSEC_NUMBER_OUTPUTS;
    bsec_library_return_t result = bsec_do_steps(inst, bsec_inputs, (uint8_t)n_inputs, outputs, &n_outputs);
    if (result != BSEC_OK) {
        return to_result_tuple_none(result);
    }

    mp_obj_t outputs_list = mp_obj_new_list(n_outputs, NULL);
    for (size_t i = 0; i < n_outputs; i++) {
        mp_obj_t dict = mp_obj_new_dict(0);
        mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_sensor_id),       mp_obj_new_int(outputs[i].sensor_id));
        mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_signal),          mp_obj_new_float(outputs[i].signal));
        mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_accuracy),        mp_obj_new_int(outputs[i].accuracy));
        mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_time_stamp_high), mp_obj_new_int_from_uint(to_uint_high(outputs[i].time_stamp)));
        mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(MP_QSTR_time_stamp_low),  mp_obj_new_int_from_uint(to_uint_low(outputs[i].time_stamp)));
        mp_obj_list_store(outputs_list, MP_OBJ_NEW_SMALL_INT(i), dict);
    }
    return to_result_tuple(result, outputs_list);
}
static MP_DEFINE_CONST_FUN_OBJ_2(bsec3_do_steps_obj, bsec3_do_steps_wrapper);

// ---------------------------------------------------------------------------
// reset_output(inst_id, sensor_id) -> int   [new in BSEC v3]
// Resets a specific virtual sensor output (e.g. zeroes IAQ to ambient).
// ---------------------------------------------------------------------------
static mp_obj_t bsec3_reset_output_wrapper(mp_obj_t inst_id_obj, mp_obj_t sensor_id_obj) {
    void *inst = resolve_instance(inst_id_obj);
    uint8_t sensor_id = (uint8_t)mp_obj_get_int(sensor_id_obj);
    bsec_library_return_t result = bsec_reset_output(inst, sensor_id);
    return mp_obj_new_int(result);
}
static MP_DEFINE_CONST_FUN_OBJ_2(bsec3_reset_output_obj, bsec3_reset_output_wrapper);

// ---------------------------------------------------------------------------
// Module definition
// ---------------------------------------------------------------------------
static const mp_rom_map_elem_t bsec3_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),          MP_ROM_QSTR(MP_QSTR_bsec3) },
    { MP_ROM_QSTR(MP_QSTR_create_instance),   MP_ROM_PTR(&bsec3_create_instance_obj) },
    { MP_ROM_QSTR(MP_QSTR_destroy_instance),  MP_ROM_PTR(&bsec3_destroy_instance_obj) },
    { MP_ROM_QSTR(MP_QSTR_init),              MP_ROM_PTR(&bsec3_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_version),       MP_ROM_PTR(&bsec3_get_version_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_configuration), MP_ROM_PTR(&bsec3_set_configuration_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_state),         MP_ROM_PTR(&bsec3_get_state_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_state),         MP_ROM_PTR(&bsec3_set_state_obj) },
    { MP_ROM_QSTR(MP_QSTR_update_subscription), MP_ROM_PTR(&bsec3_update_subscription_obj) },
    { MP_ROM_QSTR(MP_QSTR_sensor_control),    MP_ROM_PTR(&bsec3_sensor_control_obj) },
    { MP_ROM_QSTR(MP_QSTR_do_steps),          MP_ROM_PTR(&bsec3_do_steps_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset_output),      MP_ROM_PTR(&bsec3_reset_output_obj) },
};

static MP_DEFINE_CONST_DICT(bsec3_module_globals, bsec3_module_globals_table);

const mp_obj_module_t bsec3_module = {
    .base    = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&bsec3_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_bsec3, bsec3_module);
