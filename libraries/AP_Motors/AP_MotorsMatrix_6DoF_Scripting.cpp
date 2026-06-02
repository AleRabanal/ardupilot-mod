/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_Scripting/AP_Scripting_config.h>

#if AP_SCRIPTING_ENABLED

#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsMatrix_6DoF_Scripting.h"
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>


extern const AP_HAL::HAL& hal;
void AP_MotorsMatrix_6DoF_Scripting::output_to_motors()
{
    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
        case SpoolState::GROUND_IDLE:
        {
            // Motores de empuje apagados y servos de inclinación al centro neutro (1500us)
            for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    if (i >= 6 && i < 12) {
                        _actuator[i] = 0.5f; // Centro físico exacto -> Traducirá a 1500us
                        _last_servo_angle_rad[i - 6] = 0.0f; // Resetea histórico del unwrap
                    } else if (i < 6) {
                        _actuator[i] = 0.0f; // Motores de empuje apagados
                    }
                }
            }
            break;
        }
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            // EN VUELO: Convertimos las señales de mezcla en comandos físicos reales
            for (uint8_t i = 0; i < 6; i++) {
                // F_v proviene de la matriz de empuje vertical (0..5)
                float F_v = _thrust_rpyt_out[i];     
                
                // F_l proviene de la matriz de componentes laterales (6..11)
                float F_l = _thrust_rpyt_out[i + 6]; 

                // --- PROTECCIÓN ANTI-INVERSIÓN VECTORIAL ---
                // Si el PID pide empuje negativo, lo limitamos a 0 para que el servo
                // no intente dar una vuelta completa de 180 grados de forma brusca.
                if (F_v < 0.0f) {
                    F_v = 0.0f;
                }

                // --- CÓMPUTO DEL MOTOR REAL ---
                float thrust_mod = safe_sqrt((F_v * F_v) + (F_l * F_l));
                
                // Ralentí dinámico de protección
                if (thrust_mod < 0.08f) {
                    thrust_mod = 0.08f; 
                }
                
                thrust_mod = constrain_float(thrust_mod, 0.0f, 1.0f);
                _actuator[i] = thr_lin.thrust_to_actuator(thrust_mod);

                // --- CÓMPUTO DEL SERVO REAL ---
                float angle_rad = atan2f(F_l, F_v + 0.001f);

                // Algoritmo Unwrap para evitar saltos bruscos
                float diff = angle_rad - _last_servo_angle_rad[i];
                while (diff < -M_PI)  diff += 2.0f * M_PI;
                while (diff > M_PI)   diff -= 2.0f * M_PI;

                float unwrapped_angle = _last_servo_angle_rad[i] + diff;
                
                // Rango límite configurado a PI debido al mapeo del SDF en Gazebo
                float max_limit_rad = M_PI; 
                unwrapped_angle = constrain_float(unwrapped_angle, -max_limit_rad, max_limit_rad);
                _last_servo_angle_rad[i] = unwrapped_angle;

                // Guardamos el valor en el actuador del servo correspondiente (índices 6..11)
                _actuator[i + 6] = 0.5f * (unwrapped_angle / max_limit_rad) + 0.5f;
            }
            break;
    }

    // Envío de señales físicas a los canales de salida asignados
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            if (i < 6) {
                // Motores físicos (0..5): rango esperado [0, 4500]
                SRV_Channels::set_output_scaled(SRV_Channels::get_motor_function(i), _actuator[i] * 4500);
            } else if (i >= 6 && i < 12) {
                // Servos físicos (6..11): Mapeo directo a microsegundos [1000, 2000]
                float pwm_output = 1000.0f + (_actuator[i] * 1000.0f);
                pwm_output = constrain_float(pwm_output, 1000.0f, 2000.0f);
                
                SRV_Channels::set_output_pwm(SRV_Channels::get_motor_function(i), (uint16_t)pwm_output);
            }
        }
    }
}
// output_armed - sends commands to the motors
void AP_MotorsMatrix_6DoF_Scripting::output_armed_stabilizing()
{
    uint8_t i;                          // general purpose counter
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    float   forward_thrust;             // forward thrust input value, +/- 1.0
    float   right_thrust;               // right thrust input value, +/- 1.0

    // apply voltage and air pressure compensation
    const float compensation_gain = thr_lin.get_compensation_gain();
    roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    pitch_thrust = (_pitch_in + _pitch_in_ff) * compensation_gain;
    yaw_thrust = (_yaw_in + _yaw_in_ff) * compensation_gain;
    throttle_thrust = get_throttle() * compensation_gain;

    // scale horizontal thrust with throttle, this mimics a normal copter
    forward_thrust = get_forward() * throttle_thrust;
    right_thrust = get_lateral() * throttle_thrust;

    // set throttle limit flags
    if (throttle_thrust <= 0) {
        throttle_thrust = 0;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= 1) {
        throttle_thrust = 1;
        limit.throttle_upper = true;
    }

    // rotate the thrust into bodyframe
    Matrix3f rot;
    Vector3f thrust_vec;
    rot.from_euler312(_roll_offset, _pitch_offset, 0.0f);

    /* upwards thrust, independent of orientation */
    thrust_vec.x = 0.0f;
    thrust_vec.y = 0.0f;
    thrust_vec.z = throttle_thrust;
    thrust_vec = rot * thrust_vec;
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] =  thrust_vec.x * _forward_factor[i];
            _thrust_rpyt_out[i] += thrust_vec.y * _right_factor[i];
            _thrust_rpyt_out[i] += thrust_vec.z * _throttle_factor[i];

            if (fabsf(_thrust_rpyt_out[i]) >= 1) {
                limit.throttle_upper = true;
            }
            _thrust_rpyt_out[i] = constrain_float(_thrust_rpyt_out[i], -1.0f, 1.0f);
        }
    }

    /* rotations: roll, pitch and yaw */
    float rpy_ratio = 1.0f;  
    float thrust[AP_MOTORS_MAX_NUM_MOTORS];
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            thrust[i] =  roll_thrust * _roll_factor[i];
            thrust[i] += pitch_thrust * _pitch_factor[i];
            thrust[i] += yaw_thrust * _yaw_factor[i];
            float total_thrust = _thrust_rpyt_out[i] + thrust[i];
            if (total_thrust > 1.0f) {
                rpy_ratio = MIN(rpy_ratio, (1.0f - _thrust_rpyt_out[i]) / thrust[i]);
            } else if (total_thrust < -1.0f) {
                rpy_ratio = MIN(rpy_ratio, (-1.0f - _thrust_rpyt_out[i]) / thrust[i]);
            }
        }
    }

    if (rpy_ratio < 1) {
        limit.roll = true;
        limit.pitch = true;
        limit.yaw = true;
    }

    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = constrain_float(_thrust_rpyt_out[i] + thrust[i] * rpy_ratio, -1.0f, 1.0f);
        }
    }

    /* forward and lateral, independent of orientation */
    thrust_vec.x = forward_thrust;
    thrust_vec.y = right_thrust;
    thrust_vec.z = 0.0f;
    thrust_vec = rot * thrust_vec;

    float horz_ratio = 1.0f; 
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            thrust[i] =  thrust_vec.x * _forward_factor[i];
            thrust[i] += thrust_vec.y * _right_factor[i];
            thrust[i] += thrust_vec.z * _throttle_factor[i];
            float total_thrust = _thrust_rpyt_out[i] + thrust[i];
            if (total_thrust > 1.0f) {
                horz_ratio = MIN(horz_ratio, (1.0f - _thrust_rpyt_out[i]) / thrust[i]);
            } else if (total_thrust < -1.0f) {
                horz_ratio = MIN(horz_ratio, (-1.0f - _thrust_rpyt_out[i]) / thrust[i]);
            }
        }
    }

    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = constrain_float(_thrust_rpyt_out[i] + thrust[i] * horz_ratio, -1.0f, 1.0f);
        }
    }

    /* apply deadzone to reversible motors (Only applied if _reversible is true) */
    const float deadzone = constrain_float(_yaw_headroom.get() * 0.001f, 0.0f, 0.25f);
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i] && _reversible[i]) {
            if (is_negative(_thrust_rpyt_out[i])) {
                if ((_thrust_rpyt_out[i] > -deadzone) && is_positive(_last_thrust_out[i])) {
                    _thrust_rpyt_out[i] = 0.0f;
                } else {
                    _last_thrust_out[i] = _thrust_rpyt_out[i];
                }
            } else if (is_positive(_thrust_rpyt_out[i])) {
                if ((_thrust_rpyt_out[i] < deadzone) && is_negative(_last_thrust_out[i])) {
                    _thrust_rpyt_out[i] = 0.0f;
                } else {
                    _last_thrust_out[i] = _thrust_rpyt_out[i];
                }
            }
        }
    }
}

void AP_MotorsMatrix_6DoF_Scripting::set_roll_pitch(float roll_deg, float pitch_deg)
{
    _roll_offset = radians(roll_deg);
    _pitch_offset = radians(pitch_deg);
}

void AP_MotorsMatrix_6DoF_Scripting::add_motor(int8_t motor_num, float roll_factor, float pitch_factor, float yaw_factor, float throttle_factor, float forward_factor, float right_factor, bool reversible, uint8_t testing_order)
{
    if (initialised_ok()) {
        return;
    }

    if (motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS) {
        motor_enabled[motor_num] = true;

        _roll_factor[motor_num] = roll_factor;
        _pitch_factor[motor_num] = pitch_factor;
        _yaw_factor[motor_num] = yaw_factor;

        _throttle_factor[motor_num] = throttle_factor;
        _forward_factor[motor_num] = forward_factor;
        _right_factor[motor_num] = right_factor;

        _test_order[motor_num] = testing_order;

        SRV_Channel::Aux_servo_function_t function = SRV_Channels::get_motor_function(motor_num);
        SRV_Channels::set_aux_channel_default(function, motor_num);

        uint8_t chan;
        if (!SRV_Channels::find_channel(function, chan)) {
            gcs().send_text(MAV_SEVERITY_ERROR, "Motors: unable to setup motor %u", motor_num);
            return;
        }

        _reversible[motor_num] = reversible;

        // Separación explícitamente por número de motor/actuador para independizar la lógica de ángulo
        if (motor_num >= 6) {
            // Canales 6 al 11 correspondientes a Servos de Inclinación Física: Forzamos ángulo simétrico sin zonas muertas de motor
            SRV_Channels::set_angle(function, 36000);
            SRV_Channels::set_trim_to_pwm_for(function, 1500); // Centro físico en 1500us
            SRV_Channels::set_output_min_max(function, 1000, 2000); // Rango estándar extendido para servos
        } else {
            // Canales 0 al 5 correspondientes a Motores de Empuje Axial
            if (_reversible[motor_num]) {
                SRV_Channels::set_angle(function, 4500);
                SRV_Channels::set_trim_to_pwm_for(function, 1500);
            } else {
                SRV_Channels::set_range(function, 4500); // Rango estándar lineal [0, 4500]
            }
            SRV_Channels::set_output_min_max(function, get_pwm_output_min(), get_pwm_output_max());
        }
    }
}

bool AP_MotorsMatrix_6DoF_Scripting::init(uint8_t expected_num_motors) {
    uint8_t num_motors = 0;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            num_motors++;
        }
        // Inicializa el array de unwrap a cero
        _last_servo_angle_rad[i] = 0.0f;
    }

    set_initialised_ok(expected_num_motors == num_motors);

    if (!initialised_ok()) {
        _mav_type = MAV_TYPE_GENERIC;
        return false;
    }

    switch (num_motors) {
        case 3:  _mav_type = MAV_TYPE_TRICOPTER;    break;
        case 4:  _mav_type = MAV_TYPE_QUADROTOR;    break;
        case 6:  _mav_type = MAV_TYPE_HEXAROTOR;    break;
        case 8:  _mav_type = MAV_TYPE_OCTOROTOR;    break;
        case 10: _mav_type = MAV_TYPE_DECAROTOR;    break;
        case 12: _mav_type = MAV_TYPE_DODECAROTOR;   break;
        default: _mav_type = MAV_TYPE_GENERIC;
    }

    return true;
}

AP_MotorsMatrix_6DoF_Scripting *AP_MotorsMatrix_6DoF_Scripting::_singleton;

#endif // AP_SCRIPTING_ENABLED