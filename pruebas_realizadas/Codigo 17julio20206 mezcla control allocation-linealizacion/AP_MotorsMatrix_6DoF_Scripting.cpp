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
#include <AP_Logger/AP_Logger.h> // <-- AÑADE ESTA LÍNEA
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;
void AP_MotorsMatrix_6DoF_Scripting::output_to_motors()
{
    // static uint8_t decim = 0;
    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
        case SpoolState::GROUND_IDLE:
        {
            // Motores de empuje apagados y servos de inclinación al centro neutro (1500us)
            for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    if (_is_servo[i]) {
                        _actuator[i] = 0.5f; // Centro físico exacto -> Traducirá a 1500us
                        _last_servo_angle_rad[i - 6] = 0.0f; // Resetea histórico del unwrap
                    } else {
                        _actuator[i] = 0.0f; // Motores de empuje apagados
                    }
                }
            }
            break;
        }
       case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
        {

            switch(_hw_mapping) {
                case HardwareMapping::DIRECT:

                    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
                        if (motor_enabled[i]) {
                            if (_reversible[i]) {
                                // revesible motor can provide both positive and negative thrust, +- spin max, spin min does not apply
                                if (is_positive(_thrust_rpyt_out[i])) { 
                                    _actuator[i] = thr_lin.apply_thrust_curve_and_volt_scaling(_thrust_rpyt_out[i]) * thr_lin.get_spin_max();

                                } else if (is_negative(_thrust_rpyt_out[i])) {
                                    _actuator[i] = -thr_lin.apply_thrust_curve_and_volt_scaling(-_thrust_rpyt_out[i]) * thr_lin.get_spin_max();

                                } else {
                                    _actuator[i] = 0.0f;
                                }
                            } else {
                                // motor can only provide trust in a single direction, spin min to spin max as 'normal' copter
                                _actuator[i] = thr_lin.thrust_to_actuator(_thrust_rpyt_out[i]);
                            }
                        }
                    }
                    break;
                case HardwareMapping::TILTING_HEXA:
            
                    // EN VUELO: Convertimos las señales de mezcla en comandos físicos reales
                    float thrusts_mod[6];
                    float max_thrust = 0.0f;
                    float F_v[6]; // Componente vertical del empuje para cada motor
                    float F_l[6]; // Componente lateral del empuje para cada motor

                        // PASO 1: Calcular la magnitud bruta que pide el chasis para cada uno de los 6 motores
                        for (uint8_t i = 0; i < 6; i++) {
                            F_v[i] = _thrust_rpyt_out[i];     
                            F_l[i] = _thrust_rpyt_out[i + 6]; 

                            // CORRECCIÓN: Cálculo de magnitud seguro al estilo ArduPilot
                            thrusts_mod[i] = safe_sqrt(sq(F_v[i]) + sq(F_l[i]));

                            if (thrusts_mod[i] > max_thrust) {
                                max_thrust = thrusts_mod[i];
                            }
                        }

                        // Si algún motor se satura por encima de 1.0, atenuamos todos en la misma proporción 
                        // para que el dron no pierda estabilidad ni guiñada durante las traslaciones 6DoF.
                        float scale = (max_thrust > 1.0f) ? (1.0f / max_thrust) : 1.0f;
                        const float max_limit_rad = 2.0f*2.0f * M_PI; // Límite de ángulo de inclinación de los servos (2 vueltas completas = 4π radianes)

                        
                                AP::logger().Write( 
                                "TILT", "TimeUS,T1,T2,T3,T4,T5,T6", "Qffffff", 
                                AP_HAL::micros64(),
                                thrusts_mod[0], 
                                thrusts_mod[1], 
                                thrusts_mod[2], 
                                thrusts_mod[3], 
                                thrusts_mod[4], 
                                thrusts_mod[5]);

                                AP::logger().Write( 
                                "FV", "TimeUS,T1,T2,T3,T4,T5,T6", "Qffffff", 
                                AP_HAL::micros64(),
                                F_v[0], 
                                F_v[1], 
                                F_v[2], 
                                F_v[3], 
                                F_v[4], 
                                F_v[5]);

                                AP::logger().Write( 
                                "FL", "TimeUS,T1,T2,T3,T4,T5,T6", "Qffffff", 
                                AP_HAL::micros64(),
                                F_l[0], 
                                F_l[1], 
                                F_l[2], 
                                F_l[3], 
                                F_l[4], 
                                F_l[5]);


                        float thrust_mod_scaled[6];     
                        float angle_rad[6];
                    // float unwrapped_angle[6];
                        // PASO 2: Calcular el ángulo del servo y aplicar el empuje escalado a cada motor
                        for (uint8_t j = 0; j < 6; j++) {
                            // Volvemos a leer los componentes locales correspondientes al motor 'j'
                        

                            // --- CÓMPUTO DEL SERVO REAL (Dirección del vector) ---
                            angle_rad[j] = atan2f(F_l[j], F_v[j]);

                            const float max_step = 0.005f; // Límite de paso máximo por ciclo 
                            
                            // CORRECCIÓN: Usar angle_rad[j] (lo que calculaste arriba)
                            float error = angle_rad[j] - _angulo_acumulado[j];
                            
                            // Buscamos el camino más corto
                            error = wrap_PI(error);
                            
                            // CORRECCIÓN VITAL: Limitar la velocidad usando max_step, no max_limit_rad!
                            error = constrain_float(error, -max_step, max_step);
                            
                            // Actualizamos el estado real del servo
                            _angulo_acumulado[j] += error;
                            
                            // Guardamos en la variable unwrapped_angle para tu logger
                        // unwrapped_angle[j] = _angulo_acumulado[j]; 

                            // --- CÓMPUTO DEL MOTOR REAL (Módulo escalado) ---
                            thrust_mod_scaled[j] = thrusts_mod[j] * scale;
                            
                            _actuator[j] = thr_lin.thrust_to_actuator(thrust_mod_scaled[j]);

                            // Guardamos la salida normalizada para el servo correspondiente (mapeo a 0.0 - 1.0)
                            _actuator[j + 6] = 0.5f * (_angulo_acumulado[j] / max_limit_rad) + 0.5f;
                        

                        

                            _actuator[j] = thr_lin.thrust_to_actuator(thrust_mod_scaled[j]);

                        }

                        AP::logger().Write( 
                                "ANG", "TimeUS,T1,T2,T3,T4,T5,T6", "Qffffff", 
                                AP_HAL::micros64(),
                                angle_rad[0], 
                                angle_rad[1], 
                                angle_rad[2], 
                                angle_rad[3], 
                                angle_rad[4], 
                                angle_rad[5]);


                            AP::logger().Write( 
                                "UW", "TimeUS,T1,T2,T3,T4,T5,T6", "Qffffff", 
                                AP_HAL::micros64(),
                                _angulo_acumulado[0], 
                                _angulo_acumulado[1], 
                                _angulo_acumulado[2], 
                                _angulo_acumulado[3], 
                                _angulo_acumulado[4], 
                                _angulo_acumulado[5]);

                                AP::logger().Write( 
                                "TSCL", "TimeUS,T1,T2,T3,T4,T5,T6", "Qffffff", 
                                AP_HAL::micros64(),
                                thrust_mod_scaled[0], 
                                thrust_mod_scaled[1], 
                                thrust_mod_scaled[2], 
                                thrust_mod_scaled[3], 
                                thrust_mod_scaled[4], 
                                thrust_mod_scaled[5]);
                            break;
                     }


            break;
        }
    }

// Envío de señales físicas a los canales de salida asignados
     for (uint8_t i = 0; i < _num_actuators; i++) {
        if (motor_enabled[i]) {
            // Obtenemos la "función" asignada a este motor (ej. k_motor1, k_motor7...)
            SRV_Channel::Aux_servo_function_t function = SRV_Channels::get_motor_function(i);

            if (_is_servo[i]) {
                // 1. Valores por defecto por si el canal no está asignado
                uint16_t pwm_min = 1000;
                uint16_t pwm_max = 2000;
                uint16_t pwm_trim = 1500;

                SRV_Channel *chan = SRV_Channels::get_channel_for(function);
                if (chan != nullptr) {
                    pwm_min = chan->get_output_min();
                    pwm_max = chan->get_output_max();
                    pwm_trim = chan->get_trim(); // <-- EL CENTRO FÍSICO REAL
                }

                // _actuator[i] va de 0.0 a 1.0 (donde 0.5 es el centro/0 grados)
                float pwm_output;
                
                if (_actuator[i] <= 0.5f) {
                    // Mapeamos la mitad inferior: de 0.0 a 0.5 -> de MIN a TRIM
                    // Normalizamos el rango de 0.0 a 0.5 para que vaya de 0.0 a 1.0
                    float scale = _actuator[i] * 2.0f; 
                    pwm_output = pwm_min + (scale * (pwm_trim - pwm_min));
                } else {
                    // Mapeamos la mitad superior: de 0.5 a 1.0 -> de TRIM a MAX
                    // Normalizamos el rango de 0.5 a 1.0 para que vaya de 0.0 a 1.0
                    float scale = (_actuator[i] - 0.5f) * 2.0f;
                    pwm_output = pwm_trim + (scale * (pwm_max - pwm_trim));
                }

                pwm_output = constrain_float(pwm_output, pwm_min, pwm_max);
                SRV_Channels::set_output_pwm(function, (uint16_t)pwm_output);

            } else {
                // Motores físicos: rango esperado [0, 4500] (Se encarga ArduPilot automáticamente de usar los límites de ESC)
                SRV_Channels::set_output_scaled(function, _actuator[i] * 4500);
            }
        }
    }
}
// output_armed - sends commands to the motors
void AP_MotorsMatrix_6DoF_Scripting::output_armed_stabilizing()
{
    // 1. IMPORTANTE: Re-habilitamos el check del allocator
    if (!_allocator_initialized) {
        return;
    }

    const float compensation_gain = thr_lin.get_compensation_gain();
    float wrench[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; 

    // OBTENER TORQUES
    wrench[0] = (_roll_in + _roll_in_ff) * compensation_gain;
    wrench[1] = (_pitch_in + _pitch_in_ff) * compensation_gain;
    wrench[2] = (_yaw_in + _yaw_in_ff) * compensation_gain;

    // OBTENER FUERZAS
    if (_use_earth_thrust) {
        Matrix3f rot_earth_to_body = AP::ahrs().get_rotation_body_to_ned().transposed();
        Vector3f thrust_body = rot_earth_to_body * _earth_thrust_vector;
        wrench[3] = thrust_body.x * compensation_gain;
        wrench[4] = thrust_body.y * compensation_gain;
        wrench[5] = -thrust_body.z * compensation_gain;
        _use_earth_thrust = false; 
    } else {
        wrench[3] = get_forward() * compensation_gain;
        wrench[4] = get_lateral() * compensation_gain;
        wrench[5] = get_throttle() * compensation_gain;
        
        Matrix3f rot_offset;
        rot_offset.from_euler312(_roll_offset, _pitch_offset, 0.0f);
        Vector3f combined_forces = rot_offset * Vector3f(wrench[3], wrench[4], wrench[5]);
        wrench[3] = combined_forces.x;
        wrench[4] = combined_forces.y;
        wrench[5] = combined_forces.z;
    }

    float rpy_thrust[AP_MOTORS_MAX_NUM_MOTORS] = {0.0f}; // INICIALIZADO A CERO
    float rpy_ratio = 1.0f;

    // 2. PRIMER PASO: Calcular empuje base y preparar actitud
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            // Fuerza base
            _thrust_rpyt_out[i] =  wrench[3] * _forward_factor[i] +
                                   wrench[4] * _right_factor[i] +
                                   wrench[5] * _throttle_factor[i];

            // Actitud candidata
            rpy_thrust[i] = wrench[0] * _roll_factor[i] +
                            wrench[1] * _pitch_factor[i] +
                            wrench[2] * _yaw_factor[i];

            // Calcular ratio de saturación
            float total = _thrust_rpyt_out[i] + rpy_thrust[i];
            if (total > 1.0f && rpy_thrust[i] > 0.001f) {
                rpy_ratio = MIN(rpy_ratio, (1.0f - _thrust_rpyt_out[i]) / rpy_thrust[i]);
            } else if (total < -1.0f && rpy_thrust[i] < -0.001f) {
                rpy_ratio = MIN(rpy_ratio, (-1.0f - _thrust_rpyt_out[i]) / rpy_thrust[i]);
            }
        }
    }

    // 3. SEGUNDO PASO: Aplicar mezcla final solo a motores habilitados
    rpy_ratio = constrain_float(rpy_ratio, 0.0f, 1.0f);
    if (rpy_ratio < 1.0f) {
        limit.roll = limit.pitch = limit.yaw = true;
    }

    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) { // <--- ESTE CHECK ES VITAL
            _thrust_rpyt_out[i] = constrain_float(_thrust_rpyt_out[i] + (rpy_thrust[i] * rpy_ratio), -1.0f, 1.0f);
        } else {
            _thrust_rpyt_out[i] = 0.0f;
        }
    }
   AP::logger().Write(
    "CM",
    "TimeUS,Roll,Pitch,Rin,Pin,Rff,Pff",
    "Qffffff",
    AP_HAL::micros64(),
    AP::ahrs().get_roll(),
    AP::ahrs().get_pitch(),
    _roll_in,
    _pitch_in,
    _roll_in_ff,
    _pitch_in_ff);
    AP::logger().Write("WR", "TimeUS,T1,T2,T3,T4,T5,T6", "Qffffff", 
        AP_HAL::micros64(), wrench[0], wrench[1], wrench[2], wrench[3], wrench[4], wrench[5]);

    
}



void AP_MotorsMatrix_6DoF_Scripting::set_roll_pitch(float roll_deg, float pitch_deg)
{
    _roll_offset = radians(roll_deg);
    _pitch_offset = radians(pitch_deg);
}

void AP_MotorsMatrix_6DoF_Scripting::add_motor(int8_t motor_num, float roll_factor, float pitch_factor, float yaw_factor, float throttle_factor, float forward_factor, float right_factor, bool reversible, uint8_t testing_order, bool is_servo){
    if (initialised_ok()) {
        return;
    }

    if (motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS) {

        if (motor_num >= _num_actuators) {
            _num_actuators = motor_num + 1;
        }

        motor_enabled[motor_num] = true;

        _roll_factor[motor_num] = roll_factor;
        _pitch_factor[motor_num] = pitch_factor;
        _yaw_factor[motor_num] = yaw_factor;

        _throttle_factor[motor_num] = throttle_factor;
        _forward_factor[motor_num] = forward_factor;
        _right_factor[motor_num] = right_factor;

        _test_order[motor_num] = testing_order;
        _is_servo[motor_num] = is_servo;
        _reversible[motor_num] = reversible;    

        SRV_Channel::Aux_servo_function_t function = SRV_Channels::get_motor_function(motor_num);
        SRV_Channels::set_aux_channel_default(function, motor_num);

        uint8_t chan;
        if (!SRV_Channels::find_channel(function, chan)) {
            gcs().send_text(MAV_SEVERITY_ERROR, "Motors: unable to setup motor %u", motor_num);
            return;
        }

      

        // Separación explícitamente por número de motor/actuador para independizar la lógica de ángulo
        if (_is_servo[motor_num]) {
            // Configuración para Servos Físicos de Inclinación
            SRV_Channels::set_angle(function, 36000);
            SRV_Channels::set_trim_to_pwm_for(function, 1500); // Centro físico en 1500us
            SRV_Channels::set_output_min_max(function, 1000, 2000); 
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

void AP_MotorsMatrix_6DoF_Scripting::build_effectiveness_matrix()
{
    memset(_A, 0, sizeof(_A));
   for (uint8_t i = 0; i < _num_actuators; i++) {
        if (motor_enabled[i]) {
            _A[0][i] = _roll_factor[i];
            _A[1][i] = _pitch_factor[i];
            _A[2][i] = _yaw_factor[i];
            _A[3][i] = _forward_factor[i]; // Fx
            _A[4][i] = _right_factor[i];   // Fy
            _A[5][i] = _throttle_factor[i];// Fz
        }
    }
}


void AP_MotorsMatrix_6DoF_Scripting::compute_allocator()
{
    build_effectiveness_matrix();

    // Pasamos el tamaño real configurado desde LUA
    if (calcular_pseudoinversa(_A, _A_pinv, _num_actuators)) {
        _allocator_initialized = true;
    } else {
        _allocator_initialized = false;
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "6DoF: ¡Fallo crítico al calcular Allocator!");
    }
}

bool AP_MotorsMatrix_6DoF_Scripting::calcular_pseudoinversa(const float A[6][AP_MOTORS_MAX_NUM_MOTORS], float A_pinv[AP_MOTORS_MAX_NUM_MOTORS][6], uint8_t num_act)
{
    // Prevenir fallo matemático si no hay suficientes actuadores
    if (num_act < 6) { return false; } 

    float A_T[AP_MOTORS_MAX_NUM_MOTORS][6] = {0};

    // 1. Transpuesta
    for (uint8_t i = 0; i < 6; i++) {
        for (uint8_t j = 0; j < num_act; j++) {
            A_T[j][i] = A[i][j];
        }
    }

    // 2. Producto (A * A_T) siempre da matriz 6x6
    float AA_T_flat[36] = {0};
    for (uint8_t i = 0; i < 6; i++) {
        for (uint8_t j = 0; j < 6; j++) {
            float sum = 0.0f;
            for (uint8_t k = 0; k < num_act; k++) {
                sum += A[i][k] * A_T[k][j];
            }
            AA_T_flat[i * 6 + j] = sum;
        }
    }

    // 3. Inversión 6x6 nativa
    float AA_T_inv_flat[36] = {0};
    if (!mat_inverse(AA_T_flat, AA_T_inv_flat, 6)) {
        return false;
    }

    // 4. Multiplicación final: A_pinv = A_T * AA_T_inv
    for (uint8_t i = 0; i < num_act; i++) {
        for (uint8_t j = 0; j < 6; j++) {
            float sum = 0.0f;
            for (uint8_t k = 0; k < 6; k++) {
                sum += A_T[i][k] * AA_T_inv_flat[k * 6 + j];
            }
            A_pinv[i][j] = sum;
        }
    }

    return true;
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
    //compute_allocator();
    _allocator_initialized = true; 
    if (!initialised_ok()) {
        _mav_type = MAV_TYPE_GENERIC;
        return false;
    }

    /*switch (num_motors) {
        case 3:  _mav_type = MAV_TYPE_TRICOPTER;    break;
        case 4:  _mav_type = MAV_TYPE_QUADROTOR;    break;
        case 6:  _mav_type = MAV_TYPE_HEXAROTOR;    break;
        case 8:  _mav_type = MAV_TYPE_OCTOROTOR;    break;
        case 10: _mav_type = MAV_TYPE_DECAROTOR;    break;
        case 12: _mav_type = MAV_TYPE_DODECAROTOR;   break;
        default: _mav_type = MAV_TYPE_GENERIC;
    }*/

    _mav_type = MAV_TYPE_HEXAROTOR; // Forzamos a que sea un hexa para que QGC lo reconozca como 6DoF

    return true;
}

AP_MotorsMatrix_6DoF_Scripting *AP_MotorsMatrix_6DoF_Scripting::_singleton;

#endif // AP_SCRIPTING_ENABLED