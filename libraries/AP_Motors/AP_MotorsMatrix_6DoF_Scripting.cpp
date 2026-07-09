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
        {
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
            const float max_limit_rad = 2.0f*2.0f * M_PI; 

             
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

                const float max_step = 0.01f; // Límite de paso máximo por ciclo 
                
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
              

               

                // Ralentí dinámico de protección para evitar el apagado de la hélice en transiciones rápidas
               /* if (thrust_mod < 0.05f) {
                    thrust_mod = 0.05f; 
                }*/
                
                _actuator[j] = thr_lin.thrust_to_actuator(thrust_mod_scaled[j]);

                // Algoritmo Unwrap para evitar que el servo dé un giro completo de 360º innecesario
                /*float diff = angle_rad[j] - _last_servo_angle_rad[j];
                while (diff < -M_PI)  diff += 2.0f * M_PI;
                while (diff > M_PI)   diff -= 2.0f * M_PI;

                unwrapped_angle[j] = _last_servo_angle_rad[j] + diff;
                
                // Forzamos límites estrictos dentro del rango configurado para el SDF en Gazebo
                unwrapped_angle[j] = constrain_float(unwrapped_angle[j], -max_limit_rad, max_limit_rad);
                _last_servo_angle_rad[j] = unwrapped_angle[j];*/

                // Guardamos la salida normalizada para el servo correspondiente (mapeo a 0.0 - 1.0)
               // _actuator[j + 6] = 0.5f * (_angulo_acumulado[j] / max_limit_rad) + 0.5f;
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
    if (!_allocator_initialized) return;

    float wrench[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; 
    const float compensation_gain = thr_lin.get_compensation_gain();

    // 1. TORQUES: (Salidas de los PIDs de actitud)
    wrench[0] = (_roll_in + _roll_in_ff) * compensation_gain;
    wrench[1] = (_pitch_in + _pitch_in_ff) * compensation_gain;
    wrench[2] = (_yaw_in + _yaw_in_ff) * compensation_gain;

    if (_use_earth_thrust) {
        // Rotamos el vector de la Tierra (NED) al cuerpo del dron (FRD)
        Matrix3f rot_earth_to_body = AP::ahrs().get_rotation_body_to_ned().transposed();
        Vector3f thrust_body = rot_earth_to_body * _earth_thrust_vector;

        thrust_body *= compensation_gain;

        // Mapeamos a las fuerzas del Wrench corporal
        wrench[3] = thrust_body.x;  // Fuerza Frontal
        wrench[4] = thrust_body.y;  // Fuerza Lateral
        wrench[5] = -thrust_body.z; // Invertimos Z corporal para que subir sea POSITIVO en el Wrench

        _use_earth_thrust = false; 
    } else {
        wrench[3] = get_forward() * compensation_gain;
        wrench[4] = get_lateral() * compensation_gain;
        wrench[5] = get_throttle() * compensation_gain;
    }
    AP::logger().Write("WR", "TimeUS,T1,T2,T3,T4,T5,T6", "Qffffff", 
        AP_HAL::micros64(), wrench[0], wrench[1], wrench[2], wrench[3], wrench[4], wrench[5]);



    // 2. CONTROL ALLOCATION (Pseudoinversa)
    for (uint8_t k = 0; k < 12; k++) {
        _thrust_rpyt_out[k] = 0.0f;
        for (uint8_t j = 0; j < 6; j++) {
            _thrust_rpyt_out[k] += _A_pinv[k][j] * wrench[j];
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
            /*if (_reversible[motor_num]) {
                SRV_Channels::set_angle(function, 4500);
                SRV_Channels::set_trim_to_pwm_for(function, 1500);
            } else {
                SRV_Channels::set_range(function, 4500); // Rango estándar lineal [0, 4500]
            }*/
                            SRV_Channels::set_range(function, 4500); // Rango estándar lineal [0, 4500]

            SRV_Channels::set_output_min_max(function, get_pwm_output_min(), get_pwm_output_max());
        }
    }
}

void AP_MotorsMatrix_6DoF_Scripting::build_effectiveness_matrix()
{
    memset(_A, 0, sizeof(_A));



    for (uint8_t i = 0; i < 6; i++) {

        const float x  = rotor_x[i];
        const float y  = rotor_y[i];
        const float km = rotor_km[i];
        const float roll_scale  = 8.636f ;//1.0f / 0.275f;   // 3.636
        const float pitch_scale = 5.202f ;//1.0f / 0.238f;   // 4.202
        const float yaw_scale   = 8.636f; //1.0f / 0.275f;
        const float fx_scale    = 0.8f;
        const float fy_scale    = 0.8f;
        const float fz_scale    = 0.7f;

        float l = sqrtf(x*x + y*y);

        float cphi = x/l;
        float sphi = y/l;

        int v = i;
        int lidx = i + 6;

        // Roll
        _A[0][v]    = -y*roll_scale;
        _A[0][lidx] =  km * sphi*roll_scale;

        // Pitch
        _A[1][v]    =  x*pitch_scale;
        _A[1][lidx] = -km * cphi*pitch_scale;

        // Yaw
        _A[2][v]    =  km*yaw_scale;
        _A[2][lidx] =  (x*cphi + y*sphi)*yaw_scale;

        // Fx
        _A[3][v]    = 0.0f;
        _A[3][lidx] = -sphi*fx_scale;

        // Fy
        _A[4][v]    = 0.0f;
        _A[4][lidx] =  cphi*fy_scale;

        // Fz
        _A[5][v]    = 1.0f*fz_scale;
        _A[5][lidx] =  0.0f;
    }
}


void AP_MotorsMatrix_6DoF_Scripting::compute_allocator()
{
    build_effectiveness_matrix();

    // Reemplaza la función genérica 'pseudo_inverse_6x12' por nuestro método resuelto
    if (calcular_pseudoinversa_6x12(_A, _A_pinv)) {
        _allocator_initialized = true;
    } else {
        _allocator_initialized = false;
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "6DoF: ¡Fallo crítico al calcular Allocator!");
    }
}

bool AP_MotorsMatrix_6DoF_Scripting::calcular_pseudoinversa_6x12(const float A[6][12], float A_pinv[12][6])
{
    // 1. Calcular la transpuesta de A de forma directa: A_T (Tamaño 12x6)
    float A_T[12][6];
    for (uint8_t i = 0; i < 6; i++) {
        for (uint8_t j = 0; j < 12; j++) {
            A_T[j][i] = A[i][j];
        }
    }

    // 2. Calcular el producto intermedio: AA_T = A * A_T (Tamaño 6x6)
    // Lo guardamos en un array plano de 36 elementos (6x6) exigido por mat_inverseN
    float AA_T_flat[36] = {0};
    for (uint8_t i = 0; i < 6; i++) {
        for (uint8_t j = 0; j < 6; j++) {
            float sum = 0.0f;
            for (uint8_t k = 0; k < 12; k++) {
                sum += A[i][k] * A_T[k][j];
            }
            AA_T_flat[i * 6 + j] = sum;
        }
    }

    // 3. Invertir la matriz cuadrada usando la función nativa real de ArduPilot: mat_inverseN
    float AA_T_inv_flat[36] = {0};
    if (!mat_inverse(AA_T_flat, AA_T_inv_flat, 6)) {
        // Retorna falso si la matriz es singular (geometría degenerada o inválida)
        return false;
    }

    // 4. Reconstruir la matriz invertida a un array bidimensional estándar de 6x6
    float AA_T_inv[6][6];
    for (uint8_t i = 0; i < 6; i++) {
        for (uint8_t j = 0; j < 6; j++) {
            AA_T_inv[i][j] = AA_T_inv_flat[i * 6 + j];
        }
    }

    // 5. Calcular la Pseudoinversa final: A_pinv = A_T * AA_T_inv (Tamaño 12x6)
    for (uint8_t i = 0; i < 12; i++) {
        for (uint8_t j = 0; j < 6; j++) {
            float sum = 0.0f;
            for (uint8_t k = 0; k < 6; k++) {
                sum += A_T[i][k] * AA_T_inv[k][j];
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

    const float km_value = 0.15f;

    // Motor 0
    rotor_x[0] =  0.0f;    rotor_y[0] =  0.275f;   rotor_km[0] = -km_value; // Giro Horario (CW)
    // Motor 1
    rotor_x[1] =  0.0f;    rotor_y[1] = -0.275f;   rotor_km[1] =  km_value;  // Giro Antihorario (CCW)
    // Motor 2
    rotor_x[2] =  0.238f;  rotor_y[2] = -0.1375f;  rotor_km[2] = -km_value; // Giro Horario (CW)
    // Motor 3
    rotor_x[3] = -0.238f;  rotor_y[3] =  0.1375f;  rotor_km[3] =  km_value;  // Giro Antihorario (CCW)
    // Motor 4
    rotor_x[4] =  0.238f;  rotor_y[4] =  0.1375f;  rotor_km[4] =  km_value;  // Giro Antihorario (CCW)
    // Motor 5
    rotor_x[5] = -0.238f;  rotor_y[5] = -0.1375f;  rotor_km[5] = -km_value; // Giro Horario (CW)




    set_initialised_ok(expected_num_motors == num_motors);
    compute_allocator();

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