#pragma once
#if AP_SCRIPTING_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include "AP_MotorsMatrix.h"
#include <AP_Math/AP_Math.h>

class AP_MotorsMatrix_6DoF_Scripting : public AP_MotorsMatrix {
public:

    /// Constructor
   AP_MotorsMatrix_6DoF_Scripting(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMatrix(speed_hz)
    {
        // Usamos nuestro propio puntero estático local para el Singleton
        if (_singleton != nullptr) {
            AP_HAL::panic("AP_MotorsMatrix 6DoF must be singleton");
        }
        _singleton = this;
    };

    // get singleton instance
    static AP_MotorsMatrix_6DoF_Scripting *get_singleton() {
        return _singleton;
    }

    // output_to_motors - sends minimum values out to the motors
    void output_to_motors() override;

    // sets the roll and pitch offset, this rotates the thrust vector in body frame
    // these are typically set such that the throttle thrust vector is earth frame up
    void set_roll_pitch(float roll_deg, float pitch_deg) override;

    // add_motor using raw roll, pitch, throttle and yaw factors, to be called from scripting
    void add_motor(int8_t motor_num, float roll_factor, float pitch_factor, float yaw_factor, float throttle_factor, float forward_factor, float right_factor, bool reversible, uint8_t testing_order, bool is_servo);

    // if the expected number of motors have been setup then set as initalized
    bool init(uint8_t expected_num_motors) override;

    void build_effectiveness_matrix();
        // La matriz de efectividad se construye dinámicamente en función de la geometría y configuración de los motores/servos
        // Esto permite soportar diferentes configuraciones de drones 6DoF sin necesidad de hardcodear cada una
    void compute_allocator();

    // Nuevas funciones para recibir el vector 3D real de la Tierra
    void set_earth_thrust_vector(const Vector3f& thrust_vector) {
        _earth_thrust_vector = thrust_vector;
        _use_earth_thrust = true;
    }
    void disable_earth_thrust_vector() {
        _use_earth_thrust = false;
    }

      // 1. EL ENUM EN PUBLIC: Define los tipos de drones disponibles
    enum class HardwareMapping {
        DIRECT = 0,         // Salida directa lineal a PWM (drones omnidireccionales estándar)
        TILTING_HEXA = 1,   // Tu lógica de atan2 y modulo actual
        // ... nuevos en el futuro
    };
    // 2. LA FUNCIÓN EN PUBLIC: Para que Lua pueda llamarla
    void set_hardware_mapping(uint8_t mapping_type) {
        _hw_mapping = (HardwareMapping)mapping_type;
    }

protected:
    // output - sends commands to the motors
    void output_armed_stabilizing() override;

    // nothing to do for setup, scripting will mark as initalized when done
    void setup_motors(motor_frame_class frame_class, motor_frame_type frame_type) override {};

    const char* _get_frame_string() const override { return "6DoF scripting"; }

    float _forward_factor[AP_MOTORS_MAX_NUM_MOTORS];      // each motors contribution to forward thrust
    float _right_factor[AP_MOTORS_MAX_NUM_MOTORS];        // each motors contribution to right thrust

    // true if motor is revesible, it can go from -Spin max to +Spin max, if false motor is can go from Spin min to Spin max
    bool _reversible[AP_MOTORS_MAX_NUM_MOTORS];

    // store last values to allow deadzone
    float _last_thrust_out[AP_MOTORS_MAX_NUM_MOTORS];

    // Current offset angles, radians
    float _roll_offset;
    float _pitch_offset;

    // Array histórico para algoritmo Unwrap de servos de inclinación (Tilt)
    float _last_servo_angle_rad[AP_MOTORS_MAX_NUM_MOTORS];

    bool _is_servo[AP_MOTORS_MAX_NUM_MOTORS]; // true si el motor es un servo de inclinación, false si es un motor de empuje axial

   // --- VARIABLES NUEVAS PARA MATRIZ DINÁMICA ---
    uint8_t _num_actuators = 0; // Guardará cuántos motores hemos añadido desde LUA

    // Usamos el máximo de ArduPilot para reservar memoria de forma segura, 
    // pero matemáticamente solo usaremos hasta '_num_actuators'.
    float _A[6][AP_MOTORS_MAX_NUM_MOTORS];
    float _A_pinv[AP_MOTORS_MAX_NUM_MOTORS][6];

    bool _allocator_initialized = false;

    // Actualizamos la firma de la función para que acepte el array máximo y un ancho dinámico
    bool calcular_pseudoinversa(const float A[6][AP_MOTORS_MAX_NUM_MOTORS], float A_pinv[AP_MOTORS_MAX_NUM_MOTORS][6], uint8_t num_actuators);

    



    float _angulo_acumulado[6] = {0.0f};
    

private:
  Vector3f _earth_thrust_vector;
    bool _use_earth_thrust = false;
    static AP_MotorsMatrix_6DoF_Scripting *_singleton;
    HardwareMapping _hw_mapping = HardwareMapping::DIRECT;


};

#endif // AP_SCRIPTING_ENABLED