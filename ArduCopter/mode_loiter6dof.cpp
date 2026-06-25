#include "Copter.h"
#include <AP_Scripting/AP_Scripting_config.h>

#if AP_SCRIPTING_ENABLED
#include <AC_AttitudeControl/AC_AttitudeControl_Multi_6DoF.h>
#include <AP_Motors/AP_MotorsMatrix_6DoF_Scripting.h>
#endif

bool ModeLoiter6DoF::init(bool ignore_checks)
{
    loiter_nav->clear_pilot_desired_acceleration();
    loiter_nav->init_target();

    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }
    
    pos_control->set_lean_angle_max_cd(2000.0f);

    return true;
}

#include "Copter.h"

void ModeLoiter6DoF::run()
{
    // 1. DESBLOQUEO: Permitimos al controlador de posición trabajar con el vector de empuje inclinado
    pos_control->set_lean_angle_max_cd(2000.0f);

    // 2. STICKS: Procesamos el mando para mover el punto de destino
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());
    loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
    loiter_nav->update(); // Actualiza el PID de posición XY (GPS)

    // 3. ALTURA: Actualiza el PID de altura
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
    pos_control->update_z_controller();

    // 4. ACTITUD 360º CONSTANTE (Heading Hold Puro)
    static float target_yaw_rad = 0.0f;
    static bool yaw_initialized = false;
    if (!yaw_initialized) {
        target_yaw_rad = ahrs.get_yaw();
        yaw_initialized = true;
    }

    // Integramos el Yaw del piloto de forma suave y precisa
    float pilot_yaw_rate_cds = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
    float yaw_rate_rads = radians(pilot_yaw_rate_cds * 0.01f);
    float dt = attitude_control->get_dt();
    
    target_yaw_rad += yaw_rate_rads * dt;
    target_yaw_rad = wrap_PI(target_yaw_rad);

    // Leemos los ángulos deseados que pide tu script LUA
    float roll_off = 0.0f, pitch_off = 0.0f;
#if AP_SCRIPTING_ENABLED
    auto *att_6dof = AC_AttitudeControl_Multi_6DoF::get_singleton();
    if (att_6dof) {
        roll_off = att_6dof->get_roll_offset_deg();
        pitch_off = att_6dof->get_pitch_offset_deg();
    }
#endif

    // Construimos el target_quat con el Yaw integrado. 
    // Al usar nuestro nuevo controlador de cuaterniones, no habrá ningún salto brusco
    // al pasar por los 90º de Pitch o Roll.
    Quaternion target_quat;
    target_quat.from_euler(radians(roll_off), radians(pitch_off), target_yaw_rad);

    Vector3f dummy_ang_vel(0.0f, 0.0f, yaw_rate_rads);
    attitude_control->input_quaternion(target_quat, dummy_ang_vel);

    // ====================================================================
    // 5. CONSTRUCCIÓN DEL VECTOR DE FUERZA FÍSICA
    // ====================================================================
    Vector3f accel_correction_neu = pos_control->get_accel_target_cmss();
    accel_correction_neu.z += 980.665f;

    float hover_thr = motors->get_throttle_hover();
    Vector3f combined_thrust_neu = accel_correction_neu * (hover_thr / 980.665f);

    Vector3f thrust_vector_ned(combined_thrust_neu.x, combined_thrust_neu.y, -combined_thrust_neu.z);

    if (AP_MotorsMatrix_6DoF_Scripting::get_singleton()) {
        AP_MotorsMatrix_6DoF_Scripting::get_singleton()->set_earth_thrust_vector(thrust_vector_ned);
    }
}