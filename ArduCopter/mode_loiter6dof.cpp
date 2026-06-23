#include "Copter.h"
#include <AP_Scripting/AP_Scripting_config.h>

#if AP_SCRIPTING_ENABLED
#include <AC_AttitudeControl/AC_AttitudeControl_Multi_6DoF.h>
#include <AP_Motors/AP_MotorsMatrix_6DoF_Scripting.h>
#endif

#include "Copter.h"

bool ModeLoiter6DoF::init(bool ignore_checks)
{
    // Al entrar en el modo, nos aseguramos de que el dron no quiera salir disparado a otro punto
    loiter_nav->clear_pilot_desired_acceleration();
    loiter_nav->init_target();

    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }
    
    // Permitimos que el controlador de posición ignore los límites de 45 grados
    pos_control->set_lean_angle_max_cd(18000.0f);

    return true;
}
#include "Copter.h"

void ModeLoiter6DoF::run()
{
    // 1. DESBLOQUEO: Permitimos al controlador trabajar en cualquier ángulo
    pos_control->set_lean_angle_max_cd(18000.0f);

    // 2. STICKS: Procesamos el mando para mover el punto de destino (pero no inclinamos)
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());
    loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
    loiter_nav->update(); // Actualiza el PID de posición XY (GPS)

    // 3. ALTURA: Actualiza el PID de altura
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
    pos_control->update_z_controller();

    // 4. ACTITUD 360º: Usamos Cuaterniones para evitar el Gimbal Lock (bloqueo a 90º)
    float roll_off = 0.0f, pitch_off = 0.0f;
#if AP_SCRIPTING_ENABLED
    auto *att_6dof = AC_AttitudeControl_Multi_6DoF::get_singleton();
    if (att_6dof) {
        roll_off = att_6dof->get_roll_offset_deg();
        pitch_off = att_6dof->get_pitch_offset_deg();
    }
#endif
    Quaternion target_quat;
    target_quat.from_euler(radians(roll_off), radians(pitch_off), ahrs.get_yaw());
    Vector3f dummy_ang_vel(0, 0, get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz()));
    attitude_control->input_quaternion(target_quat, dummy_ang_vel);

    // ====================================================================
    // 5. CONSTRUCCIÓN DEL VECTOR DE FUERZA FÍSICA (LA CLAVE)
    // ====================================================================
    
    // A) Obtenemos la ACELERACIÓN de corrección de los PIDs (en marco NEU, cm/s^2)
    // Esto es lo que "frena" al dron si se mueve y lo mantiene en el punto.
    Vector3f accel_correction_neu = pos_control->get_accel_target_cmss();

    // B) Sumamos la GRAVEDAD para compensar el peso (981 cm/s^2 hacia ARRIBA en NEU)
    // Esto es lo que permite el Hover automático.
    accel_correction_neu.z += 980.665f;

    // C) Convertimos de Aceleración Física a Escala de Motores (0..1)
    // Usamos el throttle_hover real que el dron aprendió (ej. 0.3)
    float hover_thr = motors->get_throttle_hover();
    Vector3f combined_thrust_neu = accel_correction_neu * (hover_thr / 980.665f);

    // D) Convertimos el vector de NEU (Arriba+) a NED (Abajo+) para el Mixer
    Vector3f thrust_vector_ned(combined_thrust_neu.x, combined_thrust_neu.y, -combined_thrust_neu.z);

    if (AP_MotorsMatrix_6DoF_Scripting::get_singleton()) {
        AP_MotorsMatrix_6DoF_Scripting::get_singleton()->set_earth_thrust_vector(thrust_vector_ned);
    }
}