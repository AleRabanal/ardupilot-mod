#include "Copter.h"

bool ModeGuided6DoF::init(bool ignore_checks)
{
    // Inicializamos el controlador de posición desde cero
    pos_control->init_z_controller();
    pos_control->init_xy_controller();
    
    // Desbloqueamos el ángulo para que tus motores 6DoF tengan potencia
    pos_control->set_lean_angle_max_cd(2000.0f);
    return true;
}

#include "Copter.h"
#include "Copter.h"

void ModeGuided6DoF::run()
{
    // 1. Si no está armado, motores en reposo
    if (!motors->armed()) {
        make_safe_ground_handling();
        return;
    }

    // 2. RECUPERAR EL SETPOINT DE ROS
    // MAVROS escribe en el objeto 'mode_guided'. Le robamos el target.
    Vector3p target_pos_cm = copter.mode_guided.get_target_pos();
    
    // Inyectamos el target a nuestro controlador de posición actual
    pos_control->input_pos_xyz(target_pos_cm, 0.0f, 1000.0f);

    // 3. LÓGICA DE DESPEGUE AUTOMÁTICO
    float current_alt_cm = inertial_nav.get_position_z_up_cm();

    if (copter.ap.land_complete && target_pos_cm.z > current_alt_cm + 10.0f) {
        set_land_complete(false);
        copter.set_auto_armed(true);
        pos_control->init_z_controller();
        loiter_nav->init_target(); 
    }

    if (copter.ap.land_complete) {
        make_safe_ground_handling();
        return;
    }

    // 4. NAVEGACIÓN ACTIVA
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // Actualizamos PIDs de posición
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // 5. ACTITUD 6DoF
    float roll_off = 0.0f, pitch_off = 0.0f;
#if AP_SCRIPTING_ENABLED
    auto *att_6dof = AC_AttitudeControl_Multi_6DoF::get_singleton();
    if (att_6dof) {
        roll_off = att_6dof->get_roll_offset_deg();
        pitch_off = att_6dof->get_pitch_offset_deg();
    }
#endif

    float target_yaw_cd = auto_yaw.get_heading().yaw_angle_cd;
    attitude_control->input_euler_angle_roll_pitch_yaw(roll_off * 100.0f, pitch_off * 100.0f, target_yaw_cd, true);

    // 6. VECTOR DE EMPUJE (CORRECCIÓN DE SIGNOS)
    // Obtenemos la aceleración pura que pide el controlador (cm/s^2)
    Vector3f accel_target = pos_control->get_accel_target_cmss();
    
    // Sumamos la gravedad para compensar el peso del dron (980 cm/s^2)
    accel_target.z += 980.665f; 

    float hover_thr = motors->get_throttle_hover();
    
    // Convertimos la aceleración a una magnitud de empuje (0.0 a 1.0)
    Vector3f combined_thrust_neu = accel_target * (hover_thr / 980.665f);
    
    // TRASPASO A NED: 
    // Si tus motores apuntaban hacia abajo con '-combined_thrust_neu.z', 
    // prueba a quitar el signo menos. 
    // Sin embargo, según el estándar ArduPilot, debería ser negativo.
    // Vamos a usar la lógica que te funcionó en Loiter:
    Vector3f thrust_vector_ned(combined_thrust_neu.x, combined_thrust_neu.y, -combined_thrust_neu.z);

    if (AP_MotorsMatrix_6DoF_Scripting::get_singleton()) {
        AP_MotorsMatrix_6DoF_Scripting::get_singleton()->set_earth_thrust_vector(thrust_vector_ned);
    }

    attitude_control->set_throttle_out(combined_thrust_neu.z, true, 0.0f);
}