#include <AP_Scripting/AP_Scripting_config.h>

#if AP_SCRIPTING_ENABLED

#include "AC_AttitudeControl_Multi_6DoF.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>

// 6DoF control is extracted from the existing copter code by treating desired angles as thrust angles rather than vehicle attitude.
// Vehicle attitude is then set separately, typically the vehicle would maintain 0 roll and pitch.
// rate commands result in the vehicle behaving as a ordinary copter.

// run lowest level body-frame rate controller and send outputs to the motors
void AC_AttitudeControl_Multi_6DoF::rate_controller_run() {
    // pass current offsets to motors and run baseclass controller
    float roll_deg = roll_offset_deg;
    float pitch_deg = pitch_offset_deg;
    AP::logger().Write(
    "OFFS",
    "TimeUS,RollOff,PitchOff,RollAHRS,PitchAHRS",
    "Qffff",
    AP_HAL::micros64(),
    roll_offset_deg,
    pitch_offset_deg,
    degrees(AP::ahrs().get_roll()),
    degrees(AP::ahrs().get_pitch()));
    
    if (lateral_enable) {
        roll_deg = degrees(AP::ahrs().get_roll());
    }
    if (forward_enable) {
        pitch_deg = degrees(AP::ahrs().get_pitch());
    }
    _motors.set_roll_pitch(roll_deg, pitch_deg);

    // LLAMADA DIRECTA A LA CLASE PADRE (Sin copiar variables raras)
    AC_AttitudeControl_Multi::rate_controller_run();
}

/*
    override all input to the attitude controller and convert desired angles into thrust angles and substitute
*/

// Command an euler roll and pitch angle and an euler yaw rate with angular velocity feedforward and smoothing

// STABILIZE / ALTHOLD (Modos manuales desactivan el vector de tierra)
void AC_AttitudeControl_Multi_6DoF::input_euler_angle_roll_pitch_euler_rate_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds) {
    set_forward_lateral(euler_pitch_angle_cd, euler_roll_angle_cd);
    AC_AttitudeControl_Multi::input_euler_angle_roll_pitch_euler_rate_yaw(euler_roll_angle_cd, euler_pitch_angle_cd, euler_yaw_rate_cds);
    
    if (AP_MotorsMatrix_6DoF_Scripting::get_singleton()) {
        AP_MotorsMatrix_6DoF_Scripting::get_singleton()->disable_earth_thrust_vector();
    }
}
// Command an euler roll, pitch and yaw angle with angular velocity feedforward and smoothing

// STABILIZE / ALTHOLD ABSOLUTO
void AC_AttitudeControl_Multi_6DoF::input_euler_angle_roll_pitch_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw) {
    set_forward_lateral(euler_pitch_angle_cd, euler_roll_angle_cd);
    AC_AttitudeControl_Multi::input_euler_angle_roll_pitch_yaw(euler_roll_angle_cd, euler_pitch_angle_cd, euler_yaw_angle_cd, slew_yaw);
    
    if (AP_MotorsMatrix_6DoF_Scripting::get_singleton()) {
        AP_MotorsMatrix_6DoF_Scripting::get_singleton()->disable_earth_thrust_vector();
    }
}

// Command a thrust vector and heading rate
// SOBREESCRIBIR PARA LOITER
void AC_AttitudeControl_Multi_6DoF::input_thrust_vector_rate_heading(const Vector3f& thrust_vector, float heading_rate_cds, bool slew_yaw)
{
    // Mantiene la actitud base y el control de guiñada manual
    AC_AttitudeControl_Multi::input_euler_angle_roll_pitch_euler_rate_yaw(
        roll_offset_deg * 100.0f, pitch_offset_deg * 100.0f, heading_rate_cds);

    // Envía el vector de fuerza 3D a tu matriz
    if (AP_MotorsMatrix_6DoF_Scripting::get_singleton()) {
        AP_MotorsMatrix_6DoF_Scripting::get_singleton()->set_earth_thrust_vector(thrust_vector);
    }
}
// Command a thrust vector, heading and heading rate
// SOBREESCRIBIR PARA AUTO / GUIDED
void AC_AttitudeControl_Multi_6DoF::input_thrust_vector_heading(const Vector3f& thrust_vector, float heading_angle_cd, float heading_rate_cds)
{
    // Mantiene actitud y rumbo fijo (usado en AUTO)
    AC_AttitudeControl_Multi::input_euler_angle_roll_pitch_yaw(
        roll_offset_deg * 100.0f, pitch_offset_deg * 100.0f, heading_angle_cd, true);

    if (AP_MotorsMatrix_6DoF_Scripting::get_singleton()) {
        AP_MotorsMatrix_6DoF_Scripting::get_singleton()->set_earth_thrust_vector(thrust_vector);
    }
}

void AC_AttitudeControl_Multi_6DoF::set_forward_lateral(float &euler_pitch_angle_cd, float &euler_roll_angle_cd)
{
   AP::logger().Write(
    "SFL0",
    "TimeUS,RollIn,PitchIn,LatEn,FwdEn",
    "QffBB",
    AP_HAL::micros64(),
    euler_roll_angle_cd * 0.01f,
    euler_pitch_angle_cd * 0.01f,
    lateral_enable,
    forward_enable);

    // pitch/forward
    if (forward_enable) {
        _motors.set_forward(-sinf(radians(euler_pitch_angle_cd * 0.01f)));
        euler_pitch_angle_cd = pitch_offset_deg * 100.0f;
    } else {
        _motors.set_forward(0.0f);
        euler_pitch_angle_cd += pitch_offset_deg * 100.0f;
    }
    euler_pitch_angle_cd = wrap_180_cd(euler_pitch_angle_cd);

    // roll/lateral
    if (lateral_enable) {
        _motors.set_lateral(sinf(radians(euler_roll_angle_cd * 0.01f)));
        euler_roll_angle_cd = roll_offset_deg * 100.0f;
    } else {
        _motors.set_lateral(0.0f);
        euler_roll_angle_cd += roll_offset_deg * 100.0f;
    }
    euler_roll_angle_cd = wrap_180_cd(euler_roll_angle_cd);
AP::logger().Write(
    "SFL1",
    "TimeUS,RollOut,PitchOut,Lat,Fwd",
    "Qffff",
    AP_HAL::micros64(),
    euler_roll_angle_cd * 0.01f,
    euler_pitch_angle_cd * 0.01f,
    _motors.get_lateral(),
    _motors.get_forward());

}

/*
    all other input functions should zero thrust vectoring
*/

// Command euler yaw rate and pitch angle with roll angle specified in body frame
// (used only by tailsitter quadplanes)
void AC_AttitudeControl_Multi_6DoF::input_euler_rate_yaw_euler_angle_pitch_bf_roll(bool plane_controls, float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds) {
    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    AC_AttitudeControl_Multi::input_euler_rate_yaw_euler_angle_pitch_bf_roll(plane_controls, euler_roll_angle_cd, euler_pitch_angle_cd, euler_yaw_rate_cds);
}

// Command an euler roll, pitch, and yaw rate with angular velocity feedforward and smoothing
void AC_AttitudeControl_Multi_6DoF::input_euler_rate_roll_pitch_yaw(float euler_roll_rate_cds, float euler_pitch_rate_cds, float euler_yaw_rate_cds) {
    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    AC_AttitudeControl_Multi::input_euler_rate_roll_pitch_yaw(euler_roll_rate_cds, euler_pitch_rate_cds, euler_yaw_rate_cds);
}

// Command an angular velocity with angular velocity feedforward and smoothing
void AC_AttitudeControl_Multi_6DoF::input_rate_bf_roll_pitch_yaw(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds) {
    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    AC_AttitudeControl_Multi::input_rate_bf_roll_pitch_yaw(roll_rate_bf_cds, pitch_rate_bf_cds, yaw_rate_bf_cds);
}

// Command an angular velocity with angular velocity feedforward and smoothing
void AC_AttitudeControl_Multi_6DoF::input_rate_bf_roll_pitch_yaw_2(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds) {
    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    AC_AttitudeControl_Multi::input_rate_bf_roll_pitch_yaw_2(roll_rate_bf_cds, pitch_rate_bf_cds, yaw_rate_bf_cds);
}

// Command an angular velocity with angular velocity smoothing using rate loops only with integrated rate error stabilization
void AC_AttitudeControl_Multi_6DoF::input_rate_bf_roll_pitch_yaw_3(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds) {
    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    AC_AttitudeControl_Multi::input_rate_bf_roll_pitch_yaw_3(roll_rate_bf_cds, pitch_rate_bf_cds, yaw_rate_bf_cds);
}

// Command an angular step (i.e change) in body frame angle
void AC_AttitudeControl_Multi_6DoF::input_angle_step_bf_roll_pitch_yaw(float roll_angle_step_bf_cd, float pitch_angle_step_bf_cd, float yaw_angle_step_bf_cd) {
    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    AC_AttitudeControl_Multi::input_angle_step_bf_roll_pitch_yaw(roll_angle_step_bf_cd, pitch_angle_step_bf_cd, yaw_angle_step_bf_cd);
}



// Command a Quaternion attitude with feedforward and smoothing
// attitude_desired_quat: is updated on each time_step (_dt) by the integral of the angular velocity
// not used anywhere in current code, panic in SITL so this implementation is not overlooked

void AC_AttitudeControl_Multi_6DoF::input_quaternion(Quaternion& attitude_desired_quat, Vector3f ang_vel_target) {
    _motors.set_lateral(0.0f);
    _motors.set_forward(0.0f);

    // 1. Sincronizamos el objetivo de actitud interno con el deseado
    _attitude_target = attitude_desired_quat;
    _ang_vel_target = ang_vel_target;

    // 2. Obtener la actitud física actual del dron (Body to NED)
    Quaternion attitude_body;
    _ahrs.get_quat_body_to_ned(attitude_body);

    // 3. Calcular el error de cuaternión en el Frame del Cuerpo (Body Frame)
    // Q_error = Q_body^-1 * Q_target
    Quaternion error_quat = attitude_body.inverse() * _attitude_target;
    error_quat.normalize();

    // 4. Convertimos el cuaternión de error a un vector de rotación eje-ángulo (Axis-Angle)
    Vector3f attitude_error;
    error_quat.to_axis_angle(attitude_error);

    // 5. Control Proporcional (P) Puro en los tres ejes del cuerpo
    // Bypasseamos por completo la descomposición de ArduPilot y evitamos límites de 30/60 grados
    _ang_vel_body.x = _p_angle_roll.kP() * attitude_error.x;
    _ang_vel_body.y = _p_angle_pitch.kP() * attitude_error.y;
    _ang_vel_body.z = _p_angle_yaw.kP() * attitude_error.z;

    // 6. Añadir el feedforward de velocidad angular rotado al frame del cuerpo
    Quaternion rotation_target_to_body = attitude_body.inverse() * _attitude_target;
    Vector3f ang_vel_body_feedforward = rotation_target_to_body * _ang_vel_target;
    _ang_vel_body += ang_vel_body_feedforward;

    // Guardamos el error para que el estimador (EKF) gestione correctamente los resets
    _attitude_ang_error = error_quat;

    // NOTA: NO llamamos a la clase base AC_AttitudeControl_Multi::input_quaternion
    // para evitar que se ejecute la lógica restrictiva estándar.
}


AC_AttitudeControl_Multi_6DoF *AC_AttitudeControl_Multi_6DoF::_singleton = nullptr;

#endif // AP_SCRIPTING_ENABLED