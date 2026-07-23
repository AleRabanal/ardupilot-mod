-- mixer_6dof.lua
-- Script para cargar matriz de motores 6DOF en ArduCopter
-- Debe usarse junto con 6DoF_roll_pitch.lua

local K_m = 0.05 -- Relación torque/empuje

local ROLL_OFFSET_INICIAL  = 0.0  -- <<--- Aquí fijas tus 10 grados de Roll
local PITCH_OFFSET_INICIAL = 10.0

function calc_factors(Px, Py, Pz, Ax, Ay, Az, spin_dir)

    local forward  = Ax
    local right    = Ay
    local throttle = -Az

    local roll  = (Py * Az) - (Pz * Ay)
    local pitch = (Pz * Ax) - (Px * Az)

    local yaw_drag = -spin_dir * K_m
    local yaw   = (Px * Ay) - (Py * Ax) + yaw_drag

    return roll, pitch, yaw, throttle, forward, right
end

local m = {
    [0] = { 0.0,   0.275,  0.0,   0.321,  0.342, -0.883,  -1},
    [1] = { 0.0,  -0.275,  0.0,   0.321,  0.342, -0.883,   1},
    [2] = { 0.238,-0.1375, 0.0,   0.136, -0.449, -0.883,  -1},
    [3] = {-0.238, 0.1375, 0.0,   0.136, -0.449, -0.883,   1},
    [4] = { 0.238, 0.1375, 0.0,  -0.457,  0.107, -0.883,   1},
    [5] = {-0.238,-0.1375, 0.0,  -0.457,  0.107, -0.883,  -1},
}

for i = 0, 5 do
    local r, p, y, t, f, lat = calc_factors(
        m[i][1], m[i][2], m[i][3],
        m[i][4], m[i][5], m[i][6],
        m[i][7]
    )

    Motors_6DoF:add_motor(
        i,
        r,
        p,
        y,
        t,
        f,
        lat,
        false,
        i + 1
    )
end

if Motors_6DoF:init(6) then
    attitude_control:set_forward_enable(true)
    attitude_control:set_lateral_enable(true)
    
    -- ====================================================
    -- LÍNEA AGREGADA: Permite que el control de actitud 
    -- acepte orientaciones del vector de empuje inclinadas.
    -- ====================================================
    if attitude_control.set_vector_flight_mode then
        attitude_control:set_vector_flight_mode(true)
    elseif attitude_control.set_attenuation_enable then
        attitude_control:set_attenuation_enable(true)
    end
    -- ====================================================

    motors:set_frame_string("6DoF Hex scripting")
    gcs:send_text(6, string.format("Mixer 6DOF cargado (Roll Offset: %.1f deg)", ROLL_OFFSET_INICIAL))
else
    gcs:send_text(3, "Error: No se pudo inicializar el Mixer 6DOF")
end


function cargar_parametros_simar()
    -- Control de velocidad (Rate PIDs) de Roll y Pitch
    param:set_and_save('ATC_RAT_RLL_P', 0.07)
    param:set_and_save('ATC_RAT_RLL_I', 0.07)
    param:set_and_save('ATC_RAT_RLL_D', 0.0)
    
    param:set_and_save('ATC_RAT_PIT_P', 0.07)
    param:set_and_save('ATC_RAT_PIT_I', 0.07)
    param:set_and_save('ATC_RAT_PIT_D', 0.0)
    
    -- Ajuste drástico de Yaw (Heredado de tu MC_YAWRATE_P 0.05 de PX4)
    param:set_and_save('ATC_RAT_YAW_P', 0.16)
    param:set_and_save('ATC_RAT_YAW_I', 0.05)
    param:set_and_save('ATC_RAT_YAW_D', 0.1)
    param:set_and_save('ATC_ANG_YAW_P', 3.0)

    param:set_and_save('PSC_NE_POS_P', 5.0)
    --param:set_and_save('PSC_NE_POS_P', 2.0)
    param:set_and_save('PSC_NE_VEL_D', 1.2)
    --param:set_and_save('PSC_NE_VEL_D', 0.3)

    param:set_and_save('PSC_NE_VEL_P', 6.5)
    --param:set_and_save('PSC_NE_VEL_P', 2.5)
    param:set_and_save('PSC_NE_VEL_IMAX', 8.0)
  
    param:set_and_save('LOIT_BRK_DELAY', 0.1)
    param:set_and_save('LOIT_SPEED_MS', 5.0)
    
    -- Filtros estabilizadores para Gazebo
    param:set_and_save('INS_GYRO_FILTER', 15)

   -- Valores DEFAULT EKF3 ArduCopter 4.5.x

    param:set_and_save('EK3_DRAG_BCOEF_X', 0)
    param:set_and_save('EK3_DRAG_BCOEF_Y', 0)
    param:set_and_save('EK3_DRAG_MCOEF', 0)

    param:set_and_save('EK3_ACC_P_NSE', 0.35)
    param:set_and_save('EK3_GYRO_P_NSE', 0.015)

    param:set_and_save('EK3_POSNE_M_NSE', 0.5)
    param:set_and_save('EK3_VELNE_M_NSE', 0.5)

    param:set_and_save('EK3_GLITCH_RAD', 25)

    param:set_and_save('AHRS_GPS_USE', 1)
    param:set_and_save('EK3_GPS_CHECK', 31)
    param:set_and_save('COMPASS_USE', 0)
    param:set_and_save('COMPASS_USE2', 0)
    param:set_and_save('COMPASS_USE3', 0)   
    
    gcs:send_text(6, "6DoF SIMAR: ¡PID y parámetros inyectados con éxito!")
    --attitude_control:set_forward_enable(true)
    --attitude_control:set_lateral_enable(true)
end

-- Ejecutar la carga de parámetros tras un pequeño retraso de seguridad al arrancar
local ejecutado = false
local LOITER = 5
local STABILIZE = 0
local LOITER = 5  -- NO confíes en esto a largo plazo, pero ok temporal

function update()

    if not ejecutado then
        cargar_parametros_simar()
        ejecutado = true
    end
    
    local mode = vehicle:get_mode()

    local is_loiter = (mode == LOITER)

    attitude_control:set_forward_enable(is_loiter)
    attitude_control:set_lateral_enable(is_loiter)

    if attitude_control.set_vector_flight_mode then
        attitude_control:set_vector_flight_mode(is_loiter)
    end

    if is_loiter then
        attitude_control:set_offset_roll_pitch(ROLL_OFFSET_INICIAL, PITCH_OFFSET_INICIAL)
    else
        attitude_control:set_offset_roll_pitch(0,0)
    end 

    return update, 20
end

return update()
