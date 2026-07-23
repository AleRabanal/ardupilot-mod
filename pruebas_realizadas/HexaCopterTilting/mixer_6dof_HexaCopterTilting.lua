-- mixer_6dof_HexaCopterTilting.lua
-- Mixer 6DoF para hexacóptero tiltrotor
--
-- Layout de motores:
--   0..5   → motores físicos de empuje (vertical)
--   6..11  → servos de tilt físicos (reversible=true, set_angle)
--   12..17 → componentes laterales virtuales (solo cálculo, sin PWM)
--
-- En firmware (output_to_motors):
--   T     = sqrt(Fv[i]² + Fl[i+12]²)
--   theta = atan2(Fl[i+12], Fv[i])

local K_m = 0.05

------------------------------------------------------------
-- OFFSETS DE VUELO
------------------------------------------------------------

local ROLL_OFFSET_INICIAL  = 00.0
local PITCH_OFFSET_INICIAL = 30.0

------------------------------------------------------------
-- GEOMETRÍA
-- {x, y, spin}
-- spin: +1 = CCW, -1 = CW
------------------------------------------------------------

local motors = {
    [0] = {  0.0,     0.275,   -1 },
    [1] = {  0.0,    -0.275,    1 },
    [2] = {  0.238,  -0.1375,  -1 },
    [3] = { -0.238,   0.1375,   1 },
    [4] = {  0.238,   0.1375,   1 },
    [5] = { -0.238,  -0.1375,  -1 },
}

------------------------------------------------------------
-- MOTOR VERTICAL (empuje puro hacia arriba)
------------------------------------------------------------

function calc_vertical(x, y, spin)
    local roll     = -y
    local pitch    =  x
    local yaw      =  spin * K_m
    local throttle = 1.0
    local forward  = 0.0
    local lateral  = 0.0
    return roll, pitch, yaw, throttle, forward, lateral
end

------------------------------------------------------------
-- MOTOR LATERAL (empuje horizontal tangencial)
------------------------------------------------------------

function calc_lateral(x, y, spin)
    local r = math.sqrt(x*x + y*y)
    local cphi = x / r
    local sphi = y / r

    local forward = -sphi * 2.0
    local lateral =  cphi * 2.0

    local roll  =  K_m * sphi
    local pitch = -K_m * cphi
    local yaw   = x*cphi + y*sphi

    local throttle = 0.0
    return roll, pitch, yaw, throttle, forward, lateral
end

------------------------------------------------------------
-- REGISTRO DE MOTORES
------------------------------------------------------------

for i = 0, 5 do
    local x    = motors[i][1]
    local y    = motors[i][2]
    local spin = motors[i][3]

    -- Motor físico de empuje (0..5), reversible=false
    local r, p, yaw, t, f, l = calc_vertical(x, y, spin)
    Motors_6DoF:add_motor(i, r, p, yaw, t, f, l, false, 0)

    -- Componente lateral virtual (12..17), reversible=false, sin canal PWM
    local r2, p2, y2, t2, f2, l2 = calc_lateral(x, y, spin)
    Motors_6DoF:add_motor(i + 6, r2, p2, y2, t2, f2, l2, false, 0)
end


------------------------------------------------------------

if Motors_6DoF:init(12) then

    attitude_control:set_forward_enable(true)
    attitude_control:set_lateral_enable(true)

    if attitude_control.set_vector_flight_mode then
        attitude_control:set_vector_flight_mode(true)
    elseif attitude_control.set_attenuation_enable then
        attitude_control:set_attenuation_enable(true)
    end

    gcs:send_text(6, "Hexa Tilt 6DoF mixer cargado OK")

else
    gcs:send_text(3, "ERROR: No se pudo inicializar el mixer")
end

------------------------------------------------------------
-- PARÁMETROS
------------------------------------------------------------

function cargar_parametros()

    -- Motores de empuje → SERVO1..6 = Motor1..6 (33..38)
    param:set_and_save("SERVO1_FUNCTION",  33)
    param:set_and_save("SERVO2_FUNCTION",  34)
    param:set_and_save("SERVO3_FUNCTION",  35)
    param:set_and_save("SERVO4_FUNCTION",  36)
    param:set_and_save("SERVO5_FUNCTION",  37)
    param:set_and_save("SERVO6_FUNCTION",  38)

    -- Servos de tilt → SERVO7..12 = Motor7..12 
    param:set_and_save("SERVO7_FUNCTION",  39)
    param:set_and_save("SERVO8_FUNCTION",  40)
    param:set_and_save("SERVO9_FUNCTION",  82)
    param:set_and_save("SERVO10_FUNCTION", 83)
    param:set_and_save("SERVO11_FUNCTION", 84)
    param:set_and_save("SERVO12_FUNCTION", 85)

    --------------------------------------------------------
    -- RATE PID
    --------------------------------------------------------

    param:set_and_save('ATC_RAT_RLL_P', 0.07)
    param:set_and_save('ATC_RAT_RLL_I', 0.07)
    param:set_and_save('ATC_RAT_RLL_D', 0.0)

    param:set_and_save('ATC_RAT_PIT_P', 0.07)
    param:set_and_save('ATC_RAT_PIT_I', 0.07)
    param:set_and_save('ATC_RAT_PIT_D', 0.0)

    --------------------------------------------------------
    -- YAW
    --------------------------------------------------------

    param:set_and_save('ATC_RAT_YAW_P', 0.16)
    param:set_and_save('ATC_RAT_YAW_I', 0.05)
    param:set_and_save('ATC_RAT_YAW_D', 0.1)
    param:set_and_save('ATC_ANG_YAW_P', 3.0)

    --------------------------------------------------------
    -- POSICIÓN
    --------------------------------------------------------

    param:set_and_save('PSC_NE_POS_P',    5.0)
    param:set_and_save('PSC_NE_VEL_P',    6.5)
    param:set_and_save('PSC_NE_VEL_D',    1.2)
    param:set_and_save('PSC_NE_VEL_IMAX', 8.0)

    --------------------------------------------------------
    -- LOITER
    --------------------------------------------------------

    param:set_and_save('LOIT_BRK_DELAY', 0.1)
    param:set_and_save('LOIT_SPEED_MS',  5.0)

    --------------------------------------------------------
    -- FILTROS
    --------------------------------------------------------

    param:set_and_save('INS_GYRO_FILTER', 15)

    --------------------------------------------------------
    -- EKF
    --------------------------------------------------------

    param:set_and_save('EK3_DRAG_BCOEF_X', 0)
    param:set_and_save('EK3_DRAG_BCOEF_Y', 0)
    param:set_and_save('EK3_DRAG_MCOEF',   0)
    param:set_and_save('EK3_ACC_P_NSE',    0.35)
    param:set_and_save('EK3_GYRO_P_NSE',   0.015)
    param:set_and_save('EK3_POSNE_M_NSE',  0.5)
    param:set_and_save('EK3_VELNE_M_NSE',  0.5)
    param:set_and_save('EK3_GLITCH_RAD',   25)

    --------------------------------------------------------
    -- GPS / COMPASS
    --------------------------------------------------------

    param:set_and_save('AHRS_GPS_USE',  1)
    param:set_and_save('EK3_GPS_CHECK', 31)
    param:set_and_save('COMPASS_USE',   0)
    param:set_and_save('COMPASS_USE2',  0)
    param:set_and_save('COMPASS_USE3',  0)

    gcs:send_text(6, "Parámetros Hexa Tilt cargados")
end

------------------------------------------------------------
-- UPDATE LOOP
------------------------------------------------------------

local ejecutado = false
local LOITER    = 5

function update()

    if not ejecutado then
        cargar_parametros()
        ejecutado = true
    end

    local mode     = vehicle:get_mode()
    local is_loiter = (mode == LOITER)

    attitude_control:set_forward_enable(true)
    attitude_control:set_lateral_enable(true)

    if attitude_control.set_vector_flight_mode then
        attitude_control:set_vector_flight_mode(is_loiter)
    end

    if is_loiter then
        attitude_control:set_offset_roll_pitch(ROLL_OFFSET_INICIAL, PITCH_OFFSET_INICIAL)
    else
        attitude_control:set_offset_roll_pitch(0, 0)
    end

    return update, 20
end

return update()