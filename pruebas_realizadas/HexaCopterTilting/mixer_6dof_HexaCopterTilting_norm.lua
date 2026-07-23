-- mixer_6dof_HexaCopterTilting.lua
-- Mixer 6DoF para hexacóptero tiltrotor
--
-- Layout de motores:
--   0..5   → motores físicos de empuje (vertical)
--   6..11  → servos de tilt físicos
--
-- En Stabilize: set_forward_enable(false) → _forward_in_scaled=0 → servos a 1500 (vertical)
--               El dron vuela como un hexacóptero normal.
-- En Loiter:    set_forward_enable(true)  → servos se inclinan para traslación horizontal.
--
-- Factores normalizados a [-1, 1] dividiendo por el brazo máximo (0.275m).
-- K_m ajustado a 0.15 para dar autoridad de yaw suficiente (ratio yaw/roll = 0.545).

------------------------------------------------------------
-- CONSTANTES
------------------------------------------------------------

-- K_m: relación torque-reactivo / empuje del motor.
-- Con 0.05 el ratio yaw/roll era 0.18 → el PID de yaw no tenía autoridad.
-- Con 0.15 el ratio es 0.545, comparable a un hex estándar de ArduPilot.
-- Si en vuelo el yaw oscila, bajar a 0.12. Si sigue sin mantener, subir a 0.18.
local K_m = 0.15

-- Brazo máximo para normalizar factores a [-1, 1]
local ARM = 0.275

------------------------------------------------------------
-- OFFSETS DE VUELO
------------------------------------------------------------

local ROLL_OFFSET_INICIAL  = 0.0
local PITCH_OFFSET_INICIAL = 0.0

------------------------------------------------------------
-- GEOMETRÍA
-- {x, y, spin}
-- x: positivo = adelante (nose)
-- y: positivo = izquierda
-- spin: +1 = CCW, -1 = CW (visto desde arriba)
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
-- Factores normalizados: dividir posición por ARM, yaw por ARM también
-- para que todos los factores queden en [-1, 1] y el mixer los trate igual.
------------------------------------------------------------

function calc_vertical(x, y, spin)
    local roll     = -y   / ARM    -- [-1, 1]
    local pitch    =  x   / ARM    -- [-1, 1]
    local yaw      =  spin * K_m / ARM  -- con K_m=0.15 y ARM=0.275 → ±0.545
    local throttle = 1.0
    local forward  = 0.0
    local lateral  = 0.0
    return roll, pitch, yaw, throttle, forward, lateral
end

------------------------------------------------------------
-- MOTOR LATERAL (empuje horizontal tangencial para tilt)
-- Solo forward/lateral importan aquí; roll/pitch/yaw = 0
-- porque estos motores virtuales no generan sustentación ni par.
-- El par real del tilt se gestiona en output_to_motors() via atan2.
------------------------------------------------------------

function calc_lateral(x, y, spin)
    local r = math.sqrt(x*x + y*y)
    local cphi = x / r   -- cos del ángulo azimutal
    local sphi = y / r   -- sin del ángulo azimutal

    -- Empuje tangencial: perpendicular al brazo, en el plano horizontal
    local forward = -sphi
    local lateral =  cphi

    -- Roll/pitch/yaw = 0: el componente lateral virtual no participa
    -- en el mixer de actitud, solo en el cálculo del ángulo de tilt.
    return 0.0, 0.0, 0.0, 0.0, forward, lateral
end

------------------------------------------------------------
-- REGISTRO DE MOTORES
------------------------------------------------------------

for i = 0, 5 do
    local x    = motors[i][1]
    local y    = motors[i][2]
    local spin = motors[i][3]

    -- Motor físico de empuje (0..5)
    local r, p, yaw, t, f, l = calc_vertical(x, y, spin)
    Motors_6DoF:add_motor(i, r, p, yaw, t, f, l, false, 0)

    -- Componente lateral virtual (6..11): solo forward/lateral, sin PWM propio
    local r2, p2, y2, t2, f2, l2 = calc_lateral(x, y, spin)
    Motors_6DoF:add_motor(i + 6, r2, p2, y2, t2, f2, l2, false, 0)
end

------------------------------------------------------------
-- INICIALIZACIÓN
-- 12 entradas en el mixer: 0..5 (motores) + 6..11 (tilt virtual)
------------------------------------------------------------

if Motors_6DoF:init(12) then

    -- Desactivar traslación al inicio; el update loop lo gestiona por modo
    attitude_control:set_forward_enable(false)
    attitude_control:set_lateral_enable(false)

    gcs:send_text(6, "Hexa Tilt 6DoF mixer cargado OK")

else
    gcs:send_text(3, "ERROR: No se pudo inicializar el mixer")
end

------------------------------------------------------------
-- PARÁMETROS
------------------------------------------------------------

function cargar_parametros()

    -- Motores de empuje → SERVO1..6 = Motor1..6 (funciones 33..38)
    param:set_and_save("SERVO1_FUNCTION",  33)
    param:set_and_save("SERVO2_FUNCTION",  34)
    param:set_and_save("SERVO3_FUNCTION",  35)
    param:set_and_save("SERVO4_FUNCTION",  36)
    param:set_and_save("SERVO5_FUNCTION",  37)
    param:set_and_save("SERVO6_FUNCTION",  38)

    -- Servos de tilt → SERVO7..12 = Motor7..12 (funciones 39..44)
    param:set_and_save("SERVO7_FUNCTION",  39)
    param:set_and_save("SERVO8_FUNCTION",  40)
    param:set_and_save("SERVO9_FUNCTION",  82)
    param:set_and_save("SERVO10_FUNCTION", 83)
    param:set_and_save("SERVO11_FUNCTION", 84)
    param:set_and_save("SERVO12_FUNCTION", 85)

    --------------------------------------------------------
    -- RATE PID — valores conservadores para primer vuelo
    -- Subir P/I gradualmente si responde lento
    --------------------------------------------------------

    param:set_and_save('ATC_RAT_RLL_P', 0.07)
    param:set_and_save('ATC_RAT_RLL_I', 0.07)
    param:set_and_save('ATC_RAT_RLL_D', 0.0)

    param:set_and_save('ATC_RAT_PIT_P', 0.07)
    param:set_and_save('ATC_RAT_PIT_I', 0.07)
    param:set_and_save('ATC_RAT_PIT_D', 0.0)

    --------------------------------------------------------
    -- YAW — con K_m=0.15 el mixer tiene más autoridad;
    -- bajar P a 0.12 si oscila, subir a 0.20 si sigue sin mantener
    --------------------------------------------------------

    param:set_and_save('ATC_RAT_YAW_P', 0.16)
    param:set_and_save('ATC_RAT_YAW_I', 0.05)
    param:set_and_save('ATC_RAT_YAW_D', 0.1)
    param:set_and_save('ATC_ANG_YAW_P', 3.0)

    --------------------------------------------------------
    -- POSICIÓN
    --------------------------------------------------------

    param:set_and_save('PSC_NE_POS_P',    3.0)
    param:set_and_save('PSC_NE_VEL_P',    4.5)
    param:set_and_save('PSC_NE_VEL_D',    1.0)
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

    gcs:send_text(6, "Parametros Hexa Tilt cargados")
end

------------------------------------------------------------
-- UPDATE LOOP
------------------------------------------------------------

local ejecutado  = false
local LOITER     = 5
local POSHOLD    = 16

function update()

    if not ejecutado then
        cargar_parametros()
        ejecutado = true
    end

    local mode = vehicle:get_mode()
    local traslacion = (mode == LOITER or mode == POSHOLD)

    -- En Stabilize/AltHold: forward=false → _forward_in_scaled=0 en C++
    -- → servos permanecen a 1500us (vertical), hex vuela normal.
    -- En Loiter/PosHold: forward=true → servos se inclinan para traslación.
    attitude_control:set_forward_enable(traslacion)
    attitude_control:set_lateral_enable(traslacion)

    if attitude_control.set_vector_flight_mode then
        attitude_control:set_vector_flight_mode(traslacion)
    end

    if traslacion then
        attitude_control:set_offset_roll_pitch(ROLL_OFFSET_INICIAL, PITCH_OFFSET_INICIAL)
    else
        attitude_control:set_offset_roll_pitch(0, 0)
    end

    return update, 20
end

return update()
