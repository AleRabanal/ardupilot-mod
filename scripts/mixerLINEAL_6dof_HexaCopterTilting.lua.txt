-- mixer_6dof_HexaCopterTilting.lua
-- Mixer 6DoF para hexacóptero tiltrotor


------------------------------------------------------------
-- OFFSETS DE VUELO
------------------------------------------------------------

local ROLL_OFFSET_INICIAL  = 00.0
local PITCH_OFFSET_INICIAL = 30.0

local VELOCIDAD_ROLL_DEG_S = 5.0  -- Velocidad de giro en grados por segundo (a 45°/s tardará 8 segundos en dar la vuelta completa)
local PITCH_OFFSET_INICIAL  = 0.0

local actual_roll_offset   = 0.0   -- Variable interna para acumular el ángulo
local LOOP_PERIOD_S        = 0.02  -- 20ms expresados en segundos

--local roll_factor = 0.3
--local pitch_factor = 0.3
--local yaw_factor = 0.3
--local thrust_factor = 0.2
--local forward_factor = 0.3
--local right_factor = 0.3

--local roll_factor = 7.0
--local pitch_factor = 7.0
--local yaw_factor = 10.0
--local thrust_factor = 4.0
--local forward_factor = 1.0
--local right_factor = 1.0

-- ==============================================================================
-- CONFIGURACIÓN DE LA GEOMETRÍA Y ESCALAS
-- ==========================--====================================================
local K_m = 0.15

--local roll_scale     = 8.36
--local pitch_scale    = 5.202
--local yaw_scale      = 8.636


--local fx_scale       = 0.8  -- Escala de Forward
--local fy_scale       = 0.8  -- Escala de Right
--local fz_scale       = 0.7  -- Escala de Thrust


-- ESCALAS EQUILIBRADAS (Basadas en física real)
-- ESCALAS RECALCULADAS (Base fz_scale = 1.0)
--local roll_scale     = 11.943
--local pitch_scale    = 7.431
--local yaw_scale      = 12.337

local roll_scale     = 5.0
local pitch_scale    = 5.0
local yaw_scale      = 6.0


local fx_scale       = 1.143  -- Escala de Forward
local fy_scale       = 1.143  -- Escala de Right
local fz_scale       = 1.0    -- Escala de Thrust (Base)
-- Definimos las posiciones (X, Y) y el sentido de giro (KM) de los 6 rotores
local rotores = {
    [0] = { x =  0.0,    y =  0.275,   km = -K_m }, -- Motor 0
    [1] = { x =  0.0,    y = -0.275,   km =  K_m }, -- Motor 1
    [2] = { x =  0.238,  y = -0.1375,  km = -K_m }, -- Motor 2
    [3] = { x = -0.238,  y =  0.1375,  km =  K_m }, -- Motor 3
    [4] = { x =  0.238,  y =  0.1375,  km =  K_m }, -- Motor 4
    [5] = { x = -0.238,  y = -0.1375,  km = -K_m }  -- Motor 5
}

-- ==============================================================================
-- GENERACIÓN DINÁMICA DE LOS 12 MOTORES (Empuje + Servos)
-- ==============================================================================
for i = 0, 5 do
    local r = rotores[i]
    
    -- Cálculos geométricos (l, cos(phi), sin(phi))
    local l = math.sqrt(r.x * r.x + r.y * r.y)
    local cphi = r.x / l
    local sphi = r.y / l

    -- 1. MOTORES VIRTUALES VERTICALES (RPM -> Controlan Altura, Roll, Pitch, Yaw)
    local v_roll    = -r.y * roll_scale
    local v_pitch   =  r.x * pitch_scale
    local v_yaw     =  r.km * yaw_scale
    local v_forward =  0.0
    local v_right   =  0.0
    local v_thrust  =  1.0 * fz_scale

    -- Añadir Motor Físico de Empuje (Índices 0 al 5)
    -- Argumentos: Motor_Num, Roll_Fac, Pitch_Fac, Yaw_Fac, Throttle_Fac, Forward_Fac, Right_Fac, Reversible, Test_Order
    Motors_6DoF:add_motor(i, v_roll, v_pitch, v_yaw, v_thrust, v_forward, v_right, false, i + 1,false)


    -- 2. MOTORES VIRTUALES HORIZONTALES (Servos -> Controlan Forward, Right, Yaw y efectos cruzados)
    local h_roll    =  r.km * sphi * roll_scale
    local h_pitch   = -r.km * cphi * pitch_scale
    local h_yaw     = (r.x * cphi + r.y * sphi) * yaw_scale
    local h_forward = -sphi * fx_scale
    local h_right   =  cphi * fy_scale
    local h_thrust  =  0.0

    -- Añadir Motor Físico del Servo (Índices 6 al 11)
    Motors_6DoF:add_motor(i + 6, h_roll, h_pitch, h_yaw, h_thrust, h_forward, h_right, false, i + 7,true)
end
------------------------------------------------------------

if Motors_6DoF:init(12) then
    Motors_6DoF:set_hardware_mapping(1) --tipo de vehiculo 6dof que queremos usar (0=directo, 1=tilting hexa, 2=tilting quad, 3=tilting quad + 2 servos)
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

    param:set_and_save("RC7_OPTION",0)

    --------------------------------------------------------
    -- RATE PID
    --------------------------------------------------------


    --------------------------------------------------------
    -- DESACTIVAR ANGLE BOOST (Evita la saturación a 80º)
    --------------------------------------------------------
    param:set_and_save("ATC_ANGLE_BOOST", 1)



    param:set_and_save('ATC_RAT_RLL_P', 0.13)
    param:set_and_save('ATC_RAT_RLL_I', 0.13)
    param:set_and_save('ATC_RAT_RLL_D', 0.0086)

    param:set_and_save('ATC_RAT_PIT_P', 0.13)
    param:set_and_save('ATC_RAT_PIT_I', 0.13)
    param:set_and_save('ATC_RAT_PIT_D', 0.0086)

    --param:set_and_save('ATC_RAT_RLL_P', 0.425)
    --param:set_and_save('ATC_RAT_RLL_I', 0.425)
    --param:set_and_save('ATC_RAT_RLL_D', 0.0086)

    --param:set_and_save('ATC_RAT_PIT_P', 0.425)
    --param:set_and_save('ATC_RAT_PIT_I', 0.425)
    --param:set_and_save('ATC_RAT_PIT_D', 0.0086)

    param:set_and_save('ATC_ANG_RLL_P', 4.5)
    param:set_and_save('ATC_ANG_PIT_P', 4.5)

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

   -- param:set_and_save('PSC_NE_POS_P',    2.0)
   -- param:set_and_save('PSC_NE_VEL_P',    2.5)
   -- param:set_and_save('PSC_NE_VEL_D',    0.7)
   -- param:set_and_save('PSC_NE_VEL_IMAX', 10.0)


    -- ...
    --------------------------------------------------------
    -- VALORES PARA 6DOF (Sintonización fina)
    --------------------------------------------------------
    -- Ganancia de posición: qué tan rápido quiere volver al punto.
    param:set_and_save('PSC_POSXY_P', 2.2) 

    -- Ganancia de velocidad (P): Bajamos de 2.0 (normal) a 0.4. 
    -- Esto frenará en seco las oscilaciones.
    param:set_and_save('PSC_VELXY_P', 2.1) 

    -- Ganancia Integral (I): Muy baja para que no se acumule error en el suelo.
    param:set_and_save('PSC_VELXY_I', 0.05)
    
    -- Ganancia Derivativa (D): Es el "amortiguador". La subimos un poco.
    param:set_and_save('PSC_VELXY_D', 1.9)

    -- Filtro de aceleración: Suaviza las órdenes que van a los servos.
    --param:set_and_save('PSC_VELXY_FILT', 2.0) 
    
    gcs:send_text(6, "Gains 6DoF suavizadas")
end

    --------------------------------------------------------
    -- LOITER
    --------------------------------------------------------

    param:set_and_save('LOIT_BRK_DELAY', 0.5)
    param:set_and_save('LOIT_SPEED',  5000)
    param:set_and_save('LOIT_ACC_MAX',  200)
    param:set_and_save('LOIT_BRK_JERK',  300)
    param:set_and_save('LOIT_BRK_ACC',  200)

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

    param:set_and_save('COMPASS_ENABLE', 1) -- Maestro de brújula
    param:set_and_save('COMPASS_USE',    1) -- Usar brújula 1
    param:set_and_save('COMPASS_USE2',   1) -- Usar brújula 2 (si existe)
    param:set_and_save('COMPASS_USE3',   0) -- La 3 suele ser externa/opcional

    -- Configurar el EKF3 para que use la brújula como fuente de Yaw
    -- SRC1_YAW = 1 significa "Compass"
    param:set_and_save('EK3_SRC1_YAW',   1)
    param:set_and_save('EK3_MAG_CAL', 3)

    -- GPS
    param:set_and_save('AHRS_GPS_USE',   1)
    param:set_and_save('EK3_GPS_CHECK',  31)

    gcs:send_text(6, "Parámetros Hexa Tilt cargados")

------------------------------------------------------------
-- UPDATE LOOP
------------------------------------------------------------

local ejecutado = false
local LOITER    = 29

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
        -- 1. Incrementa el ángulo basándose en el tiempo transcurrido (20ms)
        actual_roll_offset = actual_roll_offset + (VELOCIDAD_ROLL_DEG_S * LOOP_PERIOD_S)

        -- 2. Mantén el ángulo dentro del rango de 0 a 360 grados
        if actual_roll_offset >= 360.0 then
            actual_roll_offset = actual_roll_offset - 360.0
        end

        -- 3. Aplica el offset dinámico al controlador
          attitude_control:set_forward_enable(true)
    attitude_control:set_lateral_enable(true)
        --attitude_control:set_offset_roll_pitch(0.0,0.0)
        --attitude_control:set_offset_roll_pitch(0.0,actual_roll_offset)
        attitude_control:set_offset_roll_pitch(actual_roll_offset,0.0)
    else
        -- Si sales de Loiter, resetea el ángulo a 0 de forma segura
        actual_roll_offset = 0.0
        attitude_control:set_offset_roll_pitch(0.0, 0)
    end

    return update, 20
end

return update()
