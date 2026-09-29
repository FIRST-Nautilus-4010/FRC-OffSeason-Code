package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Conjunto de constantes específicas del sistema swerve.
 *
 * Solo se incluyen las que realmente se usan en las clases actuales
 * (módulos, controlador, IO, etc.).
 */
public final class SwerveConfig {

    private SwerveConfig() {
        // Clase de solo constantes: no instanciable.
    }

    // --------------------------------------------------------------------
    // IDs DE DISPOSITIVOS (CTR)
    // --------------------------------------------------------------------

    /** CAN ID del motor de tracción del módulo delantero izquierdo. */
    public static final int FL_PWR = 4;
    /** CAN ID del motor de tracción del módulo delantero derecho. */
    public static final int FR_PWR = 3;
    /** CAN ID del motor de tracción del módulo trasero izquierdo. */
    public static final int BL_PWR = 2;
    /** CAN ID del motor de tracción del módulo trasero derecho. */
    public static final int BR_PWR = 1;

    /** CAN ID del motor de giro del módulo delantero izquierdo. */
    public static final int FL_STR = 8;
    /** CAN ID del motor de giro del módulo delantero derecho. */
    public static final int FR_STR = 7;
    /** CAN ID del motor de giro del módulo trasero izquierdo. */
    public static final int BL_STR = 6;
    /** CAN ID del motor de giro del módulo trasero derecho. */
    public static final int BR_STR = 5;

    /** CAN ID del encoder absoluto del módulo delantero izquierdo. */
    public static final int FL_ENC = 12;
    /** CAN ID del encoder absoluto del módulo delantero derecho. */
    public static final int FR_ENC = 11;
    /** CAN ID del encoder absoluto del módulo trasero izquierdo. */
    public static final int BL_ENC = 10;
    /** CAN ID del encoder absoluto del módulo trasero derecho. */
    public static final int BR_ENC = 9;

    /** CAN ID del gyro Pigeon2 usado como IMU principal. */
    public static final int PIGEON = 13;

    // --------------------------------------------------------------------
    // GEOMETRÍA Y CONVERSIONES
    // --------------------------------------------------------------------

    /** Diámetro de la rueda en metros. */
    public static final double WHEEL_DIAMETER = 0.1016;

    /**
     * Relación de transmisión del motor de tracción (drive).
     * <p>
     * Vueltas de motor por cada vuelta de rueda.
     */
    public static final double PWR_RATIO = 6.03;

    /**
     * Relación de transmisión del motor de giro (steer).
     * <p>
     * Vueltas de motor por cada vuelta completa del módulo.
     */
    public static final double STR_RATIO = 26.09;

    /**
     * Factor de conversión de rotaciones de motor de tracción a metros
     * recorridos por el módulo.
     *
     * rotaciones_motor * ROT_2_M = metros
     */
    public static final double ROT_2_M =
            (Math.PI * WHEEL_DIAMETER) / PWR_RATIO;

    /**
     * Factor de conversión de rotaciones del motor de giro a radianes de ángulo
     * del módulo.
     *
     * rotaciones_motor * ROT_2_RAD = radianes
     */
    public static final double ROT_2_RAD =
            (2.0 * Math.PI) / STR_RATIO;

    // --------------------------------------------------------------------
    // MOTION MAGIC EXPO - STEER (POSICIÓN)
    // --------------------------------------------------------------------

    /** Velocidad de crucero de Motion Magic para el steer (rot/s). */
    public static final double MAGIC_MOTION_VELOCITY_STR = 100;

    /** Aceleración de Motion Magic para el steer (rot/s²). */
    public static final double MAGIC_MOTION_ACCELERATION_STR = 1000;

    /** Jerk de Motion Magic para el steer (rot/s³). */
    public static final double MAGIC_MOTION_JERK_STR = 0;

    /**
     * Ganancia kV del modo Motion Magic Expo para el steer.
     * <p>
     * Escala la contribución de la velocidad en el perfil de movimiento.
     */
    public static final double MAGIC_MOTION_EXPO_KV_STR = 0.12;

    /**
     * Ganancia kA del modo Motion Magic Expo para el steer.
     * <p>
     * Escala la contribución de la aceleración en el perfil de movimiento.
     */
    public static final double MAGIC_MOTION_EXPO_KA_STR = 0.10;

    // --------------------------------------------------------------------
    // GANANCIAS DE CONTROL - POSICIÓN (STEER)
    // --------------------------------------------------------------------

    /**
     * kG: salida para compensar gravedad (en este caso, torque/rozamiento
     * del módulo).
     */
    public static final double POS_KG = 0;

    /** kS: salida para vencer fricción estática (offset inicial). */
    public static final double POS_KS = 0.25;

    /** kV: salida por unidad de velocidad objetivo (output / rps). */
    public static final double POS_KV = 0.12;

    /** kA: salida por unidad de aceleración objetivo (output / (rps/s)). */
    public static final double POS_KA = 0.01;

    /** kP: salida por unidad de error de posición (output / rotación). */
    public static final double POS_KP = 10;

    /** kI: salida por unidad de error integrado de posición. */
    public static final double POS_KI = 0.0;

    /** kD: salida por unidad de error de velocidad (derivada). */
    public static final double POS_KD = 0;

    // --------------------------------------------------------------------
    // GANANCIAS DE CONTROL - VELOCIDAD (DRIVE)
    // --------------------------------------------------------------------

    /** kS: salida para vencer fricción estática en el drive. */
    public static final double VEL_KS = 0.1825;

    /** kV: salida por unidad de velocidad objetivo (output / rps). */
    public static final double VEL_KV = 1.9;

    /** kA: salida por unidad de aceleración objetivo (output / (rps/s)). */
    public static final double VEL_KA = 0.14;

    /** kP: salida por unidad de error de velocidad (output / rps). */
    public static final double VEL_KP = 3.7;

    /** kI: salida por unidad de error integrado de velocidad. */
    public static final double VEL_KI = 0.2;

    /** kD: salida por unidad de derivada del error de velocidad. */
    public static final double VEL_KD = 0.21;

    // --------------------------------------------------------------------
    // LIMITES DE ACELERACIÓN / ESTABILIDAD
    // --------------------------------------------------------------------

    /** Aceleración máxima hacia adelante (m/s²) usada en el limitador. */
    public static final double MAX_FORDWARD_ACCEL = 11.2;

    /** Aceleración máxima frontal (m/s²) en el modelo de estabilidad. */
    public static final double MAX_FRONT_ACCEL = 11.2;

    /** Aceleración máxima lateral (m/s²) en el modelo de estabilidad. */
    public static final double MAX_SIDE_ACCEL = 11.2;

    /**
     * Coeficiente de fricción efectivo rueda-suelo.
     * <p>
     * Se usa solo para derivar la aceleración lateral máxima por skid.
     */
    public static final double FRICTION_COF = 2.255;

    /**
     * Aceleración máxima antes de patinar (skid) en m/s².
     * <p>
     * Aproximada como μ * g.
     */
    public static final double MAX_SKID_ACCEL = FRICTION_COF * 9.81;

    /**
     * Zona muerta de velocidad del módulo (m/s).
     * <p>
     * Si la velocidad deseada está dentro de este rango alrededor de 0,
     * se fuerza a 0 para evitar vibraciones.
     */
    public static final double VELOCITY_DEADZONE = 0.02;

    /** Factor de asistencia al strafe en el modo asistido. */
    public static final double ASSIST_STRAFE_FACTOR = 0.4;

    public static final double CONTROL_PERIOD_SEC = 0.02; // 20 ms

    public static final double DEBUG_UPDATE_INTERVAL_SEC = 0.2; // 100 ms

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 5.3;

    /** Distancia entre ruedas derecha e izquierda (m). */
    public static final double TRACKWIDTH = 0.502;

    /** Distancia entre ruedas delanteras y traseras (m). */
    public static final double WHEELBASE = 0.61;

    /**
     * Cinemática del chasis swerve.
     * <p>
     * Define la posición de cada módulo respecto al centro del robot.
     */
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(TRACKWIDTH / 2.0,  WHEELBASE / 2.0),   // Front Left
        new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),   // Front Right
        new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),   // Back Left
        new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0));  // Back Right

    public static final String LIMELIGHT_1 = "limelight-1";
    public static final String LIMELIGHT_2 = "limelight-2";

    public static final double P_X = 25;
    public static final double I_X = 0.0;
    public static final double D_X = 0.25;

    public static final double P_Y = 25;
    public static final double I_Y = 0.0;
    public static final double D_Y = 0.25;

    public static final double P_Z = 10;
    public static final double I_Z = 0.0;
    public static final double D_Z = 0.0;
    
    /** Velocidad angular máxima permitida en auton (rad/s). */
    public static final double MAX_ANG_SPD = 11.34;

    /** Aceleración angular máxima permitida en auton (rad/s²). */
    public static final double MAX_ANG_ACCEL = 50.71;

        /**
     * Constraints para el ProfiledPIDController de theta (rotación),
     * usados por el {@link edu.wpi.first.math.controller.HolonomicDriveController}.
     */
    public static final TrapezoidProfile.Constraints Z_CONTROLER =
        new TrapezoidProfile.Constraints(
            MAX_ANG_SPD,
            MAX_ANG_ACCEL);
    public static final double MASS_KG = 61.230;
    public static final double BUMPER_LENGTH_X = 0.9144;
    public static final double BUMPER_WIDTH_Y = 0.8255;
    
    public static final Orchestra orchestra = new Orchestra();
    public static final StatusCode state = orchestra.loadMusic("output.chrp");
}
