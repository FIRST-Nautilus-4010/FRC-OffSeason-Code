package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class ShooterConfig {

    private ShooterConfig() {}

    // ====================================================================
    // IDs CAN
    // ====================================================================
    public static final int SPIN_L_TALONFX_ID = 1;
    public static final int SPIN_R_TALONFX_ID = 2;

    // ====================================================================
    // REDUCCIÓN
    // ====================================================================
    /** Relación sensor → mecanismo (1:1 si no hay caja). */
    public static final double SPIN_REDUCTION = 1.0;

    // ====================================================================
    // MOTION MAGIC
    // ====================================================================
    public static final double MAGIC_MOTION_VELOCITY_ACCELERATION_STR = 950;
    public static final double MAGIC_MOTION_VELOCITY_JERK_STR         = 9500;

    // ====================================================================
    // GANANCIAS PID
    // ====================================================================
    public static final double VEL_KS = 0.10442;
    public static final double VEL_KV = 0.10882;
    public static final double VEL_KA = 0.001647;
    public static final double VEL_KP = 0.4;
    public static final double VEL_KI = 0.00;
    public static final double VEL_KD = 0.001;

    // ====================================================================
    // VELOCIDADES PREDEFINIDAS (rad/s para consistencia con IntakeIO)
    // ====================================================================
    public static final double RELEASE_VELOCITY_RPS = -10.0;

    // ====================================================================
    // SIMULACIÓN
    // ====================================================================

    /** Momento de inercia del flywheel (kg·m²). Ajusta según tu CAD. */
    public static final double MOMENT_OF_INERTIA_KG_M2 = 0.001;

    /** Desviaciones estándar para ruido en la simulación [posición, velocidad]. */
    public static final double[] STD_DEVS_SHOOTER = new double[]{0.0, 0.001};

    // ====================================================================
    // SIMULACIÓN
    // ====================================================================

    /** Velocidad mínima para considerar que el shooter puede lanzar (RPS). */
    public static final double MIN_SHOOT_VELOCITY_RPS = 5.0;

    /** Altura inicial del fuel al salir del shooter (metros). */
    public static final double SHOOTER_HEIGHT_METERS = 0.4;

    /** Velocidad de salida del proyectil (m/s). */
    public static final double SHOOTER_EXIT_VELOCITY_MPS = 8.0;

    /** Ángulo de lanzamiento del shooter (grados). */
    public static final double SHOOTER_ANGLE_DEGREES = 70;

    public static final InterpolatingDoubleTreeMap VEL_TABLE = new InterpolatingDoubleTreeMap();

    static {
        VEL_TABLE.put(1.5, 46.0);
        VEL_TABLE.put(2.0, 46.0);
        VEL_TABLE.put(2.5, 49.0);
        VEL_TABLE.put(3.0, 52.0);
        VEL_TABLE.put(3.5, 54.0);
        VEL_TABLE.put(4.0, 56.5);
        VEL_TABLE.put(4.5, 59.2);
        VEL_TABLE.put(5.0, 61.3);

    }
}