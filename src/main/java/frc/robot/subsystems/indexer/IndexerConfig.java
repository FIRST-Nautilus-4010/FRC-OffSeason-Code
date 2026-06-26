package frc.robot.subsystems.indexer;

public class IndexerConfig {

    // --------------------------------------------------------------------
    // CAN IDs
    // --------------------------------------------------------------------

    public static final int INDEXER_L_TALONFX_ID = 0; // TODO: cambiar al ID real
    public static final int INDEXER_R_TALONFX_ID = 1; // TODO: cambiar al ID real

    // --------------------------------------------------------------------
    // VELOCIDADES
    // --------------------------------------------------------------------

    /** Velocidad para indexar la nota hacia el chaneler (RPS) */
    public static final double FEED_VELOCITY_RPS = 30.0; // TODO: ajustar

    /** Velocidad para devolver la nota en dirección contraria (RPS) */
    public static final double REVERSE_VELOCITY_RPS = -20.0; // TODO: ajustar

    // --------------------------------------------------------------------
    // MECÁNICA
    // --------------------------------------------------------------------

    /** Relación de reducción del indexer (rotaciones del motor / rotaciones del mecanismo) */
    public static final double INDEXER_REDUCTION = 1.0; // TODO: ajustar según tu gearbox

    // --------------------------------------------------------------------
    // GANANCIAS (Slot 0 - Velocidad)
    // --------------------------------------------------------------------

    /** Voltaje estático para vencer la fricción (V) */
    public static final double VEL_KS = 0.0;

    /** Voltaje por unidad de velocidad (V / (rot/s)) */
    public static final double VEL_KV = 0.0;

    /** Voltaje por unidad de aceleración (V / (rot/s²)) */
    public static final double VEL_KA = 0.0;

    /** Ganancia proporcional */
    public static final double VEL_KP = 0.0;

    /** Ganancia integral */
    public static final double VEL_KI = 0.0;

    /** Ganancia derivativa */
    public static final double VEL_KD = 0.0;

    // --------------------------------------------------------------------
    // MOTION MAGIC
    // --------------------------------------------------------------------

    /** Aceleración máxima en MotionMagic (rot/s²) */
    public static final double MAGIC_MOTION_VELOCITY_ACCELERATION = 0.0; // TODO: tunar

    /** Jerk máximo en MotionMagic (rot/s³) */
    public static final double MAGIC_MOTION_VELOCITY_JERK = 0.0; // TODO: tunar
}