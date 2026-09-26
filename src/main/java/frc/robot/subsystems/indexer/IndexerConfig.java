package frc.robot.subsystems.indexer;

public class IndexerConfig {

    // --------------------------------------------------------------------
    // CAN IDs
    // --------------------------------------------------------------------

    public static final int INDEXER_L_TALONFX_ID = 3;
    public static final int INDEXER_R_TALONFX_ID = 4;

    // --------------------------------------------------------------------
    // VELOCIDADES
    // --------------------------------------------------------------------

    /** Velocidad para indexar la fuel hacia el chaneler (RPS) */
    public static final double FEED_VELOCITY_RPS = 71.67; // TODO: ajustar

    /** Velocidad para devolver la fuel en dirección contraria (RPS) */
    public static final double REVERSE_VELOCITY_RPS = -20.0; // TODO: ajustar

    // --------------------------------------------------------------------
    // MECÁNICA
    // --------------------------------------------------------------------

    /** Relación de reducción del indexer (rotaciones del motor / rotaciones del mecanismo) */
    public static final double INDEXER_REDUCTION = 1.0; // TODO: ajustar según tu gearbox

    // --------------------------------------------------------------------
    // GANANCIAS (Slot 0 - Velocidad)
    // --------------------------------------------------------------------

    public static final double VEL_KS = 0.10442;
    public static final double VEL_KV = 0.10882;
    public static final double VEL_KA = 0.001647;
    public static final double VEL_KP = 0.4;
    public static final double VEL_KI = 0.00;
    public static final double VEL_KD = 0.001;

    // --------------------------------------------------------------------
    // MOTION MAGIC
    // --------------------------------------------------------------------

    /** Aceleración máxima en MotionMagic (rot/s²) */
    public static final double MAGIC_MOTION_VELOCITY_ACCELERATION = 950; // TODO: tunear

    /** Jerk máximo en MotionMagic (rot/s³) */
    public static final double MAGIC_MOTION_VELOCITY_JERK = 9500; // TODO: tunear
}