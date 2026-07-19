package frc.robot.subsystems.chaneler;

public class ChanelerConfig {

    // --------------------------------------------------------------------
    // CAN ID
    // --------------------------------------------------------------------

    public static final int CHANELER_TALONFX_ID = 5;

    // --------------------------------------------------------------------
    // VELOCIDADES
    // --------------------------------------------------------------------

    /** Velocidad para alimentar la fuel hacia el shooter (RPS) */
    public static final double FEED_VELOCITY_RPS = 71.67; // TODO: ajustar

    /** Velocidad para devolver la fuel en dirección contraria (RPS) */
    public static final double REVERSE_VELOCITY_RPS = -20.0; // TODO: ajustar

    // --------------------------------------------------------------------
    // MECÁNICA
    // --------------------------------------------------------------------

    /** Relación de reducción del chaneler (rotaciones del motor / rotaciones del mecanismo) */
    public static final double CHANELER_REDUCTION = 1.0; // TODO: ajustar según tu gearbox

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
    public static final double MAGIC_MOTION_VELOCITY_ACCELERATION = 1000; // TODO: tunear

    /** Jerk máximo en MotionMagic (rot/s³) */
    public static final double MAGIC_MOTION_VELOCITY_JERK = 10000; // TODO: tunear
}