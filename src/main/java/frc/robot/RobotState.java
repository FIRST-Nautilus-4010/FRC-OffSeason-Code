package frc.robot;

/**
 * Enumeración de estados operacionales del robot.
 *
 * Define los modos principales en los que el robot puede operar durante
 * un partido. La máquina de estados centralizada en {@link SubsystemManager}
 * coordina todos los subsistemas basándose en el estado actual.
 *
 * Cada estado activa/desactiva subsistemas específicos y configura
 * comportamientos predeterminados para operación fluida.
 */
public enum RobotState {
    
    /**
     * Estado de conducción normal.
     *
     * El robot puede ser conducido libremente por el operador usando el joystick.
     * Subsistemas activos:
     * <ul>
     *   <li>Swerve: Control field-relative con joystick</li>
     *   <li>Intake: Stow (guardado)</li>
     *   <li>Shooter/Chaneler: Inactivos</li>
     * </ul>
     * Se utiliza para navegar por el campo y posicionarse antes de otras operaciones.
     */
    TRAVEL,
    
    /**
     * Estado de recolección de fuel.
     *
     * Activa el subsistema de intake para recopilar fuel del campo.
     * Subsistemas activos:
     * <ul>
     *   <li>Swerve:  Control field-relative con joystick</li>
     *   <li>Intake: Activo en modo GRAB (spinners girando hacia adentro)</li>
     *   <li>Shooter/Chaneler: Inactivos</li>
     * </ul>
     * El fuel recolectado se transporta automáticamente hacia el shooter via Chaneler.
     */
    INTAKE,
    
    /**
     * Estado de lanzamiento de fuel.
     *
     * Activa el subsistema de shooter para lanzar fuel hacia objetivos.
     * Subsistemas activos:
     * <ul>
     *   <li>Swerve: Control con aiming automático</li>
     *   <li>Intake: Stow (guardado)</li>
     *   <li>Shooter: Activo calculando velocidad basada en distancia</li>
     *   <li>Chaneler: Sincronizado con velocidad del shooter</li>
     * </ul>
     * El robot puede usar visión con AprilTags para calcular distancia y velocidad automáticamente.
     */
    SHOOT,
    
    /**
     * Estado de escalada.
     *
     * Activa el subsistema de climber para ascender por cadenas al final del partido.
     * Subsistemas activos:
     * <ul>
     *   <li>Swerve: Va a la posicion de escalada</li>
     *   <li>Climber: Ejecutando secuencia de escalada (RISE → EXTEND → PULL)</li>
     *   <li>Intake/Shooter/Chaneler: Inactivos</li>
     * </ul>
     * Utiliza posiciones predefinidas en {@link frc.robot.subsystems.climber.ClimberConstants}
     * para una escalada confiable y repetible.
     */
    CLIMB,
    
    /**
     * Estado de prueba y diagnóstico.
     *
     * Modo especial para validar funcionamiento de subsistemas individuales.
     * Subsistemas:
     * <ul>
     *   <li>Todos los subsistemas pueden ser controlados manualmente</li>
     *   <li>Se desactivan comportamientos automáticos</li>
     *   <li>Útil para debugging y calibración</li>
     * </ul>
     * Se accede mediante botón Y del controlador Xbox.
     */
    TEST,
    OUTAKE,
    INTAKE_TEST,
    SHOOT_MANUAL
}
