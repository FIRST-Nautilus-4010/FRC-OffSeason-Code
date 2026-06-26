package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.intake.commands.Move;

/**
 * Subsistema de ingesta de fuel del robot.
 *
 * Gestiona el pivote del intake y los motores de giro (spinners) para
 * recoger y expulsar fuel. Proporciona comandos predefinidos para las
 * operaciones de grabbing, release y stow.
 */
public class Intake extends SubsystemBase{

    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    
    /** Interfaz I/O del subsistema. */
    private final IntakeIO io;

    /**
     * Crea el subsistema Intake.
     *
     * Inicializa la interfaz de hardware y el controlador con referencias
     * a los motores de giro (principal y secundario) y el pivote.
     */
    public Intake(IntakeIO io) {
        this.io = io;
    }

    /**
     * Comando para recoger una nota (grab).
     * 
     * Posiciona el pivote y configura velocidades según 
     * {@link IntakeConstants#GRAB_ANGLE_RAD} y {@link IntakeConstants#GRAB_SPIN_RPS}.
     *
     * @return Comando de movimiento a posición de recogida
     */
    public Command grabCommand() {
        return new Move(IntakeConfig.GRAB_ANGLE_RAD, IntakeConfig.GRAB_SPIN_RPS, io, this);
    }

    /**
     * Comando para expulsar una nota (release).
     * 
     * Posiciona el pivote y configura velocidades según 
     * {@link IntakeConstants#RELEASE_ANGLE_RAD} y {@link IntakeConstants#RELEASE_SPIN_RPS}.
     *
     * @return Comando de movimiento a posición de expulsión
     */
    public Command releaseCommand() {
        return new Move(IntakeConfig.RELEASE_ANGLE_RAD, IntakeConfig.RELEASE_SPIN_RPS, io, this);
    }

    /**
     * Comando para guardar/retraer el intake (stow).
     * 
     * Posiciona el pivote y detiene los spinners según 
     * {@link IntakeConstants#STOW_ANGLE_RAD} y {@link IntakeConstants#STOW_SPIN_RPS}.
     *
     * @return Comando de movimiento a posición de reposo
     */
    public Command stowCommand() {
        return new Move(IntakeConfig.STOW_ANGLE_RAD, IntakeConfig.STOW_SPIN_RPS, io, this);
    }

    /**
     * Comando de prueba para los motores de giro.
     * 
     * Mantiene el pivote en su posición actual y ejecuta los spinners
     * a velocidad de grabbing para testing.
     *
     * @return Comando de movimiento para prueba de spinners
     */
    public Command testRollersCommand() {
        return new Move(io.getInputs().intakeData.positionPivot(), IntakeConfig.GRAB_SPIN_RPS, io, this);
    }

    /**
     * Comando para detener todos los motores del subsistema.
     *
     * @return Comando instantáneo que detiene pivote y spinners
     */
    public Command stopCommand() {
        return new InstantCommand(() -> io.stop(), this);
    }

    /**
     * Actualiza el subsistema periódicamente.
     * 
     * Publica la posición del pivote y velocidad de los spinners a SmartDashboard
     * para telemetría y debugging.
     */
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        Logger.recordOutput("Mechanism3d/Measured/Intake", 
            new Pose3d(
                new Translation3d(
                    0.22 + 0.0735, 
                0, 
                0.22 - 0.005
                ),
                new Rotation3d(
                    0, -inputs.intakeData.positionPivot(), 0
                ) 
            )
        );
    }
}