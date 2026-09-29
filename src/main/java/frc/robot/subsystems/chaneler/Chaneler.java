package frc.robot.subsystems.chaneler;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.chaneler.commands.SetVelocity;

/**
 * Subsistema del chaneler del robot.
 *
 * Gestiona el motor de transporte de notas hacia el shooter.
 * Provee comandos predefinidos para mover y detener el mecanismo.
 */
public class Chaneler extends SubsystemBase {

    public final ChanelerIOInputsAutoLogged inputs = new ChanelerIOInputsAutoLogged();

    /** Interfaz I/O del subsistema. */
    private final ChanelerIO io;

    /**
     * Crea el subsistema Chaneler.
     *
     * @param io interfaz de hardware (real o simulada)
     */
    public Chaneler(ChanelerIO io) {
        this.io = io;
    }

    // --------------------------------------------------------------------
    // COMANDOS
    // --------------------------------------------------------------------

    /**
     * Comando para alimentar la fuel hacia el shooter a velocidad de release.
     *
     * @return Comando que mueve el chaneler a {@link ChanelerConfig#FEED_VELOCITY_RPS}
     */
    public Command feedCommand() {
        return new SetVelocity(ChanelerConfig.FEED_VELOCITY_RPS, io, this);
    }

    /**
     * Comando para devolver la fuel en dirección contraria.
     *
     * @return Comando que mueve el chaneler a {@link ChanelerConfig#REVERSE_VELOCITY_RPS}
     */
    public Command reverseCommand() {
        return new SetVelocity(ChanelerConfig.REVERSE_VELOCITY_RPS, io, this);
    }

    /**
     * Comando para mover el chaneler a una velocidad personalizada.
     *
     * @param velocityRps velocidad objetivo en RPS
     * @return Comando que mantiene esa velocidad hasta ser cancelado
     */
    public Command setVelocityCommand(double velocityRps) {
        return new SetVelocity(velocityRps, io, this);
    }

    /**
     * Comando para detener el chaneler.
     *
     * @return Comando instantáneo que detiene el motor
     */
    public Command stopCommand() {
        return new InstantCommand(() -> io.stop(), this);
    }

    // --------------------------------------------------------------------
    // PERIODIC
    // --------------------------------------------------------------------

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Chaneler", inputs);
    }
}