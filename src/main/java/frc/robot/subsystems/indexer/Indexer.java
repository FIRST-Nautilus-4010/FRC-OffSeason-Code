package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.commands.SetVelocity;

/**
 * Subsistema del indexer del robot.
 *
 * Gestiona los dos motores de transporte de notas (left líder, right follower
 * en dirección opuesta). Provee comandos predefinidos para indexar,
 * revertir y detener el mecanismo.
 */
public class Indexer extends SubsystemBase {

    public final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    /** Interfaz I/O del subsistema. */
    private final IndexerIO io;

    /**
     * Crea el subsistema Indexer.
     *
     * @param io interfaz de hardware (real o simulada)
     */
    public Indexer(IndexerIO io) {
        this.io = io;
    }

    // --------------------------------------------------------------------
    // COMANDOS
    // --------------------------------------------------------------------

    /**
     * Comando para alimentar la fuel hacia el chaneler.
     *
     * @return Comando que mueve el indexer a {@link IndexerConfig#FEED_VELOCITY_RPS}
     */
    public Command feedCommand() {
        return new SetVelocity(IndexerConfig.FEED_VELOCITY_RPS, io, this);
    }

    /**
     * Comando para devolver la fuel en dirección contraria.
     *
     * @return Comando que mueve el indexer a {@link IndexerConfig#REVERSE_VELOCITY_RPS}
     */
    public Command reverseCommand() {
        return new SetVelocity(IndexerConfig.REVERSE_VELOCITY_RPS, io, this);
    }

    /**
     * Comando para mover el indexer a una velocidad personalizada.
     *
     * @param velocityRps velocidad objetivo en RPS
     * @return Comando que mantiene esa velocidad hasta ser cancelado
     */
    public Command setVelocityCommand(double velocityRps) {
        return new SetVelocity(velocityRps, io, this);
    }

    /**
     * Comando para detener el indexer.
     *
     * @return Comando instantáneo que detiene los motores
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
        Logger.processInputs("Indexer", inputs);
    }
}