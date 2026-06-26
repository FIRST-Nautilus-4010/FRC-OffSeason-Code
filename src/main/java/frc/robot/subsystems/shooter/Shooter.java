package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.commands.SetVelocity;

/**
 * Subsistema del shooter del robot.
 *
 * Gestiona el motor de lanzamiento (y su seguidor) para expulsar fuel
 * a la velocidad deseada. Provee comandos predefinidos para lanzar y detener.
 */
public class Shooter extends SubsystemBase {

    public final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    /** Interfaz I/O del subsistema. */
    private final ShooterIO io;

    /**
     * Crea el subsistema Shooter.
     *
     * @param io interfaz de hardware (real o simulada)
     */
    public Shooter(ShooterIO io) {
        this.io = io;
    }

    // --------------------------------------------------------------------
    // COMANDOS
    // --------------------------------------------------------------------

    /**
     * Comando para lanzar fuel a la velocidad de release.
     *
     * @return Comando que gira el shooter a {@link ShooterConstants#RELEASE_VELOCITY_RPS}
     */
    public Command releaseCommand(double distance) {
        return new SetVelocity(ShooterConfig.VEL_TABLE.get(distance), io, this);
    }

    /**
     * Comando para girar el shooter a una velocidad personalizada.
     *
     * @param velocityRps velocidad objetivo en RPS
     * @return Comando que mantiene esa velocidad hasta ser cancelado
     */
    public Command setVelocityCommand(double velocityRps) {
        return new SetVelocity(velocityRps, io, this);
    }

    /**
     * Comando para detener el shooter.
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
        Logger.processInputs("Shooter", inputs);
    }
}