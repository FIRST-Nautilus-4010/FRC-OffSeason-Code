package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;

/**
 * Comando para establecer la velocidad del shooter.
 *
 * Envía continuamente la velocidad objetivo al motor de lanzamiento
 * hasta que el comando sea interrumpido o cancelado.
 */
public class SetVelocity extends Command {

    /** Interfaz de hardware del subsistema Shooter. */
    private final ShooterIO io;

    /** Velocidad objetivo (RPS). */
    private final double velocityRps;

    /**
     * Crea un comando para girar el shooter a una velocidad dada.
     *
     * @param velocityRps velocidad objetivo en rotaciones por segundo
     * @param io          interfaz de hardware del shooter
     * @param shooter     subsistema Shooter (para requirements)
     */
    public SetVelocity(double velocityRps, ShooterIO io, Shooter shooter) {
        this.velocityRps = velocityRps;
        this.io = io;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        io.setVelocity(velocityRps);
    }

    @Override
    public void end(boolean interrupted) {
        io.stop();
    }
}