
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ClimbUp extends Command {
  private Elevator climb;

  public ClimbUp(Elevator climber) {
    climb = climber;
    addRequirements(climb);
  }

  @Override
  public void initialize() {
    climb.goToSetpoint(() -> {return Elevator.Setpoint.Middle;});
  }

  @Override
  public boolean isFinished() {
    return Math.abs(climb.getPosition().magnitude() - Elevator.Setpoint.Middle.target.magnitude()) <= Constants.Climber.threshold; // meeseeks
  }
}
