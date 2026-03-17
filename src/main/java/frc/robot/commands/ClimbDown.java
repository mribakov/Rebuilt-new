
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ClimbDown extends Command {
  private Elevator climb;

  public ClimbDown(Elevator climber) {
    climb = climber;
    addRequirements(climb);
  }

  @Override
  public void execute() {
    climb.setTargetSetpoint(Elevator.Setpoint.Starting);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(climb.getPosition().magnitude() - Elevator.Setpoint.Starting.target.magnitude())
        <= Constants.Climber.SETPOINT_THRESHOLD_ROT;
  }
}
