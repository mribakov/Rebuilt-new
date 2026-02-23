
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ClimberDoor extends Command {
  private Elevator climb;
  private boolean open;

  public ClimberDoor(Elevator climber, boolean open) {
    climb = climber;
    this.open = open;
    addRequirements(climb);
  }

  @Override
  public void initialize() {
    climb.setDoor(open);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
