package frc.robot.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.operator.controllers.BionicF310;

public class Swerve extends CommandBase {

    public Swerve(SwerveSubsystem subsystem) {
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        Robot.drivetrain.swerveDrive(
            Robot.driverGamepad.getThresholdAxis(BionicF310.RX), //TODO check direction of robot to see if these need to be negated.
            Robot.driverGamepad.getThresholdAxis(BionicF310.RY),
            Robot.driverGamepad.getThresholdAxis(BionicF310.LX)
        );
    }

}
