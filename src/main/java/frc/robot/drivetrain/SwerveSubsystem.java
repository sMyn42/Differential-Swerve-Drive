package frc.robot.drivetrain;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

public class SwerveSubsystem extends SubsystemBase{
    // Locations for the swerve drive modules relative to the robot center.
    private static final double WHEELBASE = 12.5;
    private static final double TRACKWIDTH = 13.5;
    private static final double RATIO = Math.sqrt(Math.pow(WHEELBASE, 2) + Math.pow(TRACKWIDTH, 2));

    /*
     * 0 is Front Right
     * 1 is Front Left
     * 2 is Back Left
     * 3 is Back Right
     */
    private SwerveModule[] mSwerveModules = new SwerveModule[] {
        new SwerveModule(0, new TalonFX(1), new TalonFX(2), new DutyCycleEncoder(1)),
        new SwerveModule(1, new TalonFX(3), new TalonFX(4), new DutyCycleEncoder(2)),
        new SwerveModule(2, new TalonFX(5), new TalonFX(6), new DutyCycleEncoder(3)),
        new SwerveModule(3, new TalonFX(7), new TalonFX(8), new DutyCycleEncoder(4))
    };

    private Translation2d m_frontLeftLocation = new Translation2d(WHEELBASE/2/3.281, TRACKWIDTH/2/3.281);
    private Translation2d m_frontRightLocation = new Translation2d(WHEELBASE/2/3.281, - TRACKWIDTH/2/3.281);
    private Translation2d m_backLeftLocation = new Translation2d(- WHEELBASE/2/3.281, TRACKWIDTH/2/3.281);
    private Translation2d m_backRightLocation = new Translation2d(- WHEELBASE/2/3.281, - TRACKWIDTH/2/3.281);

    // Creating my kinematics object using the module locations

    private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
        );

    public SwerveSubsystem(){ //lengths in feet, sorry for no docs..

        
    }

    public double[][] toDifferentialSwerve(SwerveModuleState[] input){
        SwerveModuleState[] states = input;
        double[][] motorStates = new double[2][4];

        motorStates[0][0] = states[0].speedMetersPerSecond; 
        motorStates[0][1] = -motorStates[0][0];
        motorStates[1][0] = states[1].speedMetersPerSecond; 
        motorStates[1][1] = -motorStates[1][0];
        motorStates[2][0] = states[2].speedMetersPerSecond; 
        motorStates[2][1] = -motorStates[2][0];
        motorStates[3][0] = states[3].speedMetersPerSecond; 
        motorStates[3][1] = -motorStates[3][0];
        
        return motorStates;
    }

    public void swerveDrive(double rightstickx, double rightsticky, double leftstickx){  //sets motors when given stick inputs
        double vx = rightsticky;
        double vy = -rightstickx;
        double omegaRadiansPerSecond = leftstickx;
        double robotAngle = 0.0; //ADD GYRO, think abt odometry, rn this is absolute, robot-relative.

        //Begin library based calculations of module speeds in place of actual vector math.
        ChassisSpeeds relativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omegaRadiansPerSecond, Rotation2d.fromDegrees(robotAngle));

        //get each individual swerve module state
        SwerveModuleState[] states = Robot.drivetrain.m_kinematics.toSwerveModuleStates(relativeSpeeds); //Optional center of rotation param, Translation2d
    
        SwerveDriveKinematics.normalizeWheelSpeeds(states, 1.0); //preserves direction of robot.

        //TODO need to account for differential
        // Both motors turn (POSITIVE), then wheel turns, opposite directions causes module turning.

        //ACTUATE MOTORS FOR LINEAR SPEED (8 lines) (WHEEL SPEEDS WILL REQUIRE PID)
        // frontLeft.set(states[0].speedMetersPerSecond);
        // frontRight.set(states[1].speedMetersPerSecond);
        // backLeft.set(states[2].speedMetersPerSecond);
        // backRight.set(states[3].speedMetersPerSecond);

        // frontLeftA.set(states[0].speedMetersPerSecond);
        // frontRightA.set(states[1].speedMetersPerSecond);
        // backLeftA.set(states[2].speedMetersPerSecond);
        // backRightA.set(states[3].speedMetersPerSecond);

        // //ROTATIONAL MOTOR OUTPUT ADJUSTMENT (NEEDS PID)
        //     //Sub-task: calculate angle from difference in motor ~=~ (sum of 2 motor encoder vals)/(ticks/full rev -> names 'ticks') * 2pi radians/rev
        //     //do this calc in function toDifferentialSwerve


        // SmartDashboard.putNumber("Front Left Module Angle", frontLeftA.getSelectedSensorPosition());
        // //frontLeftA.set(ControlMode.Position, states[0].angle.getDegrees()/360*ticks); 
        // SmartDashboard.putNumber("Front Right Module Angle", frontRightA.getSelectedSensorPosition());
        // //frontRightA.set(ControlMode.Position, states[1].angle.getDegrees()/360*ticks); 
        // SmartDashboard.putNumber("Back Left Module Angle", backLeftA.getSelectedSensorPosition());
        // //backLeftA.set(ControlMode.Position, states[2].angle.getDegrees()/360*ticks); 
        // SmartDashboard.putNumber("Back Right Module Angle", backRightA.getSelectedSensorPosition());
        // //backRightA.set(ControlMode.Position, states[3].angle.getDegrees()/360*ticks);
    }

    //TODO public double anglePID(){}, add gyro code to make robotangle a real thing.

}