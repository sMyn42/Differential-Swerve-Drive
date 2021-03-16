package frc.robot.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

//Implemented as a copypasta of Jack-in-the-Bot's code, with a bunch of edits.

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class SwerveModule extends PIDSubsystem{

    private double mLastTargetAngle;
    private int mModuleNumber;
    private DutyCycleEncoder mEncoder;
    private TalonFX mTopMotor, mBottomMotor;
    private double mTranslationalSpeed;

    //TODO Check if inverts are needed. (only a double invert)

    /*
    *   In the Constructor, instantiate new motors. 
    */
    public SwerveModule(int moduleNumber, TalonFX topMotor, TalonFX bottomMotor, DutyCycleEncoder encoder){
        super(new PIDController(RobotConstants.swerveModuleTurningkP, RobotConstants.swerveModuleTurningkI, RobotConstants.swerveModuleTurningkD));

        mModuleNumber = moduleNumber;
        mTopMotor = topMotor;
        mBottomMotor = bottomMotor;
        mEncoder = encoder;
        //Config Encoder
        mEncoder.reset();

        //Config Motors
        mTopMotor.configFactoryDefault();
        mTopMotor.setNeutralMode(NeutralMode.Brake);
        mTopMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, RobotConstants.talonPIDidx1, RobotConstants.kTimeoutMs);
        mTopMotor.config_kP(RobotConstants.talonPIDidx1, RobotConstants.talonkP, RobotConstants.kTimeoutMs);
        mTopMotor.config_kI(RobotConstants.talonPIDidx1, RobotConstants.talonkI, RobotConstants.kTimeoutMs);
        mTopMotor.config_kD(RobotConstants.talonPIDidx1, RobotConstants.talonkD, RobotConstants.kTimeoutMs);

        mBottomMotor.configFactoryDefault();
        mBottomMotor.setNeutralMode(NeutralMode.Brake);
        mBottomMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, RobotConstants.talonPIDidx1, RobotConstants.kTimeoutMs);
        mTopMotor.config_kP(RobotConstants.talonPIDidx1, RobotConstants.talonkP, RobotConstants.kTimeoutMs);
        mTopMotor.config_kI(RobotConstants.talonPIDidx1, RobotConstants.talonkI, RobotConstants.kTimeoutMs);
        mTopMotor.config_kD(RobotConstants.talonPIDidx1, RobotConstants.talonkD, RobotConstants.kTimeoutMs);
    }

    public double getError(){
        return this.m_controller.getPositionError();
    }

    public void setSpeeds(double linearSpeedTop, double linearSpeedBottom){
        mTopMotor.set(ControlMode.Velocity, linearSpeedTop);
        mBottomMotor.set(ControlMode.Velocity, linearSpeedBottom);
    }

    public void setTranslationalSpeed(double speed){
        mTranslationalSpeed = speed;
    }

    public double getTranslationalSpeed(){
        return mTranslationalSpeed;
    }

    public void setTargetAngle(double targetAngle){
        mLastTargetAngle = targetAngle;
        setSetpoint(mLastTargetAngle);
    }

    public double getTargetAngle(){
        return mLastTargetAngle;
    }

    public double getCurrentAngle(){
        return mEncoder.get();
    }

    @Override
    public void useOutput(double output, double setpoint) {
        setSpeeds(mTranslationalSpeed + output, mTranslationalSpeed - output); //TODO, which gets subtracted??
    }

    @Override
    public double getMeasurement() {
        return this.getCurrentAngle();
    }
}