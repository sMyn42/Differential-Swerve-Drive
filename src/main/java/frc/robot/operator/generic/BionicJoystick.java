package frc.robot.operator.generic;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * BionicJoystick abstracts away much of the prior operator interface complexity
 * to reduce the likelihood of confusion in button and axis mapping. All methods
 * rely on BionicAxis and BionicButton objects defined in their respective
 * controller's classes.
 */
public class BionicJoystick extends Joystick {
    public final double deadzone;
    public final double sensitivity;

    /**
     * @param port Port between 0...5 the USB Joystick is configured to use.
     */
    public BionicJoystick(int port, double deadzone, double sensitivity) {
        super(port);

        this.deadzone = deadzone;
        this.sensitivity = sensitivity;
    }

    /**
     * @param axis Axis to Measure
     * @return Returns axis value [-1,1]
     */
    public double getThresholdAxis(BionicAxis axis) {
        if (Math.abs(this.getRawAxis(axis.getNumber())) > Math.abs(deadzone))
            return this.getRawAxis(axis.getNumber());
        else
            return 0.0;
    }

    public double getSensitiveAxis(BionicAxis axis) {
        double axisValue = getThresholdAxis(axis);

//        return (1 - sensitivity) * axisValue + sensitivity * Math.pow(axisValue, 3);
        return (1 - sensitivity) * axisValue + sensitivity * Math.pow(axisValue, 2)*Math.signum(axisValue);

    }

    public void povActive(BionicPOV povAngle, CommandBase command) {
        BionicJoystickPOVButton newPov = new BionicJoystickPOVButton(this, povAngle.getNumber());

        newPov.whenActive(command);
    }

    /**
     * @param button      Button to Create Handler For
     * @param commandable Returns a Commandable that can be used by the operator and autonomous CommandGroups
     */
    public void buttonPressed(BionicButton button, CommandBase command) {
        JoystickButton newButton = new JoystickButton(this, button.getNumber());

        newButton.whenPressed(command);
    }

    /**
     * @param axis        Axis to Create Handler For
     * @param threshold   Minimum Threshold to Trigger Command
     * @param commandable Returns a Commandable that can be used by the operator and autonomous CommandGroups
     */
    public void buttonPressed(BionicAxis axis, double threshold, CommandBase command) {
        BionicJoystickAxisButton newButton = new BionicJoystickAxisButton(this, axis.getNumber(), threshold);

        newButton.whenActive(command);
    }
    
    /**
     * @param button      Button to Create Handler For
     * @param commandable Returns a Commandable that can be used by the operator and autonomous CommandGroups
     */
    
    public void buttonHeld(BionicButton button, CommandBase command) {
        JoystickButton newButton = new JoystickButton(this, button.getNumber());

        newButton.whenHeld(command);
    }

    /**
     * @param axis        Axis to Create Handler For
     * @param threshold   Minimum Threshold to Trigger Command
     * @param commandable Returns a Commandable that can be used by the operator and autonomous CommandGroups
     */
    public void buttonHeld(BionicAxis axis, double threshold, CommandBase command) {
        BionicJoystickAxisButton newButton = new BionicJoystickAxisButton(this, axis.getNumber(), threshold);

        newButton.whenActive(command);
    }

    /**
     * @param button      Button to Create Handler For
     * @param commandable Returns a Commandable that can be used by the operator and autonomous CommandGroups
     */
    public void buttonToggled(BionicButton button, CommandBase command) {
        JoystickButton newButton = new JoystickButton(this, button.getNumber());

        newButton.toggleWhenPressed(command);
    }

    /**
     * @param button      Button to Create Handler For
     * @param commandable Returns a Commandable that can be used by the operator and autonomous CommandGroups
     * @param interruptible If Command is Interruuptable
     */
    public void buttonToggled(BionicButton button, CommandBase command, boolean interruptible) {
        JoystickButton newButton = new JoystickButton(this, button.getNumber());

        newButton.toggleWhenPressed(command, interruptible);
    }

    /**
     * @param axis        Axis to Create Handler For
     * @param threshold   Minimum Threshold to Trigger Command
     * @param commandable Returns a Commandable that can be used by the operator and autonomous CommandGroups
     */
    public void buttonToggled(BionicAxis axis, double threshold, InstantCommand command) {
        BionicJoystickAxisButton newButton = new BionicJoystickAxisButton(this, axis.getNumber(), threshold);

        newButton.toggleWhenActive(command);
    }

    private class BionicJoystickAxisButton extends Trigger {
        private BionicJoystick inputJoystick;
        private int axisNumber;

        private double thresholdValue;

        public BionicJoystickAxisButton(BionicJoystick joystick, int axis, double minThreshold) {
            inputJoystick = joystick;

            axisNumber = axis;

            thresholdValue = minThreshold;
        }

        public boolean get() {
            return Math.abs(inputJoystick.getRawAxis(axisNumber)) > thresholdValue;
        }
    }

    private class BionicJoystickPOVButton extends Trigger {
        private BionicJoystick inputJoystick;
        private int povAngle;

        public BionicJoystickPOVButton(BionicJoystick joystick, int angle) {
            inputJoystick = joystick;

            povAngle = angle;
        }

        public boolean get() {
            return inputJoystick.getPOV() == povAngle;
        }
    }
}