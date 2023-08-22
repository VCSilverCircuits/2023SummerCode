package vcsc.teamcode.Chassis;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/*
    class that holds hardware values for odo pods
 */
@Config
public class DriveConstants {

    public static final double TICKS_PER_REV = 28;//tics of base motor
    public static final double MAX_RPM = 6000; //max rpm duh


    public static double WHEEL_RADIUS = 1.8898;//robots wheel radius in inches
    public static double GEAR_RATIO = 0.059;//output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 9.5;//distance between parallel odo pods in inches

    //needed cus we are using roadrunner. for trajectory fallowing (not used)
    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0;
    public static double kStatic = 0;

    //imu parameters
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

}