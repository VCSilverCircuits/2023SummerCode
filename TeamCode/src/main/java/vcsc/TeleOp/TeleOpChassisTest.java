package vcsc.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Chassis.DriveHardware;

@Config
@TeleOp(name = "New TeleOp", group = "GG(Get Girls)")
public class TeleOpChassisTest extends OpMode {

    //initialize all subsystems
    private DriveHardware Drive;

    @Override
    public void init() {

        Drive = new DriveHardware(hardwareMap);
        //timer for tracking button presses
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        //send telemetry to dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //wait for the driver to press start
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    //loop while program is in the int phase
    @Override
    public void init_loop() {

    }

    //while the OpMode Is Active run this code
    @Override
    public void loop() {
        Drive.FieldCentricTrig(gamepad1.left_stick_y, gamepad1.right_stick_x,
                gamepad1.left_stick_x, gamepad1.left_bumper, gamepad1.a);
    }
    @Override
    public void stop() {

    }
}
