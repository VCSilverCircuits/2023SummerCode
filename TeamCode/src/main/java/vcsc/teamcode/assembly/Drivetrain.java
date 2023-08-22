package vcsc.teamcode.assembly;

import com.qualcomm.robotcore.hardware.IMU;

import java.util.List;

import vcsc.core.assembly.Assembly;
import vcsc.core.hardware.motor.CogMotor;
import vcsc.teamcode.Chassis.DriveConstants;

public class Drivetrain extends Assembly {
    private final CogMotor[] motors;
    private final IMU imu;

    private final IMU.parameters IMU_PARAMETERS = new IMU.parameters(DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR);


    public enum Wheel {
        FRONT_LEFT(0),
        FRONT_RIGHT(1),
        REAR_LEFT(2),
        REAR_RIGHT(3)

        public final int index;

        Wheel(int index) {
            this.index = index;
        }
    }

    public Drivetrain(CogMotor[] motors, IMU imu) {
        this.motors = motors;
        this.imu = imu;
        imu.initialize(IMU_PARAMETERS);
        for (CogMotor motor : motors) {
            addComponent(motor);
        }
    }

    public Drivetrain(CogMotor frontLeft, CogMotor frontRight, CogMotor rearLeft, CogMotor rearRight, IMU imu) {
        new Drivetrain(new CogMotor[]{frontLeft, frontRight, rearLeft, rearRight}, imu);
    }

    @Override
    public void init() {
        super.init();
    }

    public void setReversed(Wheel... wheels) {
        for (Wheel wheel : wheels) {
            motors.get(wheel.index).setReversed(true);
        }
    }

    /**
     * Get the component at the specified index.
     *
     * @param index The index of the component.
     * @return Component at the specified index.
     */
    @Override
    public CogMotor getComponentByIndex(int index) {
        return null;
    }

    /**
     * Add a component to this group.
     *
     * @param component The component to add.
     */
    @Override
    public void addComponent(CogMotor component) {

    }

    /**
     * Add multiple components to this group.
     *
     * @param components The components to add.
     */
    @Override
    public void addComponents(CogMotor... components) {

    }

    /**
     * Get the list of components in this group.
     *
     * @return List<Component>
     */
    @Override
    public List<CogMotor> getComponents() {
        return null;
    }

    /**
     * Update this component group for the current iteration.
     */
    @Override
    public void update() {

    }
}
