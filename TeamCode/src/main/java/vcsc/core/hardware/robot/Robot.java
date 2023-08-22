package vcsc.core.hardware.robot;

import vcsc.core.assembly.Assembly;

public class Robot extends Assembly {
    static Robot instance;

    public Robot() {
        super();
        Robot.instance = this;
    }

    public static Robot getInstance() {
        if (Robot.instance == null) {
            new Robot();
        }
        return Robot.instance;
    }
}
