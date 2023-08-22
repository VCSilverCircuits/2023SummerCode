package vcsc.core.component;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import vcsc.core.util.Console;

public interface Component {
    // Grant access to telemetry
    Console console = Console.getInstance();

    void init();

    void update();
}
