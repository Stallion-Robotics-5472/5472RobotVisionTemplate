/*
 * Convenience base class for subsystems. Modeled on WPILib's SubsystemBase.
 *
 * Unlike WPILib's version this does NOT self-register with a static scheduler:
 * an FTC app runs many OpModes in one process, so a static registry would carry
 * subsystems (and their stale hardware handles) from one run into the next. The
 * OpMode registers its subsystems explicitly instead -- see CommandOpMode.
 */
package org.firstinspires.ftc.teamcode.lib.command;

public abstract class SubsystemBase implements Subsystem {
    private final String name;

    protected SubsystemBase() {
        this.name = getClass().getSimpleName();
    }

    protected SubsystemBase(String name) {
        this.name = name;
    }

    @Override
    public String getName() {
        return name;
    }
}
