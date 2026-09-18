package org.firstinspires.ftc.robotcore.external;
public interface Telemetry {
    Object addData(String caption, String format, Object... args);
    Object addData(String caption, Object value);
    Object addLine(String s);
    Object addLine();
    boolean update();
}
