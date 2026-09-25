package com.qualcomm.robotcore.hardware;
/**
 * Stub HardwareMap. Returns a fresh instance of whatever stub class is asked
 * for, so subsystems can be constructed offline. Tests that need to control a
 * device register it first with {@link #put}.
 */
public class HardwareMap {
    private final java.util.Map<String, Object> devices = new java.util.HashMap<>();

    public void put(String name, Object device) { devices.put(name, device); }

    @SuppressWarnings("unchecked")
    public <T> T get(Class<? extends T> c, String name) {
        Object registered = devices.get(name);
        if (registered != null) return (T) registered;
        // DcMotor is an interface in the real SDK; hand out the stub's concrete
        // motor so subsystems can be constructed offline.
        if (c == DcMotor.class || c == DcMotorSimple.class) {
            return (T) new BasicMotor();
        }
        try {
            return c.getDeclaredConstructor().newInstance();
        } catch (Exception e) {
            return null;
        }
    }
}
