package org.firstinspires.ftc.teamcode.utils;

public enum BotState {
    HOME("HOME", 800, 0, 0.4, 0),
    LOWCHAMBER("LOWCHAMBER", 0, 0, 0, 1),
    HIGHCHAMBER("HIGHCHAMBER", 0, 0, 0, 2),
    LOWBUCKET("LOWBUCKET", 0, 0, 0, 3),
    HIGHBUCKET("HIGHBUCKET", 1400, 2500, 0.5, 4),
    PREP_CLIMB("PREP_CLIMB", 0, 0, 0, 5),
    CLIMB("CLIMB", 0, 0, 0, 6),
    PRE_INTAKE("PRE_INTAKE", -200, 500, 0.0, 10),
    INTAKE_CLOSE("INTAKE_CLOSE", -240, 1000, 0.0, 7),
    INTAKE_FAR("INTAKE_FAR", -240, 2000, 0.0, 8),
    PICKUP_SPEC("PICKUP_SPEC", 0, 0, 0, 9);

    private final String name;
    private final int shoulderSetpoint;
    private final int extensionSetpoint;
    private final double elbowSetpoint;
    private final int id;

    BotState(String name, int shoulderSetpoint, int extensionSetpoint, double elbowSetpoint, int id) {
        this.name = name;
        this.shoulderSetpoint = shoulderSetpoint;
        this.extensionSetpoint = extensionSetpoint;
        this.elbowSetpoint = elbowSetpoint;
        this.id = id;
    }

    public String getName() {
        return name;
    }

    public int getShoulderSetpoint() {
        return shoulderSetpoint;
    }

    public int getExtensionSetpoint() {
        return extensionSetpoint;
    }

    public double getElbowSetpoint() {
        return elbowSetpoint;
    }

    public int getId() {
        return id;
    }
}
