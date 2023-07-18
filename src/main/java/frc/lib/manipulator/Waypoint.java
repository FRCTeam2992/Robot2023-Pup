package frc.lib.manipulator;

public class Waypoint {
    private double angle;
    private OuttakeType outtake;

    public Waypoint(double angle, OuttakeType outtake) {
        this.angle = angle;
        this.outtake = outtake;
    }

    public double angle() {
        return this.angle;
    }

    public OuttakeType outtakeType() {
        return outtake;
    }

    public static enum OuttakeType {
        Unknown(-0.6, 1.0), // Not a waypoint we outtake at or unknown so use some defaults
        Assumed_Cube(-0.7, 1.0),
        Assumed_Cone(-0.5, 1.0),
        Mid_Cube(-0.8, 1.0),
        Hybrid(-0.35, 1.0),
        Rev_Mid_Throw_Cube(-.25, 1.0),
        Rear_Low_Cube(-0.4, 1.0);

        public double speed; // How fast to spin claw
        public double time; // How long to spin claw

        private OuttakeType(double speed, double time) {
            this.speed = speed;
            this.time = time;
        }
    }

}
