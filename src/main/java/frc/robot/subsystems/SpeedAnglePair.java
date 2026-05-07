package frc.robot.subsystems;

// used to be vector2, can still be used as vector2, but is not currently
public class SpeedAnglePair {

    double x;
    double y;

    public SpeedAnglePair(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof SpeedAnglePair)) {
            return false;
        }

        SpeedAnglePair that = (SpeedAnglePair) obj;
        if (x != that.x) return false;
        if (y != that.y) return false;

        return true;
    }

    @Override
    public int hashCode() {
        int hc = 1337;
        hc *= 7 + Double.valueOf(x).hashCode();
        hc *= 11 + Double.valueOf(y).hashCode();

        return hc;
    }

    @Override
    public String toString() {
        return "SpeedAnglePair(" + x + "," + y + ")";
    }
}
