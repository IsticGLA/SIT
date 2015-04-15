package istic.gla.groupb.nivimoju.drone.client;

/**
 * Created by sacapuces on 15/04/15.
 */
public class DroneInfo {
    private String label;
    private long x;
    private long y;
    private long z;

    public long getX() {
        return x;
    }

    public void setX(long x) {
        this.x = x;
    }

    public long getY() {
        return y;
    }

    public void setY(long y) {
        this.y = y;
    }

    public long getZ() {
        return z;
    }

    public void setZ(long z) {
        this.z = z;
    }

    public String getLabel() {

        return label;
    }

    public void setLabel(String label) {
        this.label = label;
    }

    @Override
    public String toString() {
        return "DroneInfo{" +
                "label='" + label + '\'' +
                ", x=" + x +
                ", y=" + y +
                ", z=" + z +
                '}';
    }
}
