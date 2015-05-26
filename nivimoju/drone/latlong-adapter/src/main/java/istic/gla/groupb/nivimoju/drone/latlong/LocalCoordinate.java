package istic.gla.groupb.nivimoju.drone.latlong;

/**
 * A coordinate in local (blender)
 */
public class LocalCoordinate {
    private double x;
    private double y;
    private double z;

    public LocalCoordinate(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public LocalCoordinate(double x, double y){
        this.x = x;
        this.y = y;
        this.z = 0;
    }

    public LocalCoordinate(){
        this.x = 0;
        this.y = 0;
        this.z = 0;
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

    public double getZ() {
        return z;
    }

    public void setZ(double z) {
        this.z = z;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        LocalCoordinate that = (LocalCoordinate) o;

        if (Double.compare(that.x, x) != 0) return false;
        if (Double.compare(that.y, y) != 0) return false;
        return Double.compare(that.z, z) == 0;

    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        temp = Double.doubleToLongBits(x);
        result = (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(y);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(z);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    @Override
    public String toString() {
        return "[" +
                "x=" + (int)x +
                ", y=" + (int)y +
                ", z=" + (int)z +
                ']';
    }

    public double distanceInPlan(LocalCoordinate target){
        return Math.sqrt(Math.pow(this.getX()-target.getX(), 2)
        + Math.pow(this.getY()-target.getY(), 2));
    }
}
