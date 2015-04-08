package istic.gla.groupb.nivimoju.drone.latlong;

/**
 * Created by Sacapuces on 4/8/2015.
 */
public class LocalCoordinate {
    private float x;
    private float y;

    public LocalCoordinate(float x, float y){
        this.x = x;
        this.y = y;
    }

    public float getX() {
        return x;
    }

    public void setX(float x) {
        this.x = x;
    }

    public float getY() {
        return y;
    }

    public void setY(float y) {
        this.y = y;
    }
}
