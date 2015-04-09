package istic.gla.groupb.nivimoju.drone.latlong;

/**
 * Maximum value of Latitude is 90 degrees (at poles)
 * Minimum value of Latitude is 0 degree (at Equator)

 * Maximum value of Longitude is 180 degrees ( eastward from Green Witch)
 * Minimum value of Longitude is -180 degree ( westward from Green Witch)
 */
public class LatLong {
    private float latitude;
    private float longitude;

    public LatLong(float latitude, float longitude) {
        this.latitude = latitude;
        this.longitude = longitude;
    }

    public float getLatitude() {
        return latitude;
    }

    public void setLatitude(float latitude) {
        this.latitude = latitude;
    }

    public float getLongitude() {
        return longitude;
    }

    public void setLongitude(float longitude) {
        this.longitude = longitude;
    }
}
