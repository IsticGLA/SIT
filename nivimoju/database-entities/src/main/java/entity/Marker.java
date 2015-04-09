package entity;

import java.io.Serializable;

/**
 * Created by jeremy on 08/04/15.
 */
public class Marker implements Serializable {

    private MarkerType markerType;
    private double latitude;
    private double longitude;

    /**
     * Build a Marker
     */
    public Marker() {
        super();
    }

    /**
     * Build a Marker
     * @param markerType
     * @param latitude
     * @param longitude
     */
    public Marker(MarkerType markerType, double latitude, double longitude) {
        super();
        this.markerType = markerType;
        this.latitude = latitude;
        this.longitude = longitude;
    }

    public MarkerType getMarkerType() {
        return markerType;
    }

    public void setMarkerType(MarkerType markerType) {
        this.markerType = markerType;
    }

    public double getLatitude() {
        return latitude;
    }

    public void setLatitude(double latitude) {
        this.latitude = latitude;
    }

    public double getLongitude() {
        return longitude;
    }

    public void setLongitude(double longitude) {
        this.longitude = longitude;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Marker)) return false;

        Marker marker = (Marker) o;

        if (Double.compare(marker.latitude, latitude) != 0) return false;
        if (Double.compare(marker.longitude, longitude) != 0) return false;
        return markerType == marker.markerType;

    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        result = markerType.hashCode();
        temp = Double.doubleToLongBits(latitude);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(longitude);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }
}
