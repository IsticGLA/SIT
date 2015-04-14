package entity;


import java.io.Serializable;

/**
 * Created by arno on 09/03/15.
 */

public class Position implements Serializable {

    private double longitude;
    private double latitude;
    private double altitude = Double.NaN;

    public Position() {
    }

    public Position(double latitude, double longitude) {
        this.longitude = longitude;
        this.latitude = latitude;
    }

    public Position(double latitude, double longitude, double altitude) {
        this.longitude = longitude;
        this.latitude = latitude;
        this.altitude = altitude;
    }

    public boolean hasAltitude() {
        return !Double.isNaN(altitude);
    }

    public double getLongitude() {
        return longitude;
    }

    public void setLongitude(double longitude) {
        this.longitude = longitude;
    }

    public double getLatitude() {
        return latitude;
    }

    public void setLatitude(double latitude) {
        this.latitude = latitude;
    }

    public double getAltitude() {
        return altitude;
    }

    public void setAltitude(double altitude) {
        this.altitude = altitude;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (!(o instanceof Position)) {
            return false;
        }
        Position lngLatAlt = (Position) o;
        return Double.compare(lngLatAlt.latitude, latitude) == 0 && Double.compare(lngLatAlt.longitude, longitude) == 0
                && Double.compare(lngLatAlt.altitude, altitude) == 0;
    }

    @Override
    public int hashCode() {
        long temp = Double.doubleToLongBits(longitude);
        int result = (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(latitude);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(altitude);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    @Override
    public String toString() {
        return "LngLatAlt{" + "longitude=" + longitude + ", latitude=" + latitude + ", altitude=" + altitude + '}';
    }

    public double distFrom(Position target) {
        double earthRadius = 6371000.0; // meters
        double dLat = Math.toRadians(target.getLatitude()-this.getLatitude());
        double dLng = Math.toRadians(target.getLongitude()-this.getLongitude());
        double sindLat = Math.sin(dLat / 2);
        double sindLng = Math.sin(dLng / 2);
        double a = Math.pow(sindLat, 2) + Math.pow(sindLng, 2)
                * Math.cos(Math.toRadians(this.getLatitude())) * Math.cos(Math.toRadians(target.getLatitude()));
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
        double dist = earthRadius * c;

        return dist;
    }
}

