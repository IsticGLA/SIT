package istic.gla.groupb.nivimoju.drone.latlong;

/**
 * Maximum value of Latitude is 90 degrees (at poles)
 * Minimum value of Latitude is 0 degree (at Equator)

 * Maximum value of Longitude is 180 degrees ( eastward from Green Witch)
 * Minimum value of Longitude is -180 degree ( westward from Green Witch)
 */
public class LatLong {
    private double latitude;
    private double longitude;

    public LatLong(double latitude, double longitude) {
        this.latitude = latitude;
        this.longitude = longitude;
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
    public String toString() {
        return "LatLong{" +
                "latitude=" + latitude +
                ", longitude=" + longitude +
                '}';
    }

    public double distFrom(LatLong target) {
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
