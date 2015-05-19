package istic.gla.groupb.nivimoju.entity;

import istic.gla.groupb.nivimoju.util.Constant;

/**
 * Created by jeremy on 14/04/15.
 */
public class Drone extends AbstractEntity {

    private String label;
    private double latitude;
    private double longitude;
    private long idIntervention;

    /**
     * Build a Drone
     */
    public Drone() {
        super();
        this.type = Constant.TYPE_DRONE;
        this.idIntervention = -1;
    }

    /**
     * Build a Drone
     *
     * @param label
     */
    public Drone(String label) {
        super();
        this.type = Constant.TYPE_DRONE;
        this.label = label;
        this.idIntervention = -1;
    }

    /**
     * Build a Drone
     *
     * @param label
     * @param latitude
     * @param longitude
     * @param idIntervention
     */
    public Drone(String label, double latitude, double longitude, long idIntervention) {
        super();
        this.type = Constant.TYPE_DRONE;
        this.label = label;
        this.latitude = latitude;
        this.longitude = longitude;
        this.idIntervention = idIntervention;
    }

    public String getLabel() {
        return label;
    }

    public void setLabel(String label) {
        this.label = label;
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

    public long getIdIntervention() {
        return idIntervention;
    }

    public void setIdIntervention(long idIntervention) {
        this.idIntervention = idIntervention;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Drone)) return false;

        Drone drone = (Drone) o;

        if (Double.compare(drone.latitude, latitude) != 0) return false;
        if (Double.compare(drone.longitude, longitude) != 0) return false;
        if (idIntervention != drone.idIntervention) return false;
        return label.equals(drone.label);

    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        result = label.hashCode();
        temp = Double.doubleToLongBits(latitude);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(longitude);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        result = 31 * result + (int) (idIntervention ^ (idIntervention >>> 32));
        return result;
    }

    @Override
    public String toString() {
        return "Drone{" +
                "label='" + label + '\'' +
                ", latitude=" + latitude +
                ", longitude=" + longitude +
                ", idIntervention=" + idIntervention +
                '}';
    }
}
