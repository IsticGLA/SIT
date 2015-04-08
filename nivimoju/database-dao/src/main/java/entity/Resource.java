package entity;

import util.Constant;
import util.State;

import java.io.Serializable;

/**
 * Created by jeremy on 08/04/15.
 */
public class Resource extends AbstractEntity implements Serializable {

    private String label;
    private State state;
    private double latitude;
    private double longitude;

    /**
     * Build a Resource
     */
    public Resource() {
        super();
        this.type = Constant.TYPE_RESOURCE;
    }

    /**
     * Build a Resource
     * @param label
     * @param state
     * @param latitude
     * @param longitude
     */
    public Resource(String label, State state, double latitude, double longitude) {
        super();
        this.type = Constant.TYPE_RESOURCE;
        this.label = label;
        this.state = state;
        this.latitude = latitude;
        this.longitude = longitude;
    }

    public String getLabel() {
        return label;
    }

    public void setLabel(String label) {
        this.label = label;
    }

    public State getState() {
        return state;
    }

    public void setState(State state) {
        this.state = state;
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
        if (!(o instanceof Resource)) return false;

        Resource resource = (Resource) o;

        if (Double.compare(resource.latitude, latitude) != 0) return false;
        if (Double.compare(resource.longitude, longitude) != 0) return false;
        if (!label.equals(resource.label)) return false;
        return state == resource.state;

    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        result = label.hashCode();
        result = 31 * result + state.hashCode();
        temp = Double.doubleToLongBits(latitude);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(longitude);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }
}
