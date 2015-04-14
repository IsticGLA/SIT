package entity;

import util.ResourceRole;
import util.ResourceCategory;
import util.State;

import java.io.Serializable;

/**
 * Created by jeremy on 08/04/15.
 */
public class Resource implements Serializable {

    private long idRes;
    private String label;
    private State state;
    private ResourceRole resourceRole;
    private ResourceCategory resourceCategory;
    private double latitude;
    private double longitude;

    /**
     * Build a Resource
     */
    public Resource() {
        super();
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
        this.label = label;
        this.state = state;
        this.resourceRole = ResourceRole.otherwise;
        this.resourceCategory = ResourceCategory.vehicule;
        this.latitude = latitude;
        this.longitude = longitude;
    }

    /**
     * Build a resource
     * @param label
     * @param state
     * @param resourceRole
     * @param resourceCategory
     * @param latitude
     * @param longitude
     */
    public Resource(String label, State state, ResourceRole resourceRole, ResourceCategory resourceCategory, double latitude, double longitude) {
        super();
        this.label = label;
        this.state = state;
        this.resourceRole = resourceRole;
        this.resourceCategory = resourceCategory;
        this.latitude = latitude;
        this.longitude = longitude;
    }

    /**
     * Build a Resource
     * @param label
     * @param state
     */
    public Resource(String label, State state) {
        super();
        this.label = label;
        this.state = state;
        this.resourceRole = ResourceRole.otherwise;
        this.resourceCategory = ResourceCategory.vehicule;
    }

    public long getIdRes() {
        return idRes;
    }

    public void setIdRes(long idRes) {
        this.idRes = idRes;
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

    public ResourceCategory getResourceCategory() {
        return resourceCategory;
    }

    public void setResourceCategory(ResourceCategory resourceCategory) {
        this.resourceCategory = resourceCategory;
    }

    public ResourceRole getResourceRole() {
        return resourceRole;
    }

    public void setResourceRole(ResourceRole resourceRole) {
        this.resourceRole = resourceRole;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Resource)) return false;

        Resource resource = (Resource) o;

        if (Double.compare(resource.latitude, latitude) != 0) return false;
        if (Double.compare(resource.longitude, longitude) != 0) return false;
        if (!label.equals(resource.label)) return false;
        if (state != resource.state) return false;
        if (resourceRole != resource.resourceRole) return false;
        return resourceCategory == resource.resourceCategory;

    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        result = label.hashCode();
        result = 31 * result + state.hashCode();
        result = 31 * result + resourceRole.hashCode();
        result = 31 * result + resourceCategory.hashCode();
        temp = Double.doubleToLongBits(latitude);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(longitude);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }
}
