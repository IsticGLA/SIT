package istic.gla.groupb.nivimoju.entity;

import istic.gla.groupb.nivimoju.util.ResourceRole;
import istic.gla.groupb.nivimoju.util.ResourceCategory;
import istic.gla.groupb.nivimoju.util.State;

import java.io.Serializable;
import java.sql.Timestamp;
import java.util.Calendar;

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
    private Timestamp plannedHistory;
    private Timestamp arrivedHistory;
    private Timestamp activeHistory;
    private Timestamp waitingHistory;
    private Timestamp validatedHistory;
    private Timestamp refusedHistory;
    private Timestamp freeHistory;

    /**
     * Build a Resource
     */
    public Resource() {
        super();
        this.resourceRole = ResourceRole.otherwise;
        this.resourceCategory = ResourceCategory.vehicule;
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
        this.setState(state);
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
        this.setState(state);
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
    public Resource(Long id, String label, State state) {
        super();
        this.idRes = id;
        this.label = label;
        this.setState(state);
        this.resourceRole = ResourceRole.otherwise;
        this.resourceCategory = ResourceCategory.vehicule;
    }

    public void initState() {
        if (this.state == State.validated) {
            Timestamp now = new Timestamp(Calendar.getInstance().getTime().getTime());
            this.validatedHistory = now;
            this.waitingHistory = now;
        }
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
        Timestamp now = new Timestamp(Calendar.getInstance().getTime().getTime());
        switch (state) {
            case active:
                if(activeHistory == null)
                    this.activeHistory = now;
                break;
            case arrived:
                if(arrivedHistory == null)
                    this.arrivedHistory = now;
                break;
            case free:
                if(freeHistory == null)
                    this.freeHistory = now;
                break;
            case planned:
                if(plannedHistory == null)
                    this.plannedHistory = now;
                break;
            case refused:
                if(refusedHistory == null)
                    this.refusedHistory = now;
                break;
            case validated:
                if(validatedHistory == null)
                    this.validatedHistory = now;
                break;
            case waiting:
                if(waitingHistory == null)
                    this.waitingHistory = now;
                break;
            default:
                break;
        }
    }

    public Timestamp getStateDate(State state) {
        switch (state) {
            case active:
                return this.activeHistory;
            case arrived:
                return this.arrivedHistory;
            case free:
                return this.freeHistory;
            case planned:
                return this.plannedHistory;
            case refused:
                return this.refusedHistory;
            case validated:
                return this.validatedHistory;
            case waiting:
                return this.waitingHistory;
            default:
                return null;
        }
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

    public Timestamp getPlannedHistory() {
        return plannedHistory;
    }

    public Timestamp getArrivedHistory() {
        return arrivedHistory;
    }

    public Timestamp getActiveHistory() {
        return activeHistory;
    }

    public Timestamp getWaitingHistory() {
        return waitingHistory;
    }

    public Timestamp getValidatedHistory() {
        return validatedHistory;
    }

    public Timestamp getRefusedHistory() {
        return refusedHistory;
    }

    public Timestamp getFreeHistory() {
        return freeHistory;
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
