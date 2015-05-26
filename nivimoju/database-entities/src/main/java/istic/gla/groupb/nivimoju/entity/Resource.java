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
        this.resourceRole = roleByName(label);
        this.resourceCategory = ResourceCategory.vehicule;
    }

    public void initState() {
        if (this.state == State.validated) {
            Timestamp now = new Timestamp(Calendar.getInstance().getTime().getTime());
            this.validatedHistory = now;
            this.waitingHistory = now;
        }
    }

    private ResourceRole roleByName(String name) {
        if(name.contains("VSAV") || name.contains("VSR")) {
            return ResourceRole.people;
        } else if(name.contains("FPT") || name.contains("EPA")){
            return ResourceRole.fire;
        } else if (name.contains("VLCG")) {
            return ResourceRole.commands;
        } else {
            return ResourceRole.otherwise;
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
    }

    public void setStateDate(State state) {
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
        if (o == null || getClass() != o.getClass()) return false;

        Resource resource = (Resource) o;

        if (idRes != resource.idRes) return false;
        if (Double.compare(resource.latitude, latitude) != 0) return false;
        if (Double.compare(resource.longitude, longitude) != 0) return false;
        if (label != null ? !label.equals(resource.label) : resource.label != null) return false;
        if (state != resource.state) return false;
        if (resourceRole != resource.resourceRole) return false;
        if (resourceCategory != resource.resourceCategory) return false;
        if (plannedHistory != null ? !plannedHistory.equals(resource.plannedHistory) : resource.plannedHistory != null)
            return false;
        if (arrivedHistory != null ? !arrivedHistory.equals(resource.arrivedHistory) : resource.arrivedHistory != null)
            return false;
        if (activeHistory != null ? !activeHistory.equals(resource.activeHistory) : resource.activeHistory != null)
            return false;
        if (waitingHistory != null ? !waitingHistory.equals(resource.waitingHistory) : resource.waitingHistory != null)
            return false;
        if (validatedHistory != null ? !validatedHistory.equals(resource.validatedHistory) : resource.validatedHistory != null)
            return false;
        if (refusedHistory != null ? !refusedHistory.equals(resource.refusedHistory) : resource.refusedHistory != null)
            return false;
        return !(freeHistory != null ? !freeHistory.equals(resource.freeHistory) : resource.freeHistory != null);

    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        result = (int) (idRes ^ (idRes >>> 32));
        result = 31 * result + (label != null ? label.hashCode() : 0);
        result = 31 * result + (state != null ? state.hashCode() : 0);
        result = 31 * result + (resourceRole != null ? resourceRole.hashCode() : 0);
        result = 31 * result + (resourceCategory != null ? resourceCategory.hashCode() : 0);
        temp = Double.doubleToLongBits(latitude);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(longitude);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        result = 31 * result + (plannedHistory != null ? plannedHistory.hashCode() : 0);
        result = 31 * result + (arrivedHistory != null ? arrivedHistory.hashCode() : 0);
        result = 31 * result + (activeHistory != null ? activeHistory.hashCode() : 0);
        result = 31 * result + (waitingHistory != null ? waitingHistory.hashCode() : 0);
        result = 31 * result + (validatedHistory != null ? validatedHistory.hashCode() : 0);
        result = 31 * result + (refusedHistory != null ? refusedHistory.hashCode() : 0);
        result = 31 * result + (freeHistory != null ? freeHistory.hashCode() : 0);
        return result;
    }
}
