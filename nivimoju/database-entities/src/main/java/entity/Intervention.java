package entity;


import util.Constant;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by arno on 09/03/15.
 */
public class Intervention extends AbstractEntity implements Serializable {

    private String name;
    private int incidentCode;
    private double latitude;
    private double longitude;
    private List<Resource> resources;
    private List<List<Position>> exclusionArea;
    private List<List<Position>> watchArea;
    private List<Path> watchPath;

    /**
     * Build an Intervention
     */
    public Intervention() {
        super();
        this.type = Constant.TYPE_INTERVENTION;
        this.resources = new ArrayList<>();
        this.exclusionArea = new ArrayList<>();
        this.watchArea = new ArrayList<>();
        this.watchPath = new ArrayList<>();
    }

    /**
     * Build an Intervenntion
     * @param incidentCode
     * @param latitude
     * @param longitude
     */
    public Intervention(String name, int incidentCode, double latitude, double longitude) {
        super();
        this.type = Constant.TYPE_INTERVENTION;
        this.name = name;
        this.incidentCode = incidentCode;
        this.latitude = latitude;
        this.longitude = longitude;
        this.resources = new ArrayList<>();
        this.exclusionArea = new ArrayList<>();
        this.watchArea = new ArrayList<>();
        this.watchPath = new ArrayList<>();
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public int getIncidentCode() {
        return incidentCode;
    }

    public void setIncidentCode(int incidentCode) {
        this.incidentCode = incidentCode;
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

    public List<Resource> getResources() {
        return resources;
    }

    public void setResources(List<Resource> resources) {
        this.resources = resources;
    }

    public List<List<Position>> getExclusionArea() {
        return exclusionArea;
    }

    public void setExclusionArea(List<List<Position>> exclusionArea) {
        this.exclusionArea = exclusionArea;
    }

    public List<List<Position>> getWatchArea() {
        return watchArea;
    }

    public void setWatchArea(List<List<Position>> watchArea) {
        this.watchArea = watchArea;
    }

    public List<Path> getWatchPath() {
        return watchPath;
    }

    public void setWatchPath(List<Path> watchPath) {
        this.watchPath = watchPath;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Intervention)) return false;

        Intervention that = (Intervention) o;

        if (incidentCode != that.incidentCode) return false;
        if (Double.compare(that.latitude, latitude) != 0) return false;
        if (Double.compare(that.longitude, longitude) != 0) return false;
        if (!resources.equals(that.resources)) return false;
        if (!exclusionArea.equals(that.exclusionArea)) return false;
        if (!watchArea.equals(that.watchArea)) return false;
        return watchPath.equals(that.watchPath);

    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        result = incidentCode;
        temp = Double.doubleToLongBits(latitude);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(longitude);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        result = 31 * result + resources.hashCode();
        result = 31 * result + exclusionArea.hashCode();
        result = 31 * result + watchArea.hashCode();
        result = 31 * result + watchPath.hashCode();
        return result;
    }
}


