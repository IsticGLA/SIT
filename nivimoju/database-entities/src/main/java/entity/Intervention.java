package entity;


import util.Constant;

import java.io.Serializable;
import java.util.List;

/**
 * Created by arno on 09/03/15.
 */
public class Intervention extends AbstractEntity implements Serializable {

    private int incidentCode;
    private double latitude;
    private double longitude;
    private List<Resource> resources;
    private List<List<Position>> exclusionArea;
    private List<List<Position>> watchArea;
    private List<List<Position>> watchPath;
    private List<Marker> markers;

    /**
     * Build an Intervention
     */
    public Intervention() {
        super();
        this.type = Constant.TYPE_INTERVENTION;
    }

    /**
     * Build an Intervenntion
     * @param incidentCode
     * @param latitude
     * @param longitude
     * @param resources
     * @param exclusionArea
     * @param watchArea
     * @param watchPath
     * @param markers
     */
    public Intervention(int incidentCode, double latitude, double longitude,
                        List<Resource> resources, List<List<Position>> exclusionArea,
                        List<List<Position>> watchArea, List<List<Position>> watchPath,
                        List<Marker> markers) {
        super();
        this.type = Constant.TYPE_INTERVENTION;
        this.incidentCode = incidentCode;
        this.latitude = latitude;
        this.longitude = longitude;
        this.resources = resources;
        this.exclusionArea = exclusionArea;
        this.watchArea = watchArea;
        this.watchPath = watchPath;
        this.markers = markers;
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

    public List<List<Position>> getWatchPath() {
        return watchPath;
    }

    public void setWatchPath(List<List<Position>> watchPath) {
        this.watchPath = watchPath;
    }

    public List<Marker> getMarkers() {
        return markers;
    }

    public void setMarkers(List<Marker> markers) {
        this.markers = markers;
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
        if (!watchPath.equals(that.watchPath)) return false;
        return markers.equals(that.markers);

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
        result = 31 * result + markers.hashCode();
        return result;
    }
}


