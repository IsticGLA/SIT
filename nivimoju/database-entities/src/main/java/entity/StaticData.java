package entity;

import util.Constant;
import util.MarkerType;

import java.io.Serializable;

/**
 * Created by jeremy on 08/04/15.
 */
public class StaticData extends AbstractEntity implements Serializable {

    private double latitude;
    private double longitude;
    private MarkerType markerType;

    /**
     * Build a StaticData
     */
    public StaticData() {
        super();
        this.type = Constant.TYPE_STATIC_DATA;
    }

    /**
     * Build a StaticData
     * @param latitude
     * @param longitude
     * @param markerType
     */
    public StaticData(double latitude, double longitude, MarkerType markerType) {
        super();
        this.type = Constant.TYPE_STATIC_DATA;
        this.latitude = latitude;
        this.longitude = longitude;
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

    public MarkerType getMarkerType() {
        return markerType;
    }

    public void setMarkerType(MarkerType markerType) {
        this.markerType = markerType;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof StaticData)) return false;

        StaticData that = (StaticData) o;
        if (that.id != id) return false;
        if (!that.type.equals(type)) return false;
        if (Double.compare(that.latitude, latitude) != 0) return false;
        if (Double.compare(that.longitude, longitude) != 0) return false;
        return markerType.equals(that.markerType);

    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        temp = Double.doubleToLongBits(latitude);
        result = (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(longitude);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        result = 31 * result + markerType.hashCode();
        return result;
    }
}
