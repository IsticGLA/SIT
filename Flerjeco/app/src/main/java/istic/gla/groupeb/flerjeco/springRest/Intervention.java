package istic.gla.groupeb.flerjeco.springRest;

/**
 * Created by amhachi on 09/04/15.
 */
public class Intervention {
    Long id;
    String idIncidentCode;
    double latitude;
    double longitude;
    String name;

    public Intervention() {
    }

    public Intervention(String idIncidentCode, double latitude, double longitude, String name) {
        this.idIncidentCode = idIncidentCode;
        this.latitude = latitude;
        this.longitude = longitude;
        this.name = name;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public Long getId() {
        return id;
    }

    public void setId(Long id) {
        this.id = id;
    }

    public String getIdIncidentCode() {
        return idIncidentCode;
    }

    public void setIdIncidentCode(String idIncidentCode) {
        this.idIncidentCode = idIncidentCode;
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
        if (!(o instanceof Intervention)) return false;

        Intervention that = (Intervention) o;

        if (Double.compare(that.latitude, latitude) != 0) return false;
        if (Double.compare(that.longitude, longitude) != 0) return false;
        if (!id.equals(that.id)) return false;
        if (!idIncidentCode.equals(that.idIncidentCode)) return false;

        return true;
    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        result = id.hashCode();
        result = 31 * result + idIncidentCode.hashCode();
        temp = Double.doubleToLongBits(latitude);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(longitude);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    @Override
    public String toString() {
        return "Intervention{" +
                "id=" + id +
                ", idIncidentCode='" + idIncidentCode + '\'' +
                ", latitude=" + latitude +
                ", longitude=" + longitude +
                ", name='" + name + '\'' +
                '}';
    }
}
