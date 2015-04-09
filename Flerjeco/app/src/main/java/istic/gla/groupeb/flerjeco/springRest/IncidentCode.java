package istic.gla.groupeb.flerjeco.springRest;

import java.util.List;

/**
 * Created by amhachi on 08/04/15.
 */
public class IncidentCode {

    String code;
    private List<ResourceType> resourceTypeList;

    public IncidentCode() {
    }


    public IncidentCode(String code) {
        this.code = code;
    }

    public String getCode() {
        return code;
    }

    public void setCode(String code) {
        this.code = code;
    }


    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof IncidentCode)) return false;

        IncidentCode that = (IncidentCode) o;

        if (!code.equals(that.code)) return false;
        if (!resourceTypeList.equals(that.resourceTypeList)) return false;

        return true;
    }

    @Override
    public int hashCode() {
        int result = code.hashCode();
        result = 31 * result + resourceTypeList.hashCode();
        return result;
    }

    public List<ResourceType> getResourceTypeList() {

        return resourceTypeList;
    }

    public void setResourceTypeList(List<ResourceType> resourceTypeList) {
        this.resourceTypeList = resourceTypeList;
    }
}
