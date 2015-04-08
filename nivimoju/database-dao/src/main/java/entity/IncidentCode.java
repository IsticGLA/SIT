package entity;

import util.Constant;

import java.util.List;

/**
 * Created by jeremy on 08/04/15.
 */
public class IncidentCode extends AbstractEntity {

    private String code;
    private List<ResourceType> resourceTypeList;

    /**
     * Build a IncidentCode
     */
    public IncidentCode() {
        super();
        this.type = Constant.TYPE_INCIDENT_CODE;
    }

    /**
     * Build a IncidentCode
     * @param code
     * @param resourceTypeList
     */
    public IncidentCode(String code, List<ResourceType> resourceTypeList) {
        super();
        this.type = Constant.TYPE_INCIDENT_CODE;
        this.code = code;
        this.resourceTypeList = resourceTypeList;
    }

    public String getCode() {
        return code;
    }

    public void setCode(String code) {
        this.code = code;
    }

    public List<ResourceType> getResourceTypeList() {
        return resourceTypeList;
    }

    public void setResourceTypeList(List<ResourceType> resourceTypeList) {
        this.resourceTypeList = resourceTypeList;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof IncidentCode)) return false;

        IncidentCode that = (IncidentCode) o;

        if (!code.equals(that.code)) return false;
        return resourceTypeList.equals(that.resourceTypeList);

    }

    @Override
    public int hashCode() {
        int result = code.hashCode();
        result = 31 * result + resourceTypeList.hashCode();
        return result;
    }
}
