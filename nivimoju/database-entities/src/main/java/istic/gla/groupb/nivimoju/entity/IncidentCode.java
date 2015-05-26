package istic.gla.groupb.nivimoju.entity;

import istic.gla.groupb.nivimoju.util.Constant;

import java.util.List;

/**
 * Created by jeremy on 08/04/15.
 */
public class IncidentCode extends AbstractEntity {

    private String code;
    private List<Long> resourceType;

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
     * @param resourceType
     */
    public IncidentCode(String code, List<Long> resourceType) {
        super();
        this.type = Constant.TYPE_INCIDENT_CODE;
        this.code = code;
        this.resourceType = resourceType;
    }

    public String getCode() {
        return code;
    }

    public void setCode(String code) {
        this.code = code;
    }

    public List<Long> getresourceType() {
        return resourceType;
    }

    public void setresourceType(List<Long> resourceType) {
        this.resourceType = resourceType;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof IncidentCode)) return false;

        IncidentCode that = (IncidentCode) o;

        if (!code.equals(that.code)) return false;
        return resourceType.equals(that.resourceType);

    }

    @Override
    public int hashCode() {
        int result = code.hashCode();
        result = 31 * result + resourceType.hashCode();
        return result;
    }
}
