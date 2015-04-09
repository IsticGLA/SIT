package entity;

import util.Constant;

/**
 * Created by jeremy on 08/04/15.
 */
public class ResourceType extends AbstractEntity {

    private String label;

    /**
     * Build a ResourceType
     */
    public ResourceType() {
        super();
        this.type = Constant.TYPE_RESOURCE_TYPE;
    }

    /**
     * Build a ResourceType
     * @param label
     */
    public ResourceType(String label) {
        super();
        this.type = Constant.TYPE_RESOURCE_TYPE;
        this.label = label;
    }

    public String getLabel() {
        return label;
    }

    public void setLabel(String label) {
        this.label = label;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof ResourceType)) return false;

        ResourceType that = (ResourceType) o;

        return label.equals(that.label);

    }

    @Override
    public int hashCode() {
        return label.hashCode();
    }
}
