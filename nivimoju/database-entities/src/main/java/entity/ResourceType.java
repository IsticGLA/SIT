package entity;

import util.Constant;
import util.ResourceCategory;

/**
 * Created by jeremy on 08/04/15.
 */
public class ResourceType extends AbstractEntity {

    private String label;
    private ResourceCategory category;

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

    /**
     * Build a ResourceType
     * @param label
     * @param category
     */
    public ResourceType(String label, ResourceCategory category) {
        this.label = label;
        this.category = category;
    }

    public String getLabel() {
        return label;
    }

    public void setLabel(String label) {
        this.label = label;
    }

    public ResourceCategory getCategory() {
        return category;
    }

    public void setCategory(ResourceCategory category) {
        this.category = category;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof ResourceType)) return false;

        ResourceType that = (ResourceType) o;

        if (id != that.id) return false;
        if (!type.equals(that.type)) return false;
        if (category != that.category) return false;
        return label.equals(that.label);

    }

    @Override
    public int hashCode() {
        return label.hashCode();
    }
}
