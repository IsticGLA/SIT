package istic.gla.groupeb.flerjeco.springRest;

/**
 * Created by jules on 09/04/15.
 */
public class ResourceType {

    private Long id;
    private String label;

    public ResourceType() { }

    public ResourceType(Long id, String label) {
        this.id = id;
        this.label = label;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        ResourceType that = (ResourceType) o;

        if (label != null ? !label.equals(that.label) : that.label != null) return false;
        if (id != null ? !id.equals(that.id) : that.id != null) return false;

        return true;
    }

    @Override
    public int hashCode() {
        int result = id != null ? id.hashCode() : 0;
        result = 31 * result + (label != null ? label.hashCode() : 0);
        return result;
    }

    @Override
    public String toString() {
        return "ResourceType{" +
                "id=" + id +
                ", label='" + label + '\'' +
                '}';
    }

    public Long getId() {
        return id;
    }

    public void setId(Long id) {
        this.id = id;
    }

    public String getLabel() {
        return label;
    }

    public void setLabel(String label) {
        this.label = label;
    }
}
