package entity;

import java.util.Random;

/**
 * Created by corentin on 10/03/15.
 */
public abstract class AbstractEntity {
    /**
     * Unique id of entity
     */
    protected long id;

    /**
     * Type of entity
     */
    protected String type;

    /**
     * Basic contruct, assign a random id
     */
    public AbstractEntity()
    {
        id = -1;
    }

    public long getId() {
        return id;
    }

    public void setId(long id)
    {
        this.id=id;
    }

    public String getType() {
        return type;
    }
}
