package entity;

import java.sql.Date;
import java.sql.Timestamp;
import java.util.Calendar;

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
     * Timestamp
     */
    protected Timestamp lastUpdate;

    /**
     * Basic contruct, assign a random id
     */
    public AbstractEntity()
    {
        id = -1;
    }

    /**
     * Update the lastUpdate timestamp to current timestamp
     */
    public void updateDate() {
        this.lastUpdate = new Timestamp(Calendar.getInstance().getTime().getTime());
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
