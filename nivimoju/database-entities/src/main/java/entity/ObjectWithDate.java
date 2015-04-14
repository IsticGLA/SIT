package entity;

import java.io.Serializable;
import java.sql.Timestamp;

/**
 * Created by corentin on 14/04/15.
 */
public class ObjectWithDate implements Serializable {
    private Object object;
    private Timestamp date;

    public ObjectWithDate(Object object, Timestamp date) {
        this.object = object;
        this.date = date;
    }

    public Object getObject() {
        return object;
    }

    public void setObject(Object object) {
        this.object = object;
    }

    public Timestamp getDate() {
        return date;
    }

    public void setDate(Timestamp date) {
        this.date = date;
    }
}
