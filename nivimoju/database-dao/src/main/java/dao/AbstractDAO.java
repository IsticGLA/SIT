package dao;

import com.couchbase.client.deps.com.fasterxml.jackson.core.JsonProcessingException;
import com.couchbase.client.deps.com.fasterxml.jackson.databind.ObjectMapper;
import com.couchbase.client.deps.com.fasterxml.jackson.databind.ObjectWriter;
import com.couchbase.client.java.document.JsonDocument;
import com.couchbase.client.java.document.JsonLongDocument;
import com.couchbase.client.java.document.json.JsonObject;
import com.couchbase.client.java.error.FlushDisabledException;
import com.couchbase.client.java.view.*;
import entity.AbstractEntity;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Abstract class for DAO
 * AbstractDAO provides methods for DAO
 * Also use for connect and disconnect
 */
public abstract class AbstractDAO<T extends AbstractEntity> {

    /**
     * type of T
     */
    protected String type;

    protected Class<T> typeClass;

    /**
     * Connect to BDD and
     * @return Bucket to communicate with couchbase
     */
    public final void connect() {
        DAOManager.connect();
    }

    /**
     * Disconnect BDD
     */
    public final void disconnect() {
        DAOManager.disconnect();
    }

    /**
     * Create an entity
     * @param e entity to create
     */
    public final T create(T e) {
        JsonLongDocument globalId = DAOManager.getCurrentBucket().counter("globalId", 1);
        Long newId = globalId.content();
        e.setId(newId);
        JsonDocument res = DAOManager.getCurrentBucket().insert(JsonDocument.create(Long.toString(e.getId()), entityToJsonDocument(e)));

        return jsonDocumentToEntity(res.content());
    }

    /**
     * Delete an entity
     * @param e
     */
    public final long delete(T e) {
        JsonDocument res = DAOManager.getCurrentBucket().remove("" + e.getId());
        return Long.valueOf(res.id());
    }

    /**
     * Update entity
     * fails with a DocumentDoesNotExistException if the object does not exist
     * @param e
     */
    public final T update(T e) {
        JsonDocument res = DAOManager.getCurrentBucket().replace(JsonDocument.create(Long.toString(e.getId()), entityToJsonDocument(e)));
        return jsonDocumentToEntity(res.content());
    }

    /**
     * GetAll
     * @return
     */
    public final List<T> getAll()
    {

        long begin = System.currentTimeMillis();
        List<T> res = new ArrayList<T>();
        createViewAll();

        List<ViewRow> result = DAOManager.getCurrentBucket().query(ViewQuery.from("designDoc", "by_type_" + type)).allRows();

        // Iterate through the returned ViewRows
        for (ViewRow row : result) {
            //System.out.println(row.toString());
            res.add(jsonDocumentToEntity((JsonObject) row.key()));
        }
        long end = System.currentTimeMillis();
        float time = ((float) (end-begin)) / 1000f;
        System.out.println(time);
        return res;
    }

    /**
     * GetById
     * @return
     */
    public final T getById(Long id)
    {
        long begin = System.currentTimeMillis();

        String idString = Long.toString(id);
        JsonDocument res = DAOManager.getCurrentBucket().get(idString);
        if (null == res){
            return null;
        } else {
            long end = System.currentTimeMillis();
            float time = ((float) (end-begin)) / 1000f;
            System.out.println("GetById "+time);
            return jsonDocumentToEntity(res.content());
        }
    }

    /**
     * flush our bucket
     * @return
     */
    public boolean flush()
    {
        if(DAOManager.getCurrentBucket()!=null && DAOManager.currentCluster!=null)
        {
            try
            {
                return DAOManager.getCurrentBucket().bucketManager().flush();
            }
            catch (FlushDisabledException e)
            {
                e.printStackTrace();
                return false;
            }
        }
        return false;
    }

    /**
     * Transform a jsonDocument to entity
     * @param jsonDocument document to transform
     * @return entity of JsonDocument
     */
    protected T jsonDocumentToEntity(JsonObject jsonDocument){
        T entity = null;
        try {
            ObjectMapper om = new ObjectMapper();
            entity = om.readValue(jsonDocument.toString(), typeClass);
        } catch (IOException e) {
            e.printStackTrace();
        }
        return entity;
    }

    /**
     * Transform an entity to JsonDocument
     * @param entity to transform
     * @return jsonDoc of entity
     */
    protected JsonObject entityToJsonDocument(T entity){
        String json = "";
        try {
            ObjectWriter ow = new ObjectMapper().writer().withDefaultPrettyPrinter();
            json = ow.writeValueAsString(entity);
        } catch (JsonProcessingException e) {
            e.printStackTrace();
        }
        return JsonObject.fromJson(json);
    }

    public T cloneEntity(T entity) {
        JsonObject json = entityToJsonDocument(entity);
        json.put("id", entity.getId());
        return jsonDocumentToEntity(entityToJsonDocument(entity));
    }

    private void createViewAll()
    {
        DesignDocument designDoc = DAOManager.getCurrentBucket().bucketManager().getDesignDocument("designDoc");
        if (null == designDoc){
            designDoc = createDesignDocument();
        }

        String viewName = "by_type_"+type;
        String mapFunction =
                "function (doc, meta) {\n" +
                        " if(doc.type && doc.type == '"+ type + "') \n" +
                        "   { emit(doc);}\n" +
                        "}";
        designDoc.views().add(DefaultView.create(viewName, mapFunction, ""));
        DAOManager.getCurrentBucket().bucketManager().upsertDesignDocument(designDoc);
    }

    public DesignDocument createDesignDocument()
    {
        List<View> views = new ArrayList<View>();
        DesignDocument designDoc = DesignDocument.create("designDoc", views);
        DAOManager.getCurrentBucket().bucketManager().insertDesignDocument(designDoc);
        return designDoc;
    }
}
