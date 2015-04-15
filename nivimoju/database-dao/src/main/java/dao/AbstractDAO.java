package dao;

import com.couchbase.client.deps.com.fasterxml.jackson.core.JsonProcessingException;
import com.couchbase.client.deps.com.fasterxml.jackson.databind.ObjectMapper;
import com.couchbase.client.deps.com.fasterxml.jackson.databind.ObjectWriter;
import com.couchbase.client.java.document.JsonDocument;
import com.couchbase.client.java.document.JsonLongDocument;
import com.couchbase.client.java.document.json.JsonObject;
import com.couchbase.client.java.error.DocumentAlreadyExistsException;
import com.couchbase.client.java.error.DocumentDoesNotExistException;
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

    ObjectMapper om = new ObjectMapper();

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
        try {
            JsonLongDocument globalId = DAOManager.getCurrentBucket().counter("globalId", 1);
            Long newId = globalId.content();

            JsonDocument res = DAOManager.getCurrentBucket().insert(JsonDocument.create(Long.toString(newId), entityToJsonDocument(e)));

            return jsonDocumentToEntity(Long.valueOf(res.id()), res.content());
        } catch (DocumentAlreadyExistsException ex){
            return null;
        }
    }

    /**
     * Delete an entity
     * @param e
     */
    public final long delete(T e) {
        try {
            JsonDocument res = DAOManager.getCurrentBucket().remove("" + e.getId());
            return Long.valueOf(res.id());
        } catch (DocumentDoesNotExistException ex){
            return -1;
        }
    }

    /**
     * Update entity
     * fails with a DocumentDoesNotExistException if the object does not exist
     * @param e
     */
    public final T update(T e) {
        try {
            JsonDocument res = DAOManager.getCurrentBucket().replace(JsonDocument.create(Long.toString(e.getId()), entityToJsonDocument(e)));
            return jsonDocumentToEntity(Long.valueOf(res.id()), res.content());
        } catch (DocumentDoesNotExistException ex){
            return null;
        }
    }

    /**
     * GetAll
     * @return
     */
    public final List<T> getAll()
    {
        long begin = System.currentTimeMillis();
        createViewAll();
        List<ViewRow> result = DAOManager.getCurrentBucket().query(ViewQuery.from("designDoc", "by_type_" + type).stale(Stale.FALSE)).allRows();
        return viewRowsToEntities(result);
    }

    /**
     * GetById
     * @return
     */
    public final T getById(Long id)
    {
        try {
            String idString = Long.toString(id);
            JsonDocument res = DAOManager.getCurrentBucket().get(idString);
            if (null == res) {
                return null;
            } else {
                return jsonDocumentToEntity(Long.valueOf(res.id()), res.content());
            }
        } catch (DocumentDoesNotExistException ex){
            return null;
        }
    }

    public final List<T> getBy(String key, Object value){
        createViewBy(key);
        ViewQuery query = null;
        if (value instanceof Long){
            Long v = (Long) value;
            query = ViewQuery.from("designDoc", "by_" + key + "_" + type).startKey(v).stale(Stale.FALSE);
        } else if (value instanceof Integer){
            Integer v = (Integer) value;
            query = ViewQuery.from("designDoc", "by_" + key + "_" + type).startKey(v).stale(Stale.FALSE);
        } else {
            String v = (String) value;
            query = ViewQuery.from("designDoc", "by_" + key + "_" + type).startKey(v).stale(Stale.FALSE);
        }
        System.out.println(query);
        List<ViewRow> result = DAOManager.getCurrentBucket().query(query).allRows();
        return viewRowsToEntities(result);
    }

    protected List<T> viewRowsToEntities(List<ViewRow> list){
        List<T> res = new ArrayList<>();
        // Iterate through the returned ViewRows
        for (ViewRow row : list) {
            res.add(jsonDocumentToEntity(Long.valueOf(row.id()), (JsonObject) row.value()));
        }
        if (res.size() == 0){
            return null;
        }
        return res;
    }

    /**
     * Transform a jsonDocument to entity
     * @param jsonDocument document to transform
     * @return entity of JsonDocument
     */
    protected T jsonDocumentToEntity(long id, JsonObject jsonDocument){
        T entity = null;
        try {
            entity = om.readValue(jsonDocument.toString(), typeClass);
            entity.setId(id);
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
        JsonObject jsonObject = JsonObject.fromJson(json);
        jsonObject.removeKey("id");
        return jsonObject;
    }

    public T cloneEntity(T entity) {
        JsonObject json = entityToJsonDocument(entity);
        return jsonDocumentToEntity(entity.getId(), entityToJsonDocument(entity));
    }

    private void createViewAll()
    {
        DesignDocument designDoc = DAOManager.getCurrentBucket().bucketManager().getDesignDocument("designDoc");
        if (null == designDoc){
            designDoc = createDesignDocument();
        }

        String viewName = "by_type_"+type;
        if (!designDoc.toString().contains(viewName)) {
            String mapFunction =
                    "function (doc, meta) {\n" +
                            " if(doc.type && doc.type == '" + type + "') \n" +
                            "   { emit(doc.id, doc);}\n" +
                            "}";

            designDoc.views().add(DefaultView.create(viewName, mapFunction, ""));
            DAOManager.getCurrentBucket().bucketManager().upsertDesignDocument(designDoc);
        }
    }

    private void createViewBy(String key)
    {
        DesignDocument designDoc = DAOManager.getCurrentBucket().bucketManager().getDesignDocument("designDoc");
        if (null == designDoc){
            designDoc = createDesignDocument();
        }

        String viewName = "by_" + key + "_" + type;
        if (!designDoc.toString().contains(viewName)) {
            String mapFunction =
                    "function (doc, meta) {\n" +
                            " if(doc.type && doc.type == '" + type + "') \n" +
                            "   { emit(doc." + key + ", doc);}\n" +
                            "}";
            designDoc.views().add(DefaultView.create(viewName, mapFunction, ""));
            DAOManager.getCurrentBucket().bucketManager().upsertDesignDocument(designDoc);
        }
    }

    public DesignDocument createDesignDocument()
    {
        List<View> views = new ArrayList<>();
        DesignDocument designDoc = DesignDocument.create("designDoc", views);
        DAOManager.getCurrentBucket().bucketManager().insertDesignDocument(designDoc);
        return designDoc;
    }
}
