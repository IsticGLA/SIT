package dao;

import com.couchbase.client.java.Bucket;
import com.couchbase.client.java.Cluster;
import com.couchbase.client.java.CouchbaseCluster;
import com.couchbase.client.java.document.JsonDocument;
import com.couchbase.client.java.error.FlushDisabledException;
import com.couchbase.client.java.view.*;
import entity.AbstractEntity;
import util.Configuration;

import java.util.ArrayList;
import java.util.List;

/**
 * Abstract class for DAO
 * AbstractDAO provides methods for DAO
 * Also use for connect and disconnect
 */
public abstract class AbstractDAO<T extends AbstractEntity> {
    /**
     * Current Connection
     */
    protected Cluster currentCluster;

    /**
     * Current Bucket
     */
    protected Bucket currentBucket;

    /**
     * type of T
     */
    protected String type;

    /**
     * Connect to BDD and
     * @return Bucket to communicate with couchbase
     */
    public final void connect() {
        if(currentCluster == null || currentBucket==null) {
            // Connect to a cluster
            currentCluster = CouchbaseCluster.create(Configuration.COUCHBASE_HOSTNAME);

            // Open a bucket
            currentBucket = currentCluster.openBucket(Configuration.BUCKET_NAME);
        }
    }

    /**
     * Disconnect BDD
     */
    public final void disconnect() {
        if(currentCluster != null)
        {
            currentCluster.disconnect();
            currentBucket =null;
        }
    }

    /**
     * Create an entity
     * @param e entity to create
     */
    public final T create(T e) {
        currentBucket.get("globalId");
        Long newId = currentBucket.counter("globalId", 1).content();
        e.setId(newId);
        JsonDocument res = currentBucket.insert(entityToJsonDocument(e));

        return jsonDocumentToEntity(res);
    }

    /**
     * Delete an entity
     * @param e
     */
    public final T delete(T e) {
        JsonDocument res = currentBucket.remove("" + e.getId());
        return jsonDocumentToEntity(res);
    }

    /**
     * Update entity
     * fails with a DocumentDoesNotExistException if the object does not exist
     * @param e
     */
    public final T update(T e) {
        JsonDocument res = currentBucket.replace(entityToJsonDocument(e));
        return jsonDocumentToEntity(res);
    }

    /**
     * GetAll
     * @return
     */
    public final List<T> getAll()
    {
        List<T> res = new ArrayList<T>();
        createViewAll();
        ViewResult result = currentBucket.query(ViewQuery.from("designDoc", "by_type_" + type));
        // Iterate through the returned ViewRows
        for (ViewRow row : result) {
            System.out.println(row);
            res.add(jsonDocumentToEntity(row.document()));
        }
        return res;
    }

    /**
     * GetById
     * @return
     */
    public final T getById(String id)
    {
        System.out.println(id);
        JsonDocument res = currentBucket.get(id);
        return jsonDocumentToEntity(res);
    }

    /**
     * flush our bucket
     * @return
     */
    public boolean flush()
    {
        if(currentBucket!=null && currentCluster!=null)
        {
            try
            {
                return currentBucket.bucketManager().flush();
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
    protected abstract T jsonDocumentToEntity(JsonDocument jsonDocument);

    /**
     * Transform an entity to JsonDocument
     * @param entity to transform
     * @return jsonDoc of entity
     */
    protected abstract JsonDocument entityToJsonDocument(T entity);

    private void createViewAll()
    {
        DesignDocument designDoc = currentBucket.bucketManager().getDesignDocument("designDoc");
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
        currentBucket.bucketManager().upsertDesignDocument(designDoc);
    }

    public DesignDocument createDesignDocument()
    {
        List<View> views = new ArrayList<View>();
        DesignDocument designDoc = DesignDocument.create("designDoc", views);
        currentBucket.bucketManager().insertDesignDocument(designDoc);
        return designDoc;
    }
}
