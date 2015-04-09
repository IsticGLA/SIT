package dao;

import com.couchbase.client.java.Bucket;
import com.couchbase.client.java.Cluster;
import com.couchbase.client.java.CouchbaseCluster;
import util.Configuration;

/**
 * Created by jeremy on 09/04/15.
 */
public class DAOManager {

    /**
     * Current Connection
     */
    protected static Cluster currentCluster;

    /**
     * Current Bucket
     */
    protected static Bucket currentBucket;

    /**
     * Instance of the DAOManager
     */
    private static DAOManager instance = new DAOManager();

    public static void connect(){
        if(currentCluster == null || currentBucket==null) {
            // Connect to a cluster
            currentCluster = CouchbaseCluster.create(Configuration.COUCHBASE_HOSTNAME);

            // Open a bucket
            currentBucket = currentCluster.openBucket(Configuration.BUCKET_NAME);
        }
    }

    public static void disconnect() {
        if(currentCluster != null)
        {
            currentCluster.disconnect();
            currentBucket =null;
        }
    }

    public static Bucket getCurrentBucket() {
        return currentBucket;
    }
}
