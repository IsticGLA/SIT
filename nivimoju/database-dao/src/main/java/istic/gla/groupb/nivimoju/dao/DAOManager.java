package istic.gla.groupb.nivimoju.dao;

import com.couchbase.client.core.CouchbaseException;
import com.couchbase.client.java.Bucket;
import com.couchbase.client.java.Cluster;
import com.couchbase.client.java.CouchbaseCluster;
import com.couchbase.client.java.error.FlushDisabledException;
import istic.gla.groupb.nivimoju.util.Configuration;

import java.util.NoSuchElementException;
import java.util.concurrent.TimeUnit;

/**
 * Created by jeremy on 09/04/15.
 */
public class DAOManager {

    /**
     * Current Connection
     */
    protected static Cluster currentCluster = CouchbaseCluster.create(Configuration.COUCHBASE_HOSTNAME);

    /**
     * Instance of the DAOManager
     */
    private static DAOManager instance = new DAOManager();

    /**
     * Current Bucket
     */
    protected static Bucket currentBucket;

    public static void connect(){
        try {
            if (currentCluster != null && currentBucket==null) {
                // Open a bucket
                currentBucket = currentCluster.openBucket(Configuration.BUCKET_NAME);
            }
        } catch (CouchbaseException ex) {
            currentCluster.disconnect();
            currentCluster = CouchbaseCluster.create(Configuration.COUCHBASE_HOSTNAME);
            // Open a bucket
            currentBucket = null;
        }
    }

    public static void connectTest(){
        try {
            if (currentCluster != null && currentBucket==null) {
                // Open a bucket
                currentBucket = currentCluster.openBucket(Configuration.BUCKET_NAME_TEST);
            }
        } catch (CouchbaseException ex) {
            currentCluster.disconnect();
            currentCluster = CouchbaseCluster.create(Configuration.COUCHBASE_HOSTNAME);
            // Open a bucket
            currentBucket = null;
        }
    }

    public static void disconnect() {
        try {
            if (currentCluster != null) {
                if (currentBucket != null && currentBucket.close(2, TimeUnit.SECONDS)) {
                    currentBucket = null;
                }
            }
        }
        catch (NoSuchElementException ex) {
            currentBucket = null;
        }
    }


    /**
     * flush our bucket
     * @return
     */
    public static boolean flush()
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

    public static Bucket getCurrentBucket() {
        return currentBucket;
    }
}
