package istic.gla.groupb.nivimoju.util;

import java.util.Map;

/**
 * Created by alban on 12/03/15.
 */
public class Configuration {
    public static String BUCKET_NAME = "sit_bucket";
    public static String BUCKET_NAME_TEST = "test";
    public static String COUCHBASE_HOSTNAME = "37.59.58.42";

    /**
     * Load configurations
     * @param configs
     */
    public static void loadConfigurations(Map<String,String> configs)
    {
        BUCKET_NAME = configs.get("BUCKET_NAME");
        COUCHBASE_HOSTNAME = configs.get("COUCHBASE_HOSTNAME");
    }
}
