package dao;


import com.couchbase.client.java.view.*;
import entity.Intervention;
import util.Constant;

import java.util.List;

/**
 * Created by vivien on 09/04/15.
 */
public class InterventionDAO extends AbstractDAO<Intervention> {
    public InterventionDAO() {
        this.typeClass = Intervention.class;
        this.type = Constant.TYPE_INTERVENTION;
    }

    public final List<Intervention> getWaitingResources(){
        createViewOnDemandResource();
        List<ViewRow> result = DAOManager.getCurrentBucket().query(ViewQuery.from("designDoc", "by_waiting_resource_" + type).stale(Stale.FALSE)).allRows();
        return viewRowsToEntities(result);
    }

    private void createViewOnDemandResource()
    {
        DesignDocument designDoc = DAOManager.getCurrentBucket().bucketManager().getDesignDocument("designDoc");
        if (null == designDoc){
            designDoc = createDesignDocument();
        }

        String viewName = "by_waiting_resource_" + type;
        if (!designDoc.toString().contains(viewName)) {
            String mapFunction =
                    "function (doc, meta) {\n" +
                            " if(doc.type && doc.type == '" + type + "') \n" +
                            "   for (var i in doc.resources){ \n" +
                            "       if (doc.resources[i].state == 'waiting'){ \n" +
                            "           emit(doc.id, doc);\n" +
                            "       } \n" +
                            "   } \n" +
                            "}";
            designDoc.views().add(DefaultView.create(viewName, mapFunction, ""));
            DAOManager.getCurrentBucket().bucketManager().upsertDesignDocument(designDoc);
        }
    }
}
