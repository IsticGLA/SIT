package dao;

import com.couchbase.client.java.document.JsonDocument;
import com.couchbase.client.java.document.json.JsonObject;
import entity.Intervention;
import util.Constant;
import util.Tools;

/**
 * Created by corentin on 13/03/15.
 */
public class InterventionDAO extends AbstractDAO<Intervention>{

    /**
     * Contructeur UnityDAO
     */
    public InterventionDAO()
    {
        this.datatype = Constant.DATATYPE_GEOINTERVENTIONZONE;
    }

    @Override
    protected Intervention jsonDocumentToEntity(JsonDocument jsonDocument) {
        Intervention geo = new Intervention();

        try {
            JsonObject content = jsonDocument.content();
            if (Constant.DATATYPE_GEOINTERVENTIONZONE.equals(((JsonObject) content.get("properties")).get("type"))) {
                geo.setId(Long.parseLong(jsonDocument.id()));
                geo.setCoordinates(Tools.jsonArrayToZoneList(content.getArray("coordinates")));
            } else {
                throw new IllegalArgumentException();
            }
        }
        catch(Throwable t)
        {
            geo = null;
        }
        return geo;
    }

    @Override
    protected JsonDocument entityToJsonDocument(Intervention entity) {
        JsonObject properties = JsonObject.create();
        properties.put("type", entity.getDataType());
        JsonObject jsonGeoInterventionZone = JsonObject.empty()
                .put("type","Polygon")
                .put("coordinates", Tools.zoneListToJsonArray(entity.getCoordinates()))
                .put("properties", properties);
        JsonDocument doc = JsonDocument.create("" + entity.getId(), jsonGeoInterventionZone);
        System.out.println(jsonGeoInterventionZone);
        return doc;
    }
}
