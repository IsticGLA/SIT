package util;

import com.couchbase.client.java.document.json.JsonArray;
import entity.Position;

/**
 * Created by corentin on 10/03/15.
 */
public class Tools {

    public static JsonArray positionToJsonArray(Position p)
    {
        JsonArray jsonArray = JsonArray.create();
        jsonArray.add(p.getLatitude());
        jsonArray.add(p.getLongitude());
        jsonArray.add(p.getAltitude());
        return jsonArray;
    }

    public static Position jsonArrayToPosition(JsonArray jsonArray)
    {
        Position p = new Position();
        p.setLatitude((Double) jsonArray.get(0));
        p.setLongitude((Double)jsonArray.get(1));
        p.setAltitude((Double)jsonArray.get(2));
        return p;
    }

    /*
    public static Zone jsonArrayToZone(JsonArray jsonArray) {
        Zone z = new Zone();
        for(int i=0; i<jsonArray.size();i++) {
            z.addPosition(Tools.jsonArrayToPosition((JsonArray) jsonArray.get(i)));
        }
        return z;
    }

    public static JsonArray zoneToJsonArray(Zone zone) {
        JsonArray array = JsonArray.create();
        for(Position p : zone.getPositions()) {
            array.add(Tools.positionToJsonArray(p));
        }
        return array;
    }

    public static List<Zone> jsonArrayToZoneList(JsonArray jsonArray) {
        List<Zone> z = new ArrayList<Zone>();
        for(int i=0; i<jsonArray.size();i++) {
            z.add(Tools.jsonArrayToZone((JsonArray) jsonArray.get(i)));
        }
        return z;
    }

    public static JsonArray zoneListToJsonArray(List<Zone> zones) {
        JsonArray array = JsonArray.create();
        for(Zone zone : zones) {
            array.add(Tools.zoneToJsonArray(zone));
        }
        return array;
    }
    */
}