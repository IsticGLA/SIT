import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.ObjectWriter;
import istic.gla.groupb.nivimoju.drone.latlong.LatLong;
import istic.gla.groupb.nivimoju.drone.latlong.LatLongConverter;
import istic.gla.groupb.nivimoju.drone.latlong.LocalCoordinate;
import org.apache.log4j.Logger;
import org.springframework.http.*;
import org.springframework.web.client.RestTemplate;

public class DroneClient {
    private static Logger logger = Logger.getLogger(DroneClient.class);

    private String server = "http://37.59.58.42:5000/";
    private RestTemplate rest;
    private HttpHeaders headers;
    private HttpStatus status;

    public DroneClient() {
        this.rest = new RestTemplate();
        this.headers = new HttpHeaders();
        headers.add("Content-Type", "application/json");
        headers.add("Accept", "*/*");
    }

    public String get(String uri) {
        HttpEntity<String> requestEntity = new HttpEntity<>("", headers);
        ResponseEntity<String> responseEntity = rest.exchange(server + uri, HttpMethod.GET, requestEntity, String.class);
        this.setStatus(responseEntity.getStatusCode());
        return responseEntity.getBody();
    }

    public String post(String uri, String json) {
        logger.info("sending post request on " + server+uri + " with body \n" + json);
        HttpEntity<String> requestEntity = new HttpEntity<>(json, headers);
        ResponseEntity<String> responseEntity = rest.exchange(server + uri, HttpMethod.POST, requestEntity, String.class);
        this.setStatus(responseEntity.getStatusCode());
        return responseEntity.getBody();
    }

    public void put(String uri, String json) {
        HttpEntity<String> requestEntity = new HttpEntity<>(json, headers);
        ResponseEntity<String> responseEntity = rest.exchange(server + uri, HttpMethod.PUT, requestEntity, null);
        this.setStatus(responseEntity.getStatusCode());
    }

    public void delete(String uri) {
        HttpEntity<String> requestEntity = new HttpEntity<>("", headers);
        ResponseEntity<String> responseEntity = rest.exchange(server + uri, HttpMethod.DELETE, requestEntity, null);
        this.setStatus(responseEntity.getStatusCode());
    }

    public HttpStatus getStatus() {
        return status;
    }

    public void setStatus(HttpStatus status) {
        this.status = status;
    }

    private void postWaypoint(LocalCoordinate local) throws Exception {
        local.setZ(5);
        ObjectWriter ow = new ObjectMapper().writer().withDefaultPrettyPrinter();
        String json = ow.writeValueAsString(local);
        String res = post("robot/waypoint", json);
        logger.info(res);
    }

    public static void main(String[] args) throws Exception{
        DroneClient client = new DroneClient();
        LatLongConverter converter = new LatLongConverter(48.1222, -1.6428, 48.1119, -1.6337, 720, 1200);
        LatLong center = new LatLong((48.1222+48.1119)/2, (-1.6428+-1.6337)/2);
        logger.info("center : "+ center );
        LatLong piscine = new LatLong(48.115367,-1.63781);
        logger.info("piscine : "+ piscine );
        LatLong croisement = new LatLong(48.11498, -1.63795);

        LocalCoordinate local = converter.getLocal(croisement);
        logger.info("local" + local);
        client.postWaypoint(local);
    }
}
