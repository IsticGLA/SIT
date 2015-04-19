package istic.gla.groupb.nivimoju.drone.client;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.ObjectReader;
import com.fasterxml.jackson.databind.ObjectWriter;
import entity.Drone;
import entity.Path;
import entity.Position;
import istic.gla.groupb.nivimoju.drone.latlong.LatLongConverter;
import istic.gla.groupb.nivimoju.drone.latlong.LocalCoordinate;
import istic.gla.groupb.nivimoju.drone.latlong.LocalPath;
import org.apache.log4j.Logger;
import org.springframework.http.*;
import org.springframework.web.client.RestClientException;
import org.springframework.web.client.RestTemplate;

import java.io.IOException;
import java.io.InputStream;
import java.util.MissingResourceException;
import java.util.Properties;
import java.util.Map;

public class DroneClient {
    private static Logger logger = Logger.getLogger(DroneClient.class);

    private final String server;
    private RestTemplate rest;
    private HttpHeaders headers;
    private HttpStatus status;

    public DroneClient() throws IllegalStateException{
        this.rest = new RestTemplate();
        this.headers = new HttpHeaders();
        headers.add("Content-Type", "application/json");
        headers.add("Accept", "*/*");

        Properties prop = new Properties();
        InputStream input = null;
        try {
            String filename = "config.properties";
            input = DroneClient.class.getClassLoader().getResourceAsStream(filename);
            if(input==null){
                logger.error("Could not find property file for drone-simulation-client : " + filename);
                throw new MissingResourceException("Could not find property file for drone-simulation-client : ", DroneClient.class.getName(), "config.properties");
            }
            //load a properties file from class path, inside static method
            prop.load(input);
            if (prop.getProperty("flask_ip") == null) {
                throw new MissingResourceException("could not find property 'flask_ip', unable to initialize client", DroneClient.class.getName(), "flask_ip");
            }
            if (prop.getProperty("flask_port") == null) {
                throw new MissingResourceException("could not find property 'flask_port', unable to initialize client", DroneClient.class.getName(), "flask_port");
            }
            server = "http://"+prop.getProperty("flask_ip") + ":" + prop.getProperty("flask_port") +"/";
            logger.info("initialized flask server client at " + server);
        } catch (IOException e) {
            logger.error("Could not read property file for drone-simulation-client : ", e);
            throw new MissingResourceException("Could not read property file for drone-simulation-client : ", DroneClient.class.getName(), "config.properties");
        } finally{
            if(input!=null){
                try {
                    input.close();
                } catch (IOException e) {
                    logger.error(e);
                }
            }
        }


    }

    public String get(String uri) {
        HttpEntity<String> requestEntity = new HttpEntity<>("", headers);
        ResponseEntity<String> responseEntity =
                rest.exchange(server + uri, HttpMethod.GET, requestEntity, String.class);
        this.setStatus(responseEntity.getStatusCode());
        return responseEntity.getBody();
    }

    public String post(String uri, String json) {
        HttpEntity<String> requestEntity = new HttpEntity<>(json, headers);
        try {
            ResponseEntity<String> responseEntity = rest.exchange(server + uri, HttpMethod.POST, requestEntity, String.class);
            this.setStatus(responseEntity.getStatusCode());
            return responseEntity.getBody();
        } catch (RestClientException e){
            logger.error("failed to post message", e);
        }
        return null;
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

    public void postWaypoint(LocalCoordinate local) throws Exception {
        local.setZ(20);
        ObjectWriter ow = new ObjectMapper().writer().withDefaultPrettyPrinter();
        String json = ow.writeValueAsString(local);
        String res = post("robot/waypoint", json);
    }

    public void postPath(String droneLabel, LocalPath path) throws JsonProcessingException {
        ObjectWriter ow = new ObjectMapper().writer().withDefaultPrettyPrinter();
        String json = ow.writeValueAsString(path);
        try {
            String res = post(droneLabel + "/path", json);
        } catch (Exception e){
            logger.error(e);
        }
    }

    public void postStop(String droneLabel) throws JsonProcessingException {
        ObjectWriter ow = new ObjectMapper().writer().withDefaultPrettyPrinter();
        try {
            String res = post(droneLabel + "/stop", null);
        } catch (Exception e){
            logger.error(e);
        }
    }

    public DronesInfos getDronesInfos() {
        String res = get("drones/info");
        //map label localposition
        Map<String, LocalCoordinate> infos;
        ObjectReader reader = new ObjectMapper().reader(DronesInfos.class);
        try {
            return reader.readValue(res);
        } catch (IOException e) {
            logger.error(e);
        }
        return null;
    }

    public static void main(String[] args) {
        DroneClient client = new DroneClient();
        Position croisement = new Position(48.11498, -1.63795);
        Position croisement2 = new Position(48.114454, -1.639962);
        Path path = new Path();
        path.addPosition(croisement);
        path.addPosition(croisement2);
        path.setClosed(true);
        LatLongConverter converter = new LatLongConverter(48.1222, -1.6428, 48.1119, -1.6337, 720, 1200);
        try {
            client.postPath("drone_1", converter.getLocalPath(path));
        } catch (JsonProcessingException e) {
            e.printStackTrace();
        }
    }
}
