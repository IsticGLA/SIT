package istic.gla.groupb.nivimoju.drone.client;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.ObjectReader;
import com.fasterxml.jackson.databind.ObjectWriter;
import entity.Path;
import entity.Position;
import istic.gla.groupb.nivimoju.drone.latlong.LatLongConverter;
import istic.gla.groupb.nivimoju.drone.latlong.LocalPath;
import org.apache.log4j.Logger;
import org.springframework.http.*;
import org.springframework.web.client.RestClientException;
import org.springframework.web.client.RestTemplate;

import java.io.IOException;
import java.io.InputStream;
import java.util.MissingResourceException;
import java.util.Properties;

/**
 * Client for a restservice to manipulate drones
 */
public class DroneClient {
    private static Logger logger = Logger.getLogger(DroneClient.class);

    private final String server;
    private RestTemplate rest;
    private HttpHeaders headers;
    private HttpStatus status;

    /**
     * Initialize the restclient
     * @throws IllegalStateException if oncfiguration file is missing or incomplete
     */
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

    /**
     * simple get method
     * @param uri the end of the uri
     * @return a String with the body of the response
     * @throws RestClientException
     */
    private String get(String uri) throws RestClientException{
        HttpEntity<String> requestEntity = new HttpEntity<>("", headers);
        ResponseEntity<String> responseEntity =
                rest.exchange(server + uri, HttpMethod.GET, requestEntity, String.class);
        status = responseEntity.getStatusCode();
        return responseEntity.getBody();
    }

    /**
     * simple post method
     * @param uri the end of the uri
     * @param json the json to post in the body
     * @return a String with the body of the response
     * @throws RestClientException
     */
    private String post(String uri, String json) throws RestClientException{
        HttpEntity<String> requestEntity = new HttpEntity<>(json, headers);
        ResponseEntity<String> responseEntity = rest.exchange(server + uri, HttpMethod.POST, requestEntity, String.class);
        status = responseEntity.getStatusCode();
        return responseEntity.getBody();
    }

    /**
     * order a specific drone to follow a new path
     * @param droneLabel the name of the drone
     * @param path the path to follow
     * @return true if the command was successful
     */
    public boolean postPath(String droneLabel, LocalPath path) {
        try {
            ObjectWriter ow = new ObjectMapper().writer().withDefaultPrettyPrinter();
            String json = ow.writeValueAsString(path);
            String res = post(droneLabel + "/path", json);
            if(status.equals(HttpStatus.OK)){
                return true;
            } else{
                logger.warn("request failed. status code " + status + " body : " + res);
            }
        } catch (JsonProcessingException e){
            logger.error("faile to parse path : " + path);
        }
        return false;
    }

    /**
     * Command a specific drone to stop
     * @param droneLabel the drone name
     * @return true if the command was succesfull
     */
    public boolean postStop(String droneLabel){
        String res = post(droneLabel + "/stop", null);
        if(status.equals(HttpStatus.OK)){
            return true;
        } else{
            logger.warn("request failed. status code " + status + " body : " + res);
        }
        return false;
    }

    /**
     * Get informations about all drones in simulation (positions)
     * @return all information in a DronesInfos object
     */
    public DronesInfos getDronesInfos() {
        String res = get("drones/info");
        if(status.equals(HttpStatus.OK)) {
            ObjectReader reader = new ObjectMapper().reader(DronesInfos.class);
            try {
                return reader.readValue(res);
            } catch (IOException e) {
                logger.error(e);
            }
        } else{
            logger.warn("request failed. status code " + status + " body : " + res);
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
        client.postPath("drone_1", converter.getLocalPath(path));
    }
}
