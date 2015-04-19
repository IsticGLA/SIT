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
    public String post(String uri, String json) throws RestClientException{
        HttpEntity<String> requestEntity = new HttpEntity<>(json, headers);
        ResponseEntity<String> responseEntity = rest.exchange(server + uri, HttpMethod.POST, requestEntity, String.class);
        status = responseEntity.getStatusCode();
        return responseEntity.getBody();
    }

    /**
     * Donne un nouveau chemin � parcourir � un drone
     * @param droneLabel le label du drone � commander
     * @param path le chemin � suivre
     * @return true si l'ordre a �t� correctement envoy�
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
     * Ordonne � un drone de s'arr�ter
     * @param droneLabel le label du drone que l'on souhaite stopper
     * @return true si l'ordre a �t� pris en compte
     */
    public boolean postStop(String droneLabel) throws JsonProcessingException {
        try {
            ObjectWriter ow = new ObjectMapper().writer().withDefaultPrettyPrinter();
            String res = post(droneLabel + "/stop", null);
            if(status.equals(HttpStatus.OK)){
                return true;
            } else{
                logger.warn("request failed. status code " + status + " body : " + res);
            }
        } catch (Exception e){
            logger.error(e);
        }
        return false;
    }

    /**
     * R�cup�re les informations des drones
     * @return un objet DroneInfos ou null en cas d'erreur
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
        client.postPath("drone_12/c/", converter.getLocalPath(path));
    }
}
