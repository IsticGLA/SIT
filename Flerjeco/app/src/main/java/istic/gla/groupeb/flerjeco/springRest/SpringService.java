package istic.gla.groupeb.flerjeco.springRest;

import android.util.Log;

import org.apache.http.HttpStatus;
import org.apache.http.util.ExceptionUtils;
import org.springframework.http.HttpEntity;
import org.springframework.http.HttpMethod;
import org.springframework.http.ResponseEntity;
import org.springframework.http.converter.json.MappingJackson2HttpMessageConverter;
import org.springframework.web.client.HttpStatusCodeException;
import org.springframework.web.client.RestTemplate;

import java.util.Random;

import entity.IncidentCode;
import entity.Intervention;
import entity.ResourceType;

/**
 * Created by amhachi on 08/04/15.
 */
public class SpringService {
    private static final String TAG = SpringService.class.getSimpleName();
    private static final String URL = "http://ns3002211.ip-37-59-58.eu:8080/nivimoju/rest/";

    boolean test = true;
    public IncidentCode[]  codeSinistreClient() throws HttpStatusCodeException{


        final String url = URL + "incidentcode";


            RestTemplate restTemplate = new RestTemplate();
            restTemplate.getMessageConverters().add(new MappingJackson2HttpMessageConverter());

            ResponseEntity<IncidentCode[]> incidentCode = restTemplate.getForEntity(url, IncidentCode[].class);
            IncidentCode[] codes = incidentCode.getBody();
            return codes;


    }

    public Boolean  postInterventionTest(Intervention intervention) throws HttpStatusCodeException{

        return  true;
    }

    public Boolean  postIntervention(Intervention intervention){
       try {

           final String url = URL + intervention.getLatitude()+"/"+intervention.getLongitude()+"/"+intervention.getIncidentCode();

           RestTemplate restTemplate = new RestTemplate();
           restTemplate.getMessageConverters().add(new MappingJackson2HttpMessageConverter());

           restTemplate.postForObject(url, intervention, Intervention.class);

           ResponseEntity<IncidentCode[]> incidentCode = restTemplate.getForEntity(url, IncidentCode[].class);
           IncidentCode[] codes = incidentCode.getBody();

           return true;
       }catch (HttpStatusCodeException e){
           return  false;
       }
    }

    public String login(String id, String password) {
        Log.i(TAG, "login start");
        final String url = URL + "authentication/connected/"+id+"/"+password;
        String httpResult = "";

        RestTemplate restTemplate = new RestTemplate();
        restTemplate.getMessageConverters().add(new MappingJackson2HttpMessageConverter());

        try {
            ResponseEntity<String> entity = restTemplate.getForEntity(url, String.class);
            httpResult = entity.getStatusCode().toString();
        } catch (HttpStatusCodeException e) {
            httpResult = "400";
        }
        Log.i(TAG, "login end");
        return httpResult;
    }

    public ResourceType[] resourceTypes() {
        final String url = URL + "resource";

        RestTemplate restTemplate = new RestTemplate();
        restTemplate.getMessageConverters().add(new MappingJackson2HttpMessageConverter());

        ResponseEntity<ResourceType[]> resourceTypes = restTemplate.getForEntity(url, ResourceType[].class);
        return resourceTypes.getBody();
    }

    public Long requestVehicle(Long[] params) {
        final String url = URL + "intervention/" + params[0] + "/resources/" + params[1];
        RestTemplate restTemplate = new RestTemplate();
        restTemplate.getMessageConverters().add(new MappingJackson2HttpMessageConverter());

        ResponseEntity<Long> requestId = restTemplate.exchange(url, HttpMethod.PUT, null, Long.class);
        return requestId.getBody();
    }
}
