package istic.gla.groupeb.flerjeco.springRest;

import org.springframework.http.ResponseEntity;
import org.springframework.http.converter.json.MappingJackson2HttpMessageConverter;
import org.springframework.web.client.RestTemplate;

/**
 * Created by amhachi on 08/04/15.
 */
public class SpringService {

    private static final String URL = "http://localhost:8080/";

    public void codeSinistreClient() {

        final String url = URL + "rest/utl/to/code/sinistre/";

        RestTemplate restTemplate = new RestTemplate();
        restTemplate.getMessageConverters().add(new MappingJackson2HttpMessageConverter());

        ResponseEntity<IncidentCode[]> incidentCode = restTemplate.getForEntity(url, IncidentCode[].class);
        IncidentCode[] coordinates = incidentCode.getBody();

    }

}
