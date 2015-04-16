package istic.gla.groupb.nivimoju.init;

import istic.gla.goupb.nivimoju.drone.engine.DronePositionRefresherJob;
import org.apache.log4j.Logger;
import org.quartz.*;
import org.quartz.impl.StdSchedulerFactory;

import javax.servlet.ServletContextEvent;
import javax.servlet.ServletContextListener;

/**
 * Created by sacapuces on 15/04/15.
 */
public class JobInit implements ServletContextListener {
    Logger logger = Logger.getLogger(JobInit.class);

    @Override
    public void contextInitialized(ServletContextEvent servletContextEvent) {
        logger.info("initializing context");
        //JobDetail job = new JobDetail();
        //job.setName("dummyJobName");
        //job.setJobClass(HelloJob.class);
        JobDetail job = JobBuilder.newJob(DronePositionRefresherJob.class)
                .withIdentity("dummyJobName", "group1").build();

        //CronTrigger trigger = new CronTrigger();
        //trigger.setName("dummyTriggerName");
        //trigger.setCronExpression("0/5 * * * * ?");

        Trigger trigger = TriggerBuilder
                .newTrigger()
                .withIdentity("dummyTriggerName", "group1")
                .withSchedule(
                        CronScheduleBuilder.cronSchedule("0/1 * * * * ?"))
                .build();

        //schedule it
        try{
            Scheduler scheduler = new StdSchedulerFactory().getScheduler();
            scheduler.start();
            scheduler.scheduleJob(job, trigger);
        } catch (SchedulerException e){
            logger.error(e);
        }

    }

    @Override
    public void contextDestroyed(ServletContextEvent servletContextEvent) {

    }
}
