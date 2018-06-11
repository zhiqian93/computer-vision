from apscheduler.schedulers.blocking import BlockingScheduler
import CV
sched = BlockingScheduler()
# __main__ = "HH detection"
@sched.scheduled_job('cron', day_of_week='mon-fri', hour='9', minute = '00')
def scheduled_job():
    CV.job()

sched.start()