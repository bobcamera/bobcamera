import tornado
import tornado.web
import tornado.websocket
from bob_interfaces.msg import TrackingState

class NoCacheStaticFileHandler(tornado.web.StaticFileHandler):

  def set_extra_headers(self, path):
    # Disable cache
    self.set_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')

class PrometheusMetricsHandler(tornado.web.RequestHandler):

  state:TrackingState = None

  def initialize(self):
    pass

  def get(self):
    self.write(self.get_metrics())
  
  #https://prometheus.io/docs/instrumenting/exposition_formats/#comments-help-text-and-type-information
  def get_metrics(self):

    if PrometheusMetricsHandler.state != None:
      metrics = {
        "bob_trackable_count": PrometheusMetricsHandler.state.trackable,
        "bob_alive_count": PrometheusMetricsHandler.state.alive,
        "bob_started_total": PrometheusMetricsHandler.state.started,
        "bob_ended_total": PrometheusMetricsHandler.state.ended
        }
    
    # Add additional metrics to the dictionary here:

    metric_str = ""
    for m in metrics:
      metric_str += f"{m} {metrics[m]}\n"

    return metric_str 
