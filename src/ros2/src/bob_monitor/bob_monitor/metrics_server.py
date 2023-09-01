import asyncio
import os
import tornado, tornado.web, tornado.websocket
from ament_index_python.packages import get_package_share_directory
from .handlers import NoCacheStaticFileHandler, PrometheusMetricsHandler

#https://github.com/dheera/rosboard/blob/main/rosboard/rosboard.py

class MetricsServer():

  def __init__(self, port=8082):

    tornado_settings = {
      'debug': True, 
      'static_path': os.path.join(get_package_share_directory('bob_monitor'), 'static')
    }

    tornado_handlers = [
      (r"/metrics", PrometheusMetricsHandler, { }),
      (r"/(.*)", NoCacheStaticFileHandler, {
        "path": tornado_settings.get("static_path"),
        "default_filename": "index.html"
      }),
    ]

    self.tornado_application = tornado.web.Application(tornado_handlers, **tornado_settings)
    self.port = port    

  def start(self):
    self.tornado_application.listen(self.port)
    