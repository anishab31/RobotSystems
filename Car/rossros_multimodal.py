import time
import concurrent.futures
from threading import Lock
from picarx_sensors import Motors, Sensors, Interpreters, Controllers
from rossros import Bus, ConsumerProducer, Producer, Consumer, Timer, Printer, runConcurrently

try:
    from ezblock import __reset_mcu__
    from ezblock import *

    __reset_mcu__()
    time.sleep(0.01)
except ImportError:
    print("Simulator")
    from sim_ezblock import *

import logging

logging_format = "%(asctime) s : %(message) s "
logging.basicConfig(level=logging.INFO)

from logdecorator import log_on_start, log_on_end, log_on_error


def multi(m, s, i, c, speed):
    sensor_delay = 0.2
    interpreter_delay = 0.2
    controller_delay = 0.2

    grayin_bus = Bus()
    grayout_bus = Bus()

    ultrasonic_bus = Bus() #ultrasonic producer/consumer?

    stop_sign = Bus(False)

    speed_bus = Bus(initial_message=speed)

    grayscale_producer = Producer(s.get_adc_value, grayin_bus, delay=sensor_delay, termination_busses=stop_sign,
                                  name="grayscale_producer")  #needs more testing, can be improved
    grayscale_cp = ConsumerProducer(i.get_grayscale_value, grayin_bus, grayout_bus, delay=interpreter_delay,
                                    termination_busses=stop_sign, name="grayscale_cp")
    grayscale_consumer = Consumer(c.line_following, (grayout_bus, speed_bus), delay=controller_delay,
                                  termination_busses=stop_sign, name="grayscale_consumer")
    logging.info("checkpoint")
    runConcurrently([grayscale_producer, grayscale_cp, grayscale_consumer])
