import time
import main

loop_delay = 0.0
context = None

def run():
    main.context = context
    main.setup()
    while True:
        main.loop()
        time.sleep(loop_delay)
