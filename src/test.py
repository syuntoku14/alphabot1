from AlphaBot import AlphaBot
import time

bot = AlphaBot()

for i in range(5):
    bot.set_wheel_speeds(1, 3)
    time.sleep(5)
