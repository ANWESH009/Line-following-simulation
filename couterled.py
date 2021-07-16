from controller import LED

led_list=['led_0','led_1','led_2']

for l in led_list:
    led[l] = robot.getDevice(l)


led['led_2'].set(1)
led['led_1'].set(1)
led['led_0'].set(1)