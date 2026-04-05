import lgpio

def _on_change(chip, gpio, level, timestamp):
    print(chip, gpio, level, timestamp)

# def _on_change_b(chip, gpio, level, timestamp):
#     print("b ", chip, gpio, level, timestamp)

chip = lgpio.gpiochip_open(4)

pio = [10, 9, 13, 19, 20, 21]
callb = []

for pin in pio:
    lgpio.gpio_claim_input(chip, pin)
    callb.append(lgpio.callback(chip, pin, lgpio.RISING_EDGE, _on_change))
# lgpio.gpio_claim_input(chip, 9)
# b_a = lgpio.callback(chip, 10, func=_on_change_a)
# b_a = lgpio.callback(chip, 9, func=_on_change_b)

# lgpio.gpio_claim_input(chip, 13)
# lgpio.gpio_claim_input(chip, 19)
# # b_a = lgpio.callback(chip, 13, func=_on_change_a)
# # b_a = lgpio.callback(chip, 19, func=_on_change_b)

# lgpio.gpio_claim_input(chip, 20)
# lgpio.gpio_claim_input(chip, 21)
# b_a = lgpio.callback(chip, 20, func=_on_change_a)
# b_a = lgpio.callback(chip, 21, func=_on_change_b)

try:
    while True:
        pass
        # a = []
        # for pin in pio:
        #     a.append(lgpio.gpio_read(chip, pin))
        # print(a)
except:
    for i in callb:
        i.cancel()