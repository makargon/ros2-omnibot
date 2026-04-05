import lgpio

def _on_change_a(chip, gpio, level, timestamp):
    print("a ", chip, gpio, level, timestamp)

def _on_change_b(chip, gpio, level, timestamp):
    print("b ", chip, gpio, level, timestamp)

chip = lgpio.gpiochip_open(4)

lgpio.gpio_claim_input(chip, 10)
lgpio.gpio_claim_input(chip, 9)
b_a = lgpio.callback(chip, 10, func=_on_change_a)
b_a = lgpio.callback(chip, 9, func=_on_change_b)

while True:
    pass
    # print()